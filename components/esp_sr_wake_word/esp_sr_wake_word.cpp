#include "esp_sr_wake_word.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <esp_wn_models.h>

namespace esphome {
namespace esp_sr {

static const char *const TAG = "esp_sr_wake_word";

static const ssize_t DETECTION_QUEUE_LENGTH = 5;
static const size_t DATA_TIMEOUT_MS = 50;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t INFERENCE_TASK_STACK_SIZE = 8192;
static const UBaseType_t INFERENCE_TASK_PRIORITY = 3;

enum EventGroupBits : uint32_t {
  COMMAND_STOP = (1 << 0),

  TASK_STARTING = (1 << 3),
  TASK_RUNNING = (1 << 4),
  TASK_STOPPING = (1 << 5),
  TASK_STOPPED = (1 << 6),

  ERROR_MEMORY = (1 << 9),
  ERROR_MODEL_PARTITION = (1 << 10),
  ERROR_MODEL_LOAD = (1 << 11),

  WARNING_FULL_RING_BUFFER = (1 << 13),

  ERROR_BITS = ERROR_MEMORY | ERROR_MODEL_PARTITION | ERROR_MODEL_LOAD,
  ALL_BITS = 0xfffff,
};

float EspSrWakeWord::get_setup_priority() const {
  return setup_priority::AFTER_CONNECTION;
}

static const LogString *esp_sr_wake_word_state_to_string(State state) {
  switch (state) {
    case State::STARTING:
      return LOG_STR("STARTING");
    case State::DETECTING_WAKE_WORD:
      return LOG_STR("DETECTING_WAKE_WORD");
    case State::STOPPING:
      return LOG_STR("STOPPING");
    case State::STOPPED:
      return LOG_STR("STOPPED");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void EspSrWakeWordModel::log_model_config() const {
  ESP_LOGCONFIG(TAG, "    %s:", this->id_.c_str());
  ESP_LOGCONFIG(TAG, "      Wake word: %s", this->wake_word_.c_str());
  ESP_LOGCONFIG(TAG, "      Model: %s", this->model_dir_.c_str());
  ESP_LOGCONFIG(TAG, "      Enabled: %s", YESNO(this->enabled_));
  ESP_LOGCONFIG(TAG, "      Internal only: %s", YESNO(this->internal_only_));
}

bool EspSrWakeWordModel::load_model(srmodel_list_t *models) {
  if (!this->enabled_)
    return true;

  char *model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, this->model_dir_.c_str());
  if (model_name == nullptr) {
    ESP_LOGE(TAG, "Model '%s' was not found in the model partition", this->model_dir_.c_str());
    return false;
  }

  this->iface_ = static_cast<const esp_wn_iface_t *>(esp_wn_handle_from_name(model_name));
  if (this->iface_ == nullptr) {
    ESP_LOGE(TAG, "Failed to get WakeNet handle for model '%s'", this->model_dir_.c_str());
    return false;
  }

  this->data_ = this->iface_->create(model_name, DET_MODE_90);
  if (this->data_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create WakeNet instance for model '%s'", this->model_dir_.c_str());
    this->iface_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "Loaded '%s' with %d samples per chunk", this->model_dir_.c_str(), this->get_samples_per_chunk());
  return true;
}

void EspSrWakeWordModel::unload_model() {
  if (this->iface_ != nullptr && this->data_ != nullptr)
    this->iface_->destroy(this->data_);
  this->iface_ = nullptr;
  this->data_ = nullptr;
}

int EspSrWakeWordModel::get_samples_per_chunk() const {
  if (this->iface_ == nullptr || this->data_ == nullptr)
    return 0;
  return this->iface_->get_samp_chunksize(this->data_);
}

bool EspSrWakeWordModel::detect(int16_t *audio_buffer, DetectionEvent *event) {
  if (!this->enabled_ || this->iface_ == nullptr || this->data_ == nullptr)
    return false;

  const int result = static_cast<int>(this->iface_->detect(this->data_, audio_buffer));
  if (result <= 0)
    return false;

  const char *detected_word = this->iface_->get_word_name(this->data_, result);
  if (detected_word != nullptr && detected_word[0] != '\0')
    this->wake_word_ = detected_word;

  event->wake_word = &this->wake_word_;
  return true;
}

void EspSrWakeWordModel::enable() {
  if (this->parent_ != nullptr) {
    this->parent_->enable_model(this);
    return;
  }
  this->enabled_ = true;
}

void EspSrWakeWordModel::disable() {
  if (this->parent_ != nullptr) {
    this->parent_->disable_model(this);
    return;
  }
  this->enabled_ = false;
}

void EspSrWakeWord::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP-SR Wake Word:");
  ESP_LOGCONFIG(TAG, "  Models:");
  for (auto *model : this->wake_word_models_)
    model->log_model_config();
  ESP_LOGCONFIG(TAG, "  Stop after detection: %s", YESNO(this->stop_after_detection_));
}

void EspSrWakeWord::setup() {
  this->event_group_ = xEventGroupCreate();
  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  this->detection_queue_ = xQueueCreate(DETECTION_QUEUE_LENGTH, sizeof(DetectionEvent));
  if (this->detection_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create detection event queue");
    this->mark_failed();
    return;
  }

  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    if (this->state_ == State::STOPPED)
      return;

    std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
    if (this->ring_buffer_.use_count() > 1) {
      if (temp_ring_buffer->free() < data.size()) {
        xEventGroupSetBits(this->event_group_, EventGroupBits::WARNING_FULL_RING_BUFFER);
        temp_ring_buffer->reset();
      }
      temp_ring_buffer->write((void *)data.data(), data.size());
    }
  });

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
}

#ifdef USE_OTA_STATE_LISTENER
void EspSrWakeWord::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->suspend_task_();
  } else if (state == ota::OTA_ERROR) {
    this->resume_task_();
  }
}
#endif

void EspSrWakeWord::inference_task(void *params) {
  auto *this_esp_sr = static_cast<EspSrWakeWord *>(params);
  xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::TASK_STARTING);

  srmodel_list_t *models = nullptr;

  {
    std::unique_ptr<audio::AudioSourceTransferBuffer> audio_buffer;
    std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer;

    if (!(xEventGroupGetBits(this_esp_sr->event_group_) & ERROR_BITS)) {
      temp_ring_buffer = ring_buffer::RingBuffer::create(
          this_esp_sr->microphone_source_->get_audio_stream_info().ms_to_bytes(RING_BUFFER_DURATION_MS));
      if (temp_ring_buffer.use_count() == 0) {
        xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::ERROR_MEMORY);
      } else {
        this_esp_sr->ring_buffer_ = temp_ring_buffer;
      }
    }

    if (!(xEventGroupGetBits(this_esp_sr->event_group_) & ERROR_BITS)) {
      models = esp_srmodel_init("model");
      if (models == nullptr)
        xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::ERROR_MODEL_PARTITION);
    }

    int samples_per_chunk = 0;
    if (!(xEventGroupGetBits(this_esp_sr->event_group_) & ERROR_BITS) &&
        !this_esp_sr->load_models_(models, &samples_per_chunk)) {
      xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::ERROR_MODEL_LOAD);
    }

    if (!(xEventGroupGetBits(this_esp_sr->event_group_) & ERROR_BITS)) {
      const size_t bytes_per_chunk = static_cast<size_t>(samples_per_chunk) * sizeof(int16_t);
      audio_buffer = audio::AudioSourceTransferBuffer::create(bytes_per_chunk);
      if (audio_buffer == nullptr) {
        xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::ERROR_MEMORY);
      } else {
        audio_buffer->set_source(temp_ring_buffer);
      }
    }

    if (!(xEventGroupGetBits(this_esp_sr->event_group_) & ERROR_BITS)) {
      const size_t bytes_per_chunk = static_cast<size_t>(samples_per_chunk) * sizeof(int16_t);
      this_esp_sr->microphone_source_->start();
      xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::TASK_RUNNING);

      while (!(xEventGroupGetBits(this_esp_sr->event_group_) & COMMAND_STOP)) {
        audio_buffer->transfer_data_from_source(pdMS_TO_TICKS(DATA_TIMEOUT_MS));

        if (audio_buffer->available() < bytes_per_chunk)
          continue;

        this_esp_sr->process_audio_(reinterpret_cast<int16_t *>(audio_buffer->get_buffer_start()));
        audio_buffer->decrease_buffer_length(bytes_per_chunk);
      }
    }
  }

  xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::TASK_STOPPING);

  this_esp_sr->unload_models_();
  if (models != nullptr)
    esp_srmodel_deinit(models);
  this_esp_sr->microphone_source_->stop();

  xEventGroupSetBits(this_esp_sr->event_group_, EventGroupBits::TASK_STOPPED);
  while (true)
    delay(10);
}

std::vector<EspSrWakeWordModel *> EspSrWakeWord::get_wake_words() {
  std::vector<EspSrWakeWordModel *> external_wake_word_models;
  for (auto *model : this->wake_word_models_) {
    if (!model->get_internal_only())
      external_wake_word_models.push_back(model);
  }
  return external_wake_word_models;
}

void EspSrWakeWord::add_wake_word_model(EspSrWakeWordModel *model) {
  model->set_parent(this);
  this->wake_word_models_.push_back(model);
}

void EspSrWakeWord::suspend_task_() {
  if (this->inference_task_handle_ != nullptr)
    vTaskSuspend(this->inference_task_handle_);
}

void EspSrWakeWord::resume_task_() {
  if (this->inference_task_handle_ != nullptr)
    vTaskResume(this->inference_task_handle_);
}

void EspSrWakeWord::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & EventGroupBits::ERROR_MEMORY) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_MEMORY);
    ESP_LOGE(TAG, "Encountered an error allocating buffers");
  }

  if (event_group_bits & EventGroupBits::ERROR_MODEL_PARTITION) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_MODEL_PARTITION);
    ESP_LOGE(TAG, "ESP-SR model partition could not be loaded. Is srmodels.bin flashed to the 'model' partition?");
  }

  if (event_group_bits & EventGroupBits::ERROR_MODEL_LOAD) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_MODEL_LOAD);
    ESP_LOGE(TAG, "Encountered an error while loading wake word models");
  }

  if (event_group_bits & EventGroupBits::WARNING_FULL_RING_BUFFER) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::WARNING_FULL_RING_BUFFER);
    ESP_LOGW(TAG, "Not enough free bytes in ring buffer to store incoming audio data. Resetting the ring buffer. Wake "
                  "word detection accuracy will temporarily be reduced.");
  }

  if (event_group_bits & EventGroupBits::TASK_STARTING) {
    ESP_LOGD(TAG, "Inference task has started, attempting to allocate memory for buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STARTING);
  }

  if (event_group_bits & EventGroupBits::TASK_RUNNING) {
    ESP_LOGD(TAG, "Inference task is running");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_RUNNING);
    this->set_state_(State::DETECTING_WAKE_WORD);
  }

  if (event_group_bits & EventGroupBits::TASK_STOPPING) {
    ESP_LOGD(TAG, "Inference task is stopping, deallocating buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STOPPING);
  }

  if (event_group_bits & EventGroupBits::TASK_STOPPED) {
    ESP_LOGD(TAG, "Inference task is finished, freeing task resources");
    vTaskDelete(this->inference_task_handle_);
    this->inference_task_handle_ = nullptr;
    xEventGroupClearBits(this->event_group_, ALL_BITS);
    xQueueReset(this->detection_queue_);
    this->set_state_(State::STOPPED);
    this->apply_pending_model_updates_();
    if (this->pending_restart_after_stop_) {
      this->pending_restart_after_stop_ = false;
      if (this->has_enabled_model_())
        this->pending_start_ = true;
    }
  }

  if (this->pending_start_ && this->state_ == State::STOPPED) {
    this->apply_pending_model_updates_();
    if (!this->has_enabled_model_()) {
      ESP_LOGW(TAG, "Wake word detection can't start because no model is enabled");
      this->pending_start_ = false;
      return;
    }
    this->set_state_(State::STARTING);
    this->pending_start_ = false;
  }

  if (this->pending_stop_ && this->state_ == State::DETECTING_WAKE_WORD) {
    this->set_state_(State::STOPPING);
    this->pending_stop_ = false;
  }

  switch (this->state_) {
    case State::STARTING:
      if (this->inference_task_handle_ == nullptr && !this->status_has_error()) {
        xTaskCreate(EspSrWakeWord::inference_task, "esp_sr_ww", INFERENCE_TASK_STACK_SIZE, (void *)this,
                    INFERENCE_TASK_PRIORITY, &this->inference_task_handle_);

        if (this->inference_task_handle_ == nullptr)
          this->status_momentary_error("task_start", 1000);
      }
      break;

    case State::DETECTING_WAKE_WORD: {
      DetectionEvent detection_event;
      while (xQueueReceive(this->detection_queue_, &detection_event, 0)) {
        ESP_LOGD(TAG, "Detected '%s'", detection_event.wake_word->c_str());
        this->wake_word_detected_trigger_.trigger(*detection_event.wake_word);
        if (this->stop_after_detection_)
          this->stop();
      }
      break;
    }

    case State::STOPPING:
      xEventGroupSetBits(this->event_group_, EventGroupBits::COMMAND_STOP);
      break;

    case State::STOPPED:
      break;
  }
}

void EspSrWakeWord::start() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Wake word detection can't start as the component hasn't been setup yet");
    return;
  }

  if (this->is_failed()) {
    ESP_LOGW(TAG, "Wake word component is marked as failed. Please check setup logs");
    return;
  }

  if (this->is_running()) {
    ESP_LOGW(TAG, "Wake word detection is already running");
    return;
  }

  ESP_LOGD(TAG, "Starting wake word detection");
  this->pending_start_ = true;
  this->pending_stop_ = false;
}

void EspSrWakeWord::stop() {
  if (this->state_ == State::STOPPED)
    return;

  ESP_LOGD(TAG, "Stopping wake word detection");
  this->pending_start_ = false;
  this->pending_stop_ = true;
}

void EspSrWakeWord::enable_model(EspSrWakeWordModel *model) {
  if (model == nullptr)
    return;

  ESP_LOGD(TAG, "Selecting wake word model '%s'", model->get_model_dir().c_str());
  this->pending_model_enable_ = model;
  this->pending_model_disable_ = nullptr;

  if (this->is_running()) {
    this->pending_restart_after_stop_ = true;
    this->stop();
  } else {
    this->apply_pending_model_updates_();
  }
}

void EspSrWakeWord::disable_model(EspSrWakeWordModel *model) {
  if (model == nullptr)
    return;

  ESP_LOGD(TAG, "Disabling wake word model '%s'", model->get_model_dir().c_str());
  this->pending_model_disable_ = model;
  if (this->is_running() && model->is_enabled())
    this->stop();
  else
    this->apply_pending_model_updates_();
}

void EspSrWakeWord::set_state_(State state) {
  if (this->state_ != state) {
    ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(esp_sr_wake_word_state_to_string(this->state_)),
             LOG_STR_ARG(esp_sr_wake_word_state_to_string(state)));
    this->state_ = state;
  }
}

bool EspSrWakeWord::has_enabled_model_() const {
  for (auto *model : this->wake_word_models_) {
    if (model->is_enabled())
      return true;
  }
  return false;
}

void EspSrWakeWord::apply_pending_model_updates_() {
  if (this->pending_model_enable_ != nullptr) {
    this->select_model_(this->pending_model_enable_);
    this->pending_model_enable_ = nullptr;
  }

  if (this->pending_model_disable_ != nullptr) {
    this->pending_model_disable_->set_enabled(false);
    this->pending_model_disable_ = nullptr;
  }
}

void EspSrWakeWord::select_model_(EspSrWakeWordModel *model) {
  for (auto *candidate : this->wake_word_models_)
    candidate->set_enabled(candidate == model);
}

void EspSrWakeWord::unload_models_() {
  for (auto *model : this->wake_word_models_)
    model->unload_model();
}

bool EspSrWakeWord::load_models_(srmodel_list_t *models, int *samples_per_chunk) {
  *samples_per_chunk = 0;
  bool loaded_any_model = false;

  for (auto *model : this->wake_word_models_) {
    if (!model->load_model(models))
      return false;

    const int model_samples_per_chunk = model->get_samples_per_chunk();
    if (model_samples_per_chunk == 0)
      continue;

    if (*samples_per_chunk == 0) {
      *samples_per_chunk = model_samples_per_chunk;
    } else if (*samples_per_chunk != model_samples_per_chunk) {
      ESP_LOGE(TAG, "Loaded models use different sample chunk sizes");
      return false;
    }

    loaded_any_model = true;
  }

  if (!loaded_any_model) {
    ESP_LOGE(TAG, "No ESP-SR wake word models were loaded");
    return false;
  }

  return true;
}

void EspSrWakeWord::process_audio_(int16_t *audio_buffer) {
  for (auto *model : this->wake_word_models_) {
    DetectionEvent detection_event;
    if (model->detect(audio_buffer, &detection_event)) {
      xQueueSend(this->detection_queue_, &detection_event, portMAX_DELAY);
      App.wake_loop_threadsafe();
    }
  }
}

}  // namespace esp_sr
}  // namespace esphome

#endif  // USE_ESP32

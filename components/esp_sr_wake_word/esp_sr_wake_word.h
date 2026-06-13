#pragma once

#ifdef USE_ESP32

#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_wn_iface.h>
#include <model_path.h>

#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace esp_sr {

enum class State : uint8_t {
  STARTING,
  DETECTING_WAKE_WORD,
  STOPPING,
  STOPPED,
};

struct DetectionEvent {
  std::string *wake_word;
};

class EspSrWakeWord;

class EspSrWakeWordModel {
 public:
  EspSrWakeWordModel(const std::string &id, const std::string &model_dir, const std::string &wake_word,
                     bool default_enabled, bool internal_only)
      : id_(id), model_dir_(model_dir), wake_word_(wake_word), enabled_(default_enabled),
        internal_only_(internal_only) {}

  const std::string &get_id() const {
    return this->id_;
  }
  const std::string &get_model_dir() const {
    return this->model_dir_;
  }
  const std::string &get_wake_word() const {
    return this->wake_word_;
  }
  bool get_internal_only() const {
    return this->internal_only_;
  }

  void set_parent(EspSrWakeWord *parent) {
    this->parent_ = parent;
  }
  void set_enabled(bool enabled) {
    this->enabled_ = enabled;
  }
  void enable();
  void disable();
  bool is_enabled() const {
    return this->enabled_;
  }

  bool load_model(srmodel_list_t *models);
  void unload_model();
  bool detect(int16_t *audio_buffer, DetectionEvent *event);
  int get_samples_per_chunk() const;
  void log_model_config() const;

 protected:
  std::string id_;
  std::string model_dir_;
  std::string wake_word_;
  bool enabled_{false};
  bool internal_only_{false};
  EspSrWakeWord *parent_{nullptr};
  const esp_wn_iface_t *iface_{nullptr};
  model_iface_data_t *data_{nullptr};
};

class EspSrWakeWord : public Component
#ifdef USE_OTA_STATE_LISTENER
    ,
                      public ota::OTAGlobalStateListener
#endif
{
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

  void start();
  void stop();
  bool is_running() const {
    return this->state_ != State::STOPPED;
  }

  void set_microphone_source(microphone::MicrophoneSource *microphone_source) {
    this->microphone_source_ = microphone_source;
  }
  void set_stop_after_detection(bool stop_after_detection) {
    this->stop_after_detection_ = stop_after_detection;
  }

  Trigger<std::string> *get_wake_word_detected_trigger() {
    return &this->wake_word_detected_trigger_;
  }

  void add_wake_word_model(EspSrWakeWordModel *model);
  std::vector<EspSrWakeWordModel *> get_wake_words();
  void enable_model(EspSrWakeWordModel *model);
  void disable_model(EspSrWakeWordModel *model);

 protected:
  microphone::MicrophoneSource *microphone_source_{nullptr};
  Trigger<std::string> wake_word_detected_trigger_;
  State state_{State::STOPPED};

  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  std::vector<EspSrWakeWordModel *> wake_word_models_;

  bool pending_start_{false};
  bool pending_stop_{false};
  bool pending_restart_after_stop_{false};
  bool stop_after_detection_{true};
  EspSrWakeWordModel *pending_model_enable_{nullptr};
  EspSrWakeWordModel *pending_model_disable_{nullptr};

  EventGroupHandle_t event_group_{nullptr};
  QueueHandle_t detection_queue_{nullptr};
  TaskHandle_t inference_task_handle_{nullptr};

  static void inference_task(void *params);

  void suspend_task_();
  void resume_task_();
  void set_state_(State state);
  bool has_enabled_model_() const;
  void apply_pending_model_updates_();
  void select_model_(EspSrWakeWordModel *model);
  void unload_models_();
  bool load_models_(srmodel_list_t *models, int *samples_per_chunk);
  void process_audio_(int16_t *audio_buffer);
};

}  // namespace esp_sr
}  // namespace esphome

#endif  // USE_ESP32

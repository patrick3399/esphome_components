#pragma once

#ifdef USE_ESP32

#include "esp_sr_wake_word.h"

#include "esphome/core/automation.h"

namespace esphome {
namespace esp_sr {

template <typename... Ts>
class StartAction : public Action<Ts...>, public Parented<EspSrWakeWord> {
 public:
  void play(const Ts &...x) override {
    this->parent_->start();
  }
};

template <typename... Ts>
class StopAction : public Action<Ts...>, public Parented<EspSrWakeWord> {
 public:
  void play(const Ts &...x) override {
    this->parent_->stop();
  }
};

template <typename... Ts>
class IsRunningCondition : public Condition<Ts...>, public Parented<EspSrWakeWord> {
 public:
  bool check(const Ts &...x) override {
    return this->parent_->is_running();
  }
};

template <typename... Ts>
class EnableModelAction : public Action<Ts...> {
 public:
  explicit EnableModelAction(EspSrWakeWordModel *wake_word_model) : wake_word_model_(wake_word_model) {}
  void play(const Ts &...x) override {
    this->wake_word_model_->enable();
  }

 protected:
  EspSrWakeWordModel *wake_word_model_;
};

template <typename... Ts>
class DisableModelAction : public Action<Ts...> {
 public:
  explicit DisableModelAction(EspSrWakeWordModel *wake_word_model) : wake_word_model_(wake_word_model) {}
  void play(const Ts &...x) override {
    this->wake_word_model_->disable();
  }

 protected:
  EspSrWakeWordModel *wake_word_model_;
};

template <typename... Ts>
class ModelIsEnabledCondition : public Condition<Ts...> {
 public:
  explicit ModelIsEnabledCondition(EspSrWakeWordModel *wake_word_model) : wake_word_model_(wake_word_model) {}
  bool check(const Ts &...x) override {
    return this->wake_word_model_->is_enabled();
  }

 protected:
  EspSrWakeWordModel *wake_word_model_;
};

}  // namespace esp_sr
}  // namespace esphome

#endif  // USE_ESP32

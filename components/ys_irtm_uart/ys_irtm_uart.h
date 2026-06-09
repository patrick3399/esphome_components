#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

#include <string>

#ifdef USE_IR_RF
#include "esphome/components/infrared/infrared.h"
#include "esphome/components/remote_base/nec_protocol.h"
#include "esphome/components/remote_base/remote_base.h"
#endif

namespace esphome {
namespace ys_irtm_uart {

enum YSIrtmBaudRate : uint8_t {
  BAUD_4800 = 0x01,
  BAUD_9600 = 0x02,
  BAUD_19200 = 0x03,
  BAUD_57600 = 0x04,
};

class YSIrtmUartComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_device_address(uint8_t addr) {
    this->device_address_ = addr;
  }

  bool send_raw_hex(const std::string &data);

  // Sends an NEC IR command. The first frame is transmitted immediately; remaining
  // repeat frames are queued and fired from loop() after the required inter-frame gap
  // (~110 ms per NEC spec). If a previous send is still pending, it is replaced.
  bool send_nec(uint8_t user_code_hi, uint8_t user_code_lo, uint8_t key_code, uint8_t repeats = 0);

  bool send_proxy_packet(const std::string &protocol, uint32_t address, uint32_t command, uint8_t repeats);

  // Persist a new device address into module EEPROM (command 0xF2)
  bool set_module_address(uint8_t new_address);

  // Persist a new baud rate into module EEPROM (command 0xF3, takes effect after power cycle)
  bool set_module_baudrate(YSIrtmBaudRate baud_id);

  void add_on_ir_receive_callback(std::function<void(uint8_t, uint8_t, uint8_t)> cb) {
    this->ir_receive_callback_.add(std::move(cb));
  }

 protected:
  void send_command_(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3);

  struct PendingNec {
    uint8_t user_code_hi;
    uint8_t user_code_lo;
    uint8_t key_code;
    uint8_t remaining;  // frames left to send after the current one
    uint32_t last_sent_ms;  // millis() when the last frame was sent (overflow-safe)
  };

  uint8_t device_address_{0xA1};
  uint8_t pending_acks_{0};  // ACK bytes expected from module (one per TX command sent)
  optional<PendingNec> pending_{};
  LazyCallbackManager<void(uint8_t, uint8_t, uint8_t)> ir_receive_callback_{};
};

// ---------------------------------------------------------------------------
// Action: send_raw
// ---------------------------------------------------------------------------
template <typename... Ts>
class SendRawAction : public Action<Ts...> {
 public:
  SendRawAction() {}
  explicit SendRawAction(YSIrtmUartComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, data)

  void play(const Ts &...x) override {
    const auto raw = this->data_.value(x...);
    if (!this->parent_->send_raw_hex(raw))
      ESP_LOGW("ys_irtm_uart", "send_raw failed, data='%s'", raw.c_str());
  }

  void set_parent(YSIrtmUartComponent *p) {
    this->parent_ = p;
  }

 protected:
  YSIrtmUartComponent *parent_{nullptr};
};

// ---------------------------------------------------------------------------
// Action: send_nec
// ---------------------------------------------------------------------------
template <typename... Ts>
class SendNecAction : public Action<Ts...> {
 public:
  SendNecAction() {}
  explicit SendNecAction(YSIrtmUartComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint8_t, user_code_hi)
  TEMPLATABLE_VALUE(uint8_t, user_code_lo)
  TEMPLATABLE_VALUE(uint8_t, key_code)
  TEMPLATABLE_VALUE(uint8_t, repeats)

  void play(const Ts &...x) override {
    this->parent_->send_nec(this->user_code_hi_.value(x...), this->user_code_lo_.value(x...),
                            this->key_code_.value(x...), this->repeats_.value(x...));
  }

  void set_parent(YSIrtmUartComponent *p) {
    this->parent_ = p;
  }

 protected:
  YSIrtmUartComponent *parent_{nullptr};
};

// ---------------------------------------------------------------------------
// Action: send_proxy_packet  (HA IR/RF Proxy bridge)
// ---------------------------------------------------------------------------
template <typename... Ts>
class SendProxyPacketAction : public Action<Ts...> {
 public:
  SendProxyPacketAction() {}
  explicit SendProxyPacketAction(YSIrtmUartComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, protocol)
  TEMPLATABLE_VALUE(uint32_t, address)
  TEMPLATABLE_VALUE(uint32_t, command)
  TEMPLATABLE_VALUE(uint8_t, repeats)

  void play(const Ts &...x) override {
    if (!this->parent_->send_proxy_packet(this->protocol_.value(x...), this->address_.value(x...),
                                          this->command_.value(x...), this->repeats_.value(x...)))
      ESP_LOGW("ys_irtm_uart", "send_proxy_packet failed");
  }

  void set_parent(YSIrtmUartComponent *p) {
    this->parent_ = p;
  }

 protected:
  YSIrtmUartComponent *parent_{nullptr};
};

// ---------------------------------------------------------------------------
// Action: set_address
// ---------------------------------------------------------------------------
template <typename... Ts>
class SetModuleAddressAction : public Action<Ts...> {
 public:
  SetModuleAddressAction() {}
  explicit SetModuleAddressAction(YSIrtmUartComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(uint8_t, address)

  void play(const Ts &...x) override {
    this->parent_->set_module_address(this->address_.value(x...));
  }

  void set_parent(YSIrtmUartComponent *p) {
    this->parent_ = p;
  }

 protected:
  YSIrtmUartComponent *parent_{nullptr};
};

// ---------------------------------------------------------------------------
// Action: set_baudrate
// baud_id is the YSIrtmBaudRate enum value (1–4); set by codegen, not templatable.
// ---------------------------------------------------------------------------
template <typename... Ts>
class SetBaudrateAction : public Action<Ts...> {
 public:
  SetBaudrateAction() {}
  explicit SetBaudrateAction(YSIrtmUartComponent *parent) : parent_(parent) {}

  void set_baud_id(uint8_t id) {
    this->baud_id_ = id;
  }

  void play(const Ts &...x) override {
    this->parent_->set_module_baudrate(static_cast<YSIrtmBaudRate>(this->baud_id_));
  }

  void set_parent(YSIrtmUartComponent *p) {
    this->parent_ = p;
  }

 protected:
  YSIrtmUartComponent *parent_{nullptr};
  uint8_t baud_id_{BAUD_9600};
};

// ---------------------------------------------------------------------------
// Trigger: on_ir_receive
// Variables: user_code_hi (uint8), user_code_lo (uint8), key_code (uint8)
// ---------------------------------------------------------------------------
class IrReceiveTrigger : public Trigger<uint8_t, uint8_t, uint8_t> {
 public:
  explicit IrReceiveTrigger(YSIrtmUartComponent *parent) {
    parent->add_on_ir_receive_callback([this](uint8_t hi, uint8_t lo, uint8_t key) { this->trigger(hi, lo, key); });
  }
};

// ---------------------------------------------------------------------------
// infrared platform: YsIrtmInfrared
// Wraps YSIrtmUartComponent as an infrared::Infrared entity (TX-only, NEC).
// HA sends raw timings → decode NEC → send via UART.
// ---------------------------------------------------------------------------
#ifdef USE_IR_RF
class YsIrtmInfrared : public infrared::Infrared {
 public:
  void setup() override;
  void dump_config() override;

  void set_ys_irtm(YSIrtmUartComponent *ys) {
    this->ys_ = ys;
  }

 protected:
  void control(const infrared::InfraredCall &call) override;

  YSIrtmUartComponent *ys_{nullptr};
};
#endif  // USE_IR_RF

}  // namespace ys_irtm_uart
}  // namespace esphome

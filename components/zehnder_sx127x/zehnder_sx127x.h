#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/sx127x/sx127x.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "protocol.h"

namespace esphome {
namespace zehnder_sx127x {

// A Zehnder fan (RPMe, ComfoAir) controlled over 868 MHz with an SX127x instead of the original nRF905.
// Speed 1-3 like the RFZ remote; every speed change is confirmed by the fan (command 0x07) and resent until it is.
class ZehnderSX127x : public fan::Fan, public Component, public sx127x::SX127xListener {
 public:
  void set_parent(sx127x::SX127x *parent) { this->parent_ = parent; }
  void set_network_id(uint32_t id) { this->network_id_ = id; }
  void set_fan_id(uint8_t id) { this->fan_id_ = id; }
  void set_remote_id(uint8_t id) { this->remote_id_ = id; }
  void set_query_interval(uint32_t ms) { this->query_interval_ = ms; }
  void set_confirmed_binary_sensor(binary_sensor::BinarySensor *s) { this->confirmed_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }  // after the sx127x is configured
  fan::FanTraits get_traits() override { return fan::FanTraits(false, true, false, 3); }
  void on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) override;

 protected:
  void control(const fan::FanCall &call) override;
  void send_(uint8_t rx_id, uint8_t command, uint8_t param, uint8_t count, uint8_t tries);
  void transmit_();
  void transmit_frame_(const uint8_t *frame);
  void watch_receiver_();
  uint8_t read_reg_(uint8_t reg);
  void write_reg_(uint8_t reg, uint8_t value);

  sx127x::SX127x *parent_{nullptr};
  binary_sensor::BinarySensor *confirmed_{nullptr};
  HighFrequencyLoopRequester high_freq_;
  uint32_t network_id_{0};
  uint8_t fan_id_{0};
  uint8_t remote_id_{0};
  uint32_t query_interval_{60000};

  uint8_t tx_frame_[FRAME_LEN]{};
  uint8_t tries_left_{0};
  bool waiting_{false};
  uint32_t next_try_{0};
  uint32_t last_query_{0};
  uint32_t last_reply_{0};
  bool have_reply_{false};
  uint32_t last_poll_us_{0};
  bool ack_pending_{false};
  uint32_t ack_at_{0};
  bool triggered_{false};
  uint32_t triggered_at_{0};
};

}  // namespace zehnder_sx127x
}  // namespace esphome

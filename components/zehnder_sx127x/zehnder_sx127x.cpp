#include "zehnder_sx127x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zehnder_sx127x {

static const char *const TAG = "zehnder_sx127x";

static constexpr uint32_t RETRY_MS = 1000;  // the fan answers within ~0.4 s, its repeats last ~0.3 s more
static constexpr uint8_t SPEED_TRIES = 5;
static constexpr uint8_t QUERY_TRIES = 3;
static constexpr uint8_t CMD_SET_SPEED_ACK = 0x05;
static constexpr uint32_t ACK_DELAY_MS = 300;  // after the last copy of the fan's reply, as the RFZ does it

// SX127x FSK registers used directly (the sx127x component does not expose these settings).
static constexpr uint8_t REG_RX_CONFIG = 0x0D;
static constexpr uint8_t REG_IRQ_FLAGS1 = 0x3E;
static constexpr uint8_t RX_RESTART = 0x40;
static constexpr uint8_t AFC_AUTO_ON = 0x10;
static constexpr uint8_t IRQ_RSSI = 0x08;
static constexpr uint8_t IRQ_SYNC_MATCH = 0x01;

void ZehnderSX127x::setup() {
  this->parent_->register_listener(this);
  // AFC off: the nRF905 preamble is only 0.2 ms, too short for the AFC to settle. Correct a crystal offset with
  // the sx127x frequency instead (see README).
  this->write_reg_(REG_RX_CONFIG, (this->read_reg_(REG_RX_CONFIG) & ~AFC_AUTO_ON) | RX_RESTART);
  this->high_freq_.start();  // watch_receiver_() needs to run about every millisecond
  this->last_query_ = millis() - this->query_interval_ + 2000;  // first query 2 s after boot
}

void ZehnderSX127x::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Zehnder SX127x fan:\n"
                "  Network: 0x%08" PRIX32 "\n"
                "  Fan ID: 0x%02X\n"
                "  Remote ID: 0x%02X\n"
                "  Query interval: %" PRIu32 " ms",
                this->network_id_, this->fan_id_, this->remote_id_, this->query_interval_);
}

void ZehnderSX127x::loop() {
  this->watch_receiver_();
  uint32_t now = millis();

  if (this->tries_left_ > 0 && (int32_t) (now - this->next_try_) >= 0) {
    this->transmit_();
  } else if (this->waiting_ && this->tries_left_ == 0 && (int32_t) (now - this->next_try_) >= 0) {
    this->waiting_ = false;
    ESP_LOGW(TAG, "No confirmation from fan 0x%02X", this->fan_id_);
    if (this->tx_frame_[COMMAND] == CMD_SET_SPEED)
      this->last_query_ = now - this->query_interval_;  // ask what the fan does now, so the entity shows the truth
  }

  if (this->ack_pending_ && (int32_t) (now - this->ack_at_) >= 0) {
    this->ack_pending_ = false;
    static const uint8_t params[3] = {0x4F, 0x02, 0x20};  // as sent by an RFZ to this fan
    uint8_t ack[FRAME_LEN];
    build(ack, this->network_id_, this->fan_id_, this->remote_id_, CMD_SET_SPEED_ACK, params, 3);
    this->transmit_frame_(ack);
  }

  if (!this->waiting_ && !this->ack_pending_ && now - this->last_query_ >= this->query_interval_) {
    this->last_query_ = now;
    this->send_(this->fan_id_, CMD_QUERY, 0, 0, QUERY_TRIES);
  }

  if (this->confirmed_ != nullptr) {
    bool ok = this->have_reply_ && now - this->last_reply_ < 3 * this->query_interval_ + 5000;
    if (!this->confirmed_->has_state() || this->confirmed_->state != ok)
      this->confirmed_->publish_state(ok);
  }
}

// A noise spike above rx_floor triggers the receiver, which then freezes AGC and hunts for a sync word until a
// random match (~0.65 s on average), deaf meanwhile. No sync within 3 ms after a trigger: restart the receiver.
void ZehnderSX127x::watch_receiver_() {
  uint32_t now_us = micros();
  if (now_us - this->last_poll_us_ < 1000)
    return;
  this->last_poll_us_ = now_us;
  uint8_t irq = this->read_reg_(REG_IRQ_FLAGS1);
  if (!(irq & IRQ_RSSI) || (irq & IRQ_SYNC_MATCH)) {
    this->triggered_ = false;
    return;
  }
  uint32_t now = millis();
  if (!this->triggered_) {
    this->triggered_ = true;
    this->triggered_at_ = now;
  } else if (now - this->triggered_at_ > 3) {
    this->write_reg_(REG_RX_CONFIG, this->read_reg_(REG_RX_CONFIG) | RX_RESTART);
    this->triggered_ = false;
  }
}

void ZehnderSX127x::control(const fan::FanCall &call) {
  if (call.get_state().has_value())
    this->state = *call.get_state();
  if (call.get_speed().has_value())
    this->speed = *call.get_speed();
  if (this->state && this->speed == 0)
    this->speed = 1;
  // shortcut: off sends speed 0 like the original firmware; not tested on an RPMe, verify before relying on it
  uint8_t speed = this->state ? this->speed : 0;
  ESP_LOGD(TAG, "Set speed %u", speed);
  // Speed goes to main unit ID 0x00, as the RFZ remote does it.
  this->send_(0x00, CMD_SET_SPEED, speed, 1, SPEED_TRIES);
  this->publish_state();  // optimistic; corrected by the fan's confirmation
}

void ZehnderSX127x::send_(uint8_t rx_id, uint8_t command, uint8_t param, uint8_t count, uint8_t tries) {
  build(this->tx_frame_, this->network_id_, rx_id, this->remote_id_, command, &param, count);
  this->tries_left_ = tries;
  this->waiting_ = true;
  this->transmit_();
}

void ZehnderSX127x::transmit_() {
  this->transmit_frame_(this->tx_frame_);
  this->tries_left_--;
  this->next_try_ = millis() + RETRY_MS;
}

void ZehnderSX127x::transmit_frame_(const uint8_t *frame) {
  std::vector<uint8_t> air(AIR_LEN);
  encode(frame, air.data());
  if (this->parent_->transmit_packet(air) != sx127x::SX127xError::NONE)
    ESP_LOGW(TAG, "Transmit failed");
}

void ZehnderSX127x::on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) {
  uint8_t f[FRAME_LEN];
  if (packet.size() != AIR_LEN || !decode(packet.data(), f))
    return;
  // Verbose only: logging every frame over wifi takes milliseconds, and frames arrive ~25 ms apart in bursts.
  ESP_LOGV(TAG, "Frame network 0x%08" PRIX32 " %02X:%02X <- %02X:%02X ttl %02X command %02X params %s", network_of(f),
           f[RX_TYPE], f[RX_ID], f[TX_TYPE], f[TX_ID], f[TTL_BYTE], f[COMMAND],
           format_hex_pretty(f + PARAM_COUNT, 10).c_str());
  if (network_of(f) != this->network_id_ || f[COMMAND] != CMD_FAN_SETTINGS || f[TX_TYPE] != TYPE_MAIN_UNIT ||
      f[TX_ID] != this->fan_id_)
    return;

  // Settings from the fan, to us or to an RFZ remote: either way it is the fan's current speed.
  this->last_reply_ = millis();
  this->have_reply_ = true;
  uint8_t speed = f[PARAMS];  // params: speed, voltage %, timer, ...
  bool to_us = f[RX_TYPE] == TYPE_REMOTE && f[RX_ID] == this->remote_id_;
  bool setting = this->waiting_ && this->tx_frame_[COMMAND] == CMD_SET_SPEED;
  if (setting && speed != this->tx_frame_[PARAMS]) {
    // Seen when a new speed follows the previous one within seconds: the fan answers with its old speed and
    // ignores the command. Not a confirmation; keep resending.
    if (f[TTL_BYTE] == TTL)  // once per burst, not for each repeated copy
      ESP_LOGD(TAG, "Fan still reports speed %u, wanted %u", speed, this->tx_frame_[PARAMS]);
    return;
  }
  if (to_us && this->waiting_) {
    ESP_LOGD(TAG, "Confirmed by fan: speed %u (%u%%), %u tries left", speed, f[PARAMS + 1], this->tries_left_);
    this->tries_left_ = 0;
    this->waiting_ = false;
    this->ack_pending_ = setting;  // close a speed change with 0x05, like the RFZ and the nRF905 firmware do
  }
  if (to_us && this->ack_pending_)
    this->ack_at_ = millis() + ACK_DELAY_MS;  // every copy of the reply pushes the ack to after the burst
  bool on = speed > 0;
  int level = speed > 3 ? 3 : speed;
  if (on != this->state || (on && level != this->speed)) {
    ESP_LOGI(TAG, "Fan reports speed %u (%u%%)", speed, f[PARAMS + 1]);
    this->state = on;
    if (on)
      this->speed = level;
    this->publish_state();
  }
}

uint8_t ZehnderSX127x::read_reg_(uint8_t reg) {
  this->parent_->enable();
  this->parent_->write_byte(reg & 0x7F);
  uint8_t value = this->parent_->read_byte();
  this->parent_->disable();
  return value;
}

void ZehnderSX127x::write_reg_(uint8_t reg, uint8_t value) {
  this->parent_->enable();
  this->parent_->write_byte(reg | 0x80);
  this->parent_->write_byte(value);
  this->parent_->disable();
}

}  // namespace zehnder_sx127x
}  // namespace esphome

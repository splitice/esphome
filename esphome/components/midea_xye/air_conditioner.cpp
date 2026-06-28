#include "air_conditioner.h"

#include <cstdio>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace midea {
namespace ac {

const char *const Constants::TAG = "midea_xye";
const char *const Constants::FREEZE_PROTECTION = "Freeze Protection";
const char *const Constants::SILENT = "Silent";
const char *const Constants::TURBO = "Turbo";

static void set_sensor(Sensor *sensor, float value) {
  if (sensor != nullptr && (!sensor->has_state() || sensor->get_raw_state() != value))
    sensor->publish_state(value);
}

static void set_number(number::Number *number, float value) {
  if (number != nullptr && (!number->has_state() || number->state != value))
    number->publish_state(value);
}

#ifdef USE_TEXT_SENSOR
static void set_text_sensor(text_sensor::TextSensor *sens, const std::string &value) {
  if (sens != nullptr && (!sens->has_state() || sens->get_raw_state() != value))
    sens->publish_state(value);
}
#endif

#ifdef USE_BINARY_SENSOR
static void set_binary_sensor(binary_sensor::BinarySensor *sens, bool value) {
  if (sens != nullptr && (!sens->has_state() || sens->state != value))
    sens->publish_state(value);
}
#endif

template<typename T> void update_property(T &property, const T &value, bool &flag) {
  if (property != value) {
    property = value;
    flag = true;
  }
}

static void log_vrf_frame(const char *prefix, const uint8_t *frame, uint8_t len) {
  char hex[(VRF_RX_MAX_LEN * 3) + 1];
  size_t pos = 0;
  for (uint8_t i = 0; i < len && pos < sizeof(hex); i++) {
    int written = snprintf(hex + pos, sizeof(hex) - pos, "%02X%s", frame[i], (i + 1 < len) ? " " : "");
    if (written <= 0)
      break;
    pos += static_cast<size_t>(written);
  }
  hex[sizeof(hex) - 1] = 0;
  ESP_LOGD(Constants::TAG, "%s %s", prefix, hex);
}

void AirConditioner::control(const ClimateCall &call) {
  if (this->vrf_protocol_) {
    this->control_vrf(call);
    return;
  }

  if (call.get_mode().has_value()) {
    this->mode = call.get_mode().value();
    followMeInit = false;
  }
  if (call.get_target_temperature().has_value())
    this->target_temperature = call.get_target_temperature().value();
  if (call.get_fan_mode().has_value())
    this->fan_mode = call.get_fan_mode().value();
  if (call.get_swing_mode().has_value())
    this->swing_mode = call.get_swing_mode().value();
  if (call.get_preset().has_value())
    this->preset = call.get_preset().value();
  this->publish_state();

  if (controlState != STATE_WAIT_DATA) {
    controlState = STATE_SEND_C3;
  } else {
    queuedCommand = STATE_SEND_C3;
  }
}

void AirConditioner::setup() {
  // this->uart_->check_uart_settings(4800, 1, UART_CONFIG_PARITY_NONE, 8);
  if (!this->supported_modes_.empty()) {
    this->last_on_mode_ = *this->supported_modes_.begin();
  } else {
    this->last_on_mode_ = ClimateMode::CLIMATE_MODE_COOL;
  }
  if (this->vrf_protocol_ && !EncodeVrfMode(this->last_on_mode_, this->vrf_last_mode_nibble_)) {
    this->last_on_mode_ = ClimateMode::CLIMATE_MODE_COOL;
    this->vrf_last_mode_nibble_ = 0x02;
  }
  controlState = STATE_SEND_C0;
  ForceReadNextCycle = 1;
  followMeInit = false;
  lastFollowMeTemperature = 0;
  this->vrf_waiting_response_ = false;
  this->vrf_queue_head_ = 0;
  this->vrf_queue_tail_ = 0;
  this->vrf_queue_count_ = 0;

  // Start up in Auto fan mode (since unit doesn't report it correctly)
  this->fan_mode = ClimateFanMode::CLIMATE_FAN_AUTO;

  if (this->vrf_protocol_ && this->use_fahrenheit_) {
    ESP_LOGW(Constants::TAG, "VRF protocol uses Celsius setpoints; Fahrenheit encoding will be ignored.");
  }

#ifdef USE_SWITCH
  if (this->use_fahrenheit_switch_ != nullptr) {
    this->use_fahrenheit_switch_->publish_state(this->use_fahrenheit_);
  }
#endif
}

// TODO: Not sure if we really need this.
void AirConditioner::setPowerState(bool state) {
  if (this->vrf_protocol_) {
    ClimateMode mode = state ? this->last_on_mode_ : ClimateMode::CLIMATE_MODE_OFF;
    if (!this->queue_vrf_mode_command(mode) && state) {
      this->queue_vrf_mode_command(ClimateMode::CLIMATE_MODE_COOL);
    }
    this->publish_state();
    return;
  }

  if (state)
    this->mode = this->last_on_mode_;
  else
    this->mode = ClimateMode::CLIMATE_MODE_OFF;

  if (controlState != STATE_WAIT_DATA) {
    controlState = STATE_SEND_C3;
  } else {
    queuedCommand = STATE_SEND_C3;
  }
}

void AirConditioner::control_vrf(const ClimateCall &call) {
  bool need_publish = false;

  if (call.get_mode().has_value()) {
    need_publish |= this->queue_vrf_mode_command(call.get_mode().value());
  }
  if (call.get_target_temperature().has_value()) {
    need_publish |= this->queue_vrf_temperature_command(call.get_target_temperature().value());
  }
  if (call.get_fan_mode().has_value()) {
    ESP_LOGW(Constants::TAG, "VRF fan control is not implemented; ignoring fan mode command.");
  }
  if (call.get_swing_mode().has_value()) {
    ESP_LOGW(Constants::TAG, "VRF swing control is not implemented; ignoring swing mode command.");
  }
  if (call.get_preset().has_value()) {
    ESP_LOGW(Constants::TAG, "VRF preset control is not implemented; ignoring preset command.");
  }

  if (need_publish) {
    this->publish_state();
  }
}

bool AirConditioner::queue_vrf_payload(const uint8_t *payload, uint8_t len) {
  if (len > VRF_PAYLOAD_MAX_LEN) {
    ESP_LOGW(Constants::TAG, "Cannot queue VRF payload with length %d > %d", len, VRF_PAYLOAD_MAX_LEN);
    return false;
  }
  if (this->vrf_queue_count_ >= VRF_QUEUE_LEN) {
    ESP_LOGW(Constants::TAG, "Cannot queue VRF payload; queue is full");
    return false;
  }

  VrfPayload &queued = this->vrf_queue_[this->vrf_queue_tail_];
  queued.len = len;
  memcpy(queued.data, payload, len);
  this->vrf_queue_tail_ = (this->vrf_queue_tail_ + 1) % VRF_QUEUE_LEN;
  this->vrf_queue_count_++;
  return true;
}

bool AirConditioner::queue_vrf_mode_command(ClimateMode mode) {
  uint8_t value = 0;

  if (mode == ClimateMode::CLIMATE_MODE_OFF) {
    value = this->vrf_last_mode_nibble_;
  } else {
    uint8_t mode_nibble = 0;
    if (!EncodeVrfMode(mode, mode_nibble)) {
      ESP_LOGW(Constants::TAG, "VRF mode %d is not supported", mode);
      return false;
    }
    value = 0x40 | mode_nibble;
  }

  const uint8_t payload[] = {0x01, 0x00, value};
  if (!this->queue_vrf_payload(payload, sizeof(payload))) {
    return false;
  }

  if (mode == ClimateMode::CLIMATE_MODE_OFF) {
    this->mode = ClimateMode::CLIMATE_MODE_OFF;
    this->action = climate::CLIMATE_ACTION_OFF;
  } else {
    this->mode = mode;
    this->last_on_mode_ = mode;
    this->vrf_last_mode_nibble_ = value & 0x0F;
    this->action =
        (mode == ClimateMode::CLIMATE_MODE_FAN_ONLY) ? climate::CLIMATE_ACTION_FAN : climate::CLIMATE_ACTION_IDLE;
  }
  return true;
}

bool AirConditioner::queue_vrf_temperature_command(float target_temperature) {
  uint8_t encoded = EncodeVrfTemp(target_temperature);
  const uint8_t payload[] = {0x01, 0x03, encoded, 0x04, encoded, 0x02, encoded};
  if (!this->queue_vrf_payload(payload, sizeof(payload))) {
    return false;
  }

  this->target_temperature = DecodeVrfTemp(encoded);
  return true;
}

void AirConditioner::prepareTXData(uint8_t command) {
  TXData[0] = PREAMBLE;
  TXData[1] = command;
  TXData[2] = SERVER_ID;
  TXData[3] = CLIENT_ID;
  TXData[4] = FROM_CLIENT;
  TXData[5] = CLIENT_ID;
  TXData[6] = 0;
  TXData[7] = 0;
  TXData[8] = 0;
  TXData[9] = 0;
  TXData[10] = 0;
  TXData[11] = 0;
  TXData[12] = 0;
  TXData[13] = 0xFF - TXData[1];
  TXData[15] = PROLOGUE;
  TXData[14] = CalculateCRC(TXData, TX_LEN);
}

uint8_t AirConditioner::adjust_target_temperature(float target_temperature) const {
  float adjusted_target_temperature = target_temperature;
  if (this->use_fahrenheit_) {
    adjusted_target_temperature = ((9.0f / 5.0f) * adjusted_target_temperature + 32.0f) + 0x87;
  }

  if (this->mode == ClimateMode::CLIMATE_MODE_HEAT) {
    adjusted_target_temperature = ceilf(adjusted_target_temperature);
  } else {
    adjusted_target_temperature = floorf(adjusted_target_temperature);
  }

  return static_cast<uint8_t>(adjusted_target_temperature);
}

float AirConditioner::read_target_temperature(uint8_t target_temperature, bool fahrenheit_encoded) const {
  float ret;
  if (fahrenheit_encoded) {
    ret = ((static_cast<float>(target_temperature) - 0x87f) - 32.0f) * 5.0f / 9.0f;
  } else {
    ret = static_cast<float>(target_temperature & 0xBF);
  }
  return ret;
}

void AirConditioner::setACParams() {
  // construct set command
  prepareTXData(CLIENT_COMMAND_SET);

  // set mode
  switch (this->mode) {
    case ClimateMode::CLIMATE_MODE_OFF:
      TXData[6] = OP_MODE_OFF;
      break;
    case ClimateMode::CLIMATE_MODE_HEAT_COOL:
      TXData[6] = OP_MODE_AUTO;
      break;
    case ClimateMode::CLIMATE_MODE_FAN_ONLY:
      TXData[6] = OP_MODE_FAN;
      break;
    case ClimateMode::CLIMATE_MODE_DRY:
      TXData[6] = OP_MODE_DRY;
      break;
    case ClimateMode::CLIMATE_MODE_HEAT:
      TXData[6] = OP_MODE_HEAT;
      break;
    case ClimateMode::CLIMATE_MODE_COOL:
      TXData[6] = OP_MODE_COOL;
      break;
    default:
      TXData[6] = OP_MODE_OFF;
  }
  // set fan mode
  if (this->mode != ClimateMode::CLIMATE_MODE_HEAT_COOL) {
    switch (this->fan_mode.value()) {
      case ClimateFanMode::CLIMATE_FAN_AUTO:
        TXData[7] = FAN_MODE_AUTO;
        break;
      case ClimateFanMode::CLIMATE_FAN_HIGH:
        TXData[7] = FAN_MODE_HIGH;
        break;
      case ClimateFanMode::CLIMATE_FAN_MEDIUM:
        TXData[7] = FAN_MODE_MEDIUM;
        break;
      case ClimateFanMode::CLIMATE_FAN_LOW:
        TXData[7] = FAN_MODE_LOW;
        break;
      default:
        TXData[7] = FAN_MODE_AUTO;
    }
  } else {
    // Auto is full-auto - can't set fan mode either.
    this->fan_mode = ClimateFanMode::CLIMATE_FAN_AUTO;
    TXData[7] = FAN_MODE_AUTO;
  }
  TXData[8] = this->adjust_target_temperature(this->target_temperature);

  // set mode flags
  TXData[11] = ((this->preset == ClimatePreset::CLIMATE_PRESET_BOOST) * MODE_FLAG_AUX_HEAT) |
               ((this->preset == ClimatePreset::CLIMATE_PRESET_SLEEP) * MODE_FLAG_ECO) |
               ((this->swing_mode != ClimateSwingMode::CLIMATE_SWING_OFF) * MODE_FLAG_SWING) | (0 * MODE_FLAG_VENT);

  // set timer start
  // TODO: This is not tested. If you use it probably want to switch to
  // State.TimerStart so timer doesn't get ovedrridden TXData[9] =
  // CalculateSetTime(DesiredState.TimerStart); set timer stop TXData[10] =
  // CalculateSetTime(DesiredState.TimerStop);

  TXData[14] = CalculateCRC(TXData, TX_LEN);
}

void AirConditioner::sendRecv(uint8_t cmdSent) {
  // TODO: Reimplement flow control for manual RS485 flow control chips
  // digitalWrite(ComControlPin, RS485_TX_PIN_VALUE);
  this->uart_->write_array(TXData, TX_LEN);
  this->uart_->flush();
  controlState = STATE_WAIT_DATA;
  // Delay the remaining for 100 ms to allow response from the AC unit.
  this->set_timeout("read-result", 100, [this, cmdSent]() {
    // digitalWrite(ComControlPin, RS485_RX_PIN_VALUE);

    uint8_t i = 0;
    uint8_t total_bytes = 0;
    while (this->uart_->available()) {
      uint8_t byte = 0;
      if (!this->uart_->read_byte(&byte)) {
        break;
      }
      if (i < RX_LEN) {
        RXData[i] = byte;
        i++;
      }
      total_bytes++;
    }
    if (i == RX_LEN) {
      if (total_bytes > RX_LEN) {
        ESP_LOGW(Constants::TAG, "Received %d bytes for Command %02X, using first %d", total_bytes, cmdSent, RX_LEN);
      }
      if (cmdSent != 0xC3) {
        ParseResponse(cmdSent);
      }
      if (queuedCommand != 0) {
        controlState = queuedCommand;
        queuedCommand = 0;
      } else {
        switch (cmdSent) {
          case 0xC0:
            controlState = STATE_SEND_C4;
            break;
          case 0xC3:
            controlState = STATE_SEND_C6;
            break;
          case 0xC4:
            controlState = STATE_SEND_C3;
            break;
          case 0xC6:
            controlState = STATE_SEND_C0;
            break;
        }
      }
    } else {
      ESP_LOGE(Constants::TAG, "Received incorrect message length from AC for Command %02X with length %d", cmdSent,
               i);
      controlState = STATE_SEND_C0;
    }
  });
}

void AirConditioner::update() {
  if (this->vrf_protocol_) {
    this->update_vrf();
    return;
  }

  uint8_t cmdSent = 0x00;
  // Possible States:
  // 0: Waiting for Response from Command
  // 1: Sending Set C3 Command
  // 2: Sending Set C6 Command
  // 3: Sending Query C0 Command
  // 4: Sending Query C4 Command
  switch (controlState) {
    case STATE_SEND_C3: {
      // construct set command (includes preparing the data)
      setACParams();
      cmdSent = CLIENT_COMMAND_SET;
      sendRecv(cmdSent);
      break;
    }
    case STATE_SEND_C6: {
      // If the AC mode changed, follow-me should be
      // refreshed, if emulating the wired controller's
      // behavior.
      
      // prepared by do_follow_me

      if(TXData[1] != 0xC6) {
        if(lastFollowMeTemperature == 0) {
          controlState = STATE_SEND_C0;
          break;
        }
        this->do_follow_me(lastFollowMeTemperature, false);
      }
      
      cmdSent = 0xC6;
      sendRecv(cmdSent);
      if (this->mode == ClimateMode::CLIMATE_MODE_OFF) {
        ESP_LOGI(Constants::TAG, "Set static pressure.");
      } else {
        ESP_LOGI(Constants::TAG, "Sent Follow-Me data.");
      }
      break;
    }
    case STATE_SEND_C0: {
      // construct query command
      prepareTXData(CLIENT_COMMAND_QUERY);
      cmdSent = CLIENT_COMMAND_QUERY;
      sendRecv(cmdSent);
      break;
    }
    case STATE_SEND_C4: {
      // extended query
      setACParams();
      
      TXData[1] = 0xC4;
      TXData[13] = 0xFF - 0xC4;

      cmdSent = 0xC4;
      sendRecv(cmdSent);
      break;
    }
    case STATE_WAIT_DATA: {
      // Wait for data to processed. Do nothing during the loop.
      break;
    }
    default: {
      controlState = STATE_SEND_C3;
    }
  }
}

void AirConditioner::update_vrf() {
  if (this->vrf_waiting_response_) {
    return;
  }

  if (this->vrf_queue_count_ > 0) {
    const VrfPayload &payload = this->vrf_queue_[this->vrf_queue_head_];
    this->send_vrf_payload(payload.data, payload.len);
    this->vrf_queue_head_ = (this->vrf_queue_head_ + 1) % VRF_QUEUE_LEN;
    this->vrf_queue_count_--;
    return;
  }

  const uint8_t payload[] = {VRF_POLL_REQUEST};
  this->send_vrf_payload(payload, sizeof(payload));
}

void AirConditioner::send_vrf_payload(const uint8_t *payload, uint8_t len) {
  if (len > VRF_PAYLOAD_MAX_LEN) {
    ESP_LOGW(Constants::TAG, "Cannot send VRF payload with length %d > %d", len, VRF_PAYLOAD_MAX_LEN);
    return;
  }

  uint8_t frame[VRF_FRAME_MAX_LEN];
  frame[0] = PREAMBLE;
  frame[1] = VRF_COMMAND_STATUS;
  frame[2] = SERVER_ID;
  frame[3] = 0x00;
  frame[4] = CLIENT_ID;
  frame[5] = 0x00;
  frame[6] = len;
  memcpy(&frame[7], payload, len);

  uint16_t crc = CalculateVrfCRC(&frame[1], 6 + len);
  uint8_t crc_pos = 7 + len;
  frame[crc_pos] = crc & 0xFF;
  frame[crc_pos + 1] = (crc >> 8) & 0xFF;
  frame[crc_pos + 2] = PROLOGUE;
  frame[crc_pos + 3] = VRF_FRAME_END_2;

  uint8_t frame_len = 11 + len;
  log_vrf_frame("VRF TX:", frame, frame_len);
  this->uart_->write_array(frame, frame_len);
  this->uart_->flush();
  this->vrf_waiting_response_ = true;

  this->set_timeout("vrf-read-result", this->response_timeout, [this]() {
    uint8_t frame[VRF_RX_MAX_LEN];
    uint8_t i = 0;
    uint16_t total_bytes = 0;
    while (this->uart_->available()) {
      uint8_t byte = 0;
      if (!this->uart_->read_byte(&byte)) {
        break;
      }
      if (i < VRF_RX_MAX_LEN) {
        frame[i] = byte;
        i++;
      }
      total_bytes++;
    }

    this->vrf_waiting_response_ = false;
    if (i == 0) {
      ESP_LOGD(Constants::TAG, "No VRF response received");
      return;
    }
    if (total_bytes > i) {
      ESP_LOGW(Constants::TAG, "Received %d VRF bytes, using first %d", static_cast<int>(total_bytes),
               static_cast<int>(i));
    }

    uint8_t start = 0;
    while (start < i && frame[start] != PREAMBLE) {
      start++;
    }
    if (start >= i) {
      ESP_LOGW(Constants::TAG, "Received VRF data without frame preamble");
      return;
    }
    if (start > 0) {
      ESP_LOGW(Constants::TAG, "Skipping %d byte(s) before VRF frame preamble", static_cast<int>(start));
    }

    this->parse_vrf_response(&frame[start], i - start);
  });
}

void AirConditioner::parse_vrf_response(const uint8_t *frame, uint8_t len) {
  if (len < 11) {
    ESP_LOGW(Constants::TAG, "Received short VRF frame with length %d", len);
    return;
  }
  if (frame[0] != PREAMBLE || frame[1] != VRF_COMMAND_STATUS) {
    ESP_LOGW(Constants::TAG, "Received invalid VRF frame header");
    return;
  }

  uint8_t payload_len = frame[6];
  uint8_t expected_len = 11 + payload_len;
  if (len < expected_len) {
    ESP_LOGW(Constants::TAG, "Received incomplete VRF frame with length %d, expected %d", len, expected_len);
    return;
  }
  if (frame[expected_len - 2] != PROLOGUE || frame[expected_len - 1] != VRF_FRAME_END_2) {
    ESP_LOGW(Constants::TAG, "Received VRF frame with invalid terminator");
    return;
  }

  uint16_t received_crc = frame[expected_len - 4] | (static_cast<uint16_t>(frame[expected_len - 3]) << 8);
  uint16_t calculated_crc = CalculateVrfCRC(&frame[1], 6 + payload_len);
  if (received_crc != calculated_crc) {
    ESP_LOGW(Constants::TAG, "Received VRF frame with invalid CRC %04X != %04X",
             static_cast<unsigned>(received_crc), static_cast<unsigned>(calculated_crc));
    return;
  }

  log_vrf_frame("VRF RX:", frame, expected_len);

  // Short 0x23 frames may be command echoes/acks. Status frames carry at least
  // enough payload for pwr/mode, fan, setpoint, and swing bytes.
  if (payload_len < 11) {
    ESP_LOGD(Constants::TAG, "VRF frame payload length %d is too short for status parsing", payload_len);
    return;
  }

  uint8_t pwr_mode = frame[8];
  uint8_t mode_nibble = pwr_mode & 0x0F;
  bool powered = (pwr_mode & 0xF0) == 0x40;
  bool need_publish = false;

  if (powered) {
    ClimateMode decoded_mode;
    if (DecodeVrfMode(mode_nibble, decoded_mode)) {
      update_property(this->mode, decoded_mode, need_publish);
      this->last_on_mode_ = decoded_mode;
      this->vrf_last_mode_nibble_ = mode_nibble;
      climate::ClimateAction action =
          (decoded_mode == ClimateMode::CLIMATE_MODE_FAN_ONLY) ? climate::CLIMATE_ACTION_FAN
                                                               : climate::CLIMATE_ACTION_IDLE;
      update_property(this->action, action, need_publish);
    } else {
      ESP_LOGW(Constants::TAG, "Received unknown VRF mode nibble %02X", mode_nibble);
    }
  } else {
    update_property(this->mode, ClimateMode::CLIMATE_MODE_OFF, need_publish);
    update_property(this->action, climate::CLIMATE_ACTION_OFF, need_publish);
  }

  update_property(this->target_temperature, DecodeVrfTemp(frame[10]), need_publish);
  ESP_LOGD(Constants::TAG, "VRF status pwr/mode=%02X fan=%02X setpoint=%.1f swing=%02X", pwr_mode, frame[9],
           DecodeVrfTemp(frame[10]), frame[17]);

  if (need_publish) {
    this->publish_state();
  }
}

uint8_t AirConditioner::CalculateCRC(uint8_t *data, uint8_t len) {
  uint32_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    if (i != len - 2) {
      crc += data[i];
    }
  }
  return 0xFF - (crc & 0xFF);
}

uint16_t AirConditioner::CalculateVrfCRC(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void AirConditioner::ParseResponse(uint8_t cmdSent) {
  // validate the response
  if ((RXData[RX_BYTE_PREAMBLE] == PREAMBLE) && (RXData[RX_BYTE_PROLOGUE] == PROLOGUE) &&
      (RXData[RX_BYTE_TO_CLIENT] == TO_CLIENT) && (RXData[RX_BYTE_CRC] == CalculateCRC(RXData, RX_LEN))) {
    switch (RXData[RX_BYTE_COMMAND_TYPE]) {
      case CLIENT_COMMAND_QUERY: {
        ClimateMode mode = ClimateMode::CLIMATE_MODE_OFF;
        ClimateFanMode fan_mode = ClimateFanMode::CLIMATE_FAN_AUTO;
        ClimatePreset preset = ClimatePreset::CLIMATE_PRESET_NONE;

        switch (RXData[RX_C0_BYTE_OP_MODE] & 0xEF) {
          case OP_MODE_OFF:
            mode = ClimateMode::CLIMATE_MODE_OFF;
            break;
          case OP_MODE_AUTO:
            mode = ClimateMode::CLIMATE_MODE_HEAT_COOL;
            break;
          case OP_MODE_FAN:
            mode = ClimateMode::CLIMATE_MODE_FAN_ONLY;
            break;
          case OP_MODE_DRY:
            mode = ClimateMode::CLIMATE_MODE_DRY;
            break;
          case OP_MODE_HEAT:
            mode = ClimateMode::CLIMATE_MODE_HEAT;
            break;
          case OP_MODE_COOL:
            mode = ClimateMode::CLIMATE_MODE_COOL;
            break;
        }

        // The unit seems to show 0x10 when off after running auto.
        // Check to see if we haven't already matched to OFF state.
        // If not, and we match otherwise, we are in auto mode.
        if (mode != ClimateMode::CLIMATE_MODE_OFF &&
            ((RXData[RX_C0_BYTE_OP_MODE] & OP_MODE_AUTO_FLAG) == OP_MODE_AUTO_FLAG)) {
          mode = ClimateMode::CLIMATE_MODE_HEAT_COOL;
        }

        uint8_t current_fan_speed = RXData[RX_C0_BYTE_FAN_MODE] & 0x0F;
        switch (current_fan_speed) {
          case FAN_MODE_HIGH:
            fan_mode = ClimateFanMode::CLIMATE_FAN_HIGH;
            break;
          case FAN_MODE_MEDIUM:
            fan_mode = ClimateFanMode::CLIMATE_FAN_MEDIUM;
            break;
          case FAN_MODE_LOW:
            fan_mode = ClimateFanMode::CLIMATE_FAN_LOW;
            break;
          case FAN_MODE_OFF:
            fan_mode = ClimateFanMode::CLIMATE_FAN_OFF;
            break;
        }
        if ((RXData[RX_C0_BYTE_FAN_MODE] & FAN_MODE_AUTO) == FAN_MODE_AUTO) {
          fan_mode = ClimateFanMode::CLIMATE_FAN_AUTO;
        }

        if (RXData[RX_C0_BYTE_MODE_FLAGS] & MODE_FLAG_AUX_HEAT)
          preset = ClimatePreset::CLIMATE_PRESET_BOOST;
        else if (RXData[RX_C0_BYTE_MODE_FLAGS] & MODE_FLAG_ECO)
          preset = ClimatePreset::CLIMATE_PRESET_SLEEP;

        bool need_publish = false;

        update_property(this->mode, mode, need_publish);
        if (mode == ClimateMode::CLIMATE_MODE_OFF) {
          if (this->action != climate::CLIMATE_ACTION_OFF) {
            this->action = climate::CLIMATE_ACTION_OFF;
            need_publish = true;
          }
        } else {
          this->last_on_mode_ = mode;
        }

        if (mode != ClimateMode::CLIMATE_MODE_OFF ||
            ForceReadNextCycle == 1)  // Don't update below states unless mode is an ON state
        {
          // Don't update the fan mode. Assume it set correctly.
          // Show Heating vs Heat at least in Heat mode. Will figure
          // out how to determine if compressor is on in other modes later.
          // Cursor filled out the rest of the modes, but this is still a hack.

          // If we are using C, update the temperature here. Mask out 0x40. If we are using F, update
          // via 0xC4.
          // In either case, don't update it if the user is the in middle of setting it to something new...
          //
          if ((!this->use_fahrenheit_) && (this->queuedCommand != STATE_SEND_C3)) {
            update_property(this->target_temperature,
                            this->read_target_temperature(RXData[RX_C0_BYTE_SET_TEMP], false), need_publish);
          }
          update_property(this->current_temperature, CalculateTemp(RXData[RX_C0_BYTE_T1_TEMP]), need_publish);
          if (fabs(this->current_temperature - CalculateTemp(RXData[RX_C0_BYTE_T2A_TEMP])) > 5.0) {
            // Compressor running
            if ((this->mode == climate::CLIMATE_MODE_HEAT) && (RXData[9] & 0x0F) != 0x00) {
              this->action = climate::CLIMATE_ACTION_HEATING;
              need_publish = true;
            } else if ((this->mode == climate::CLIMATE_MODE_COOL) && (RXData[9] & 0x0F) != 0x00) {
              this->action = climate::CLIMATE_ACTION_COOLING;
              need_publish = true;
            } else if ((this->mode == climate::CLIMATE_MODE_DRY) && (RXData[9] & 0x0F) != 0x00) {
              this->action = climate::CLIMATE_ACTION_DRYING;
              need_publish = true;
            }
          } else {
            if ((RXData[9] & 0x0F) != 0x00) {
              // This can be the case when in Fan only mode, or when release cold air is on in heat
              // Or anytime with cool...
              this->action = climate::CLIMATE_ACTION_FAN;
              need_publish = true;
            } else if ((this->action != climate::CLIMATE_ACTION_IDLE) && (RXData[9] & 0x0F) == 0x00) {
              this->action = climate::CLIMATE_ACTION_IDLE;
              need_publish = true;
            }
          }

          // Auto modes - FIXME needs more work.
          if ((this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
              ((RXData[RX_C0_BYTE_OP_MODE] & 0xEF) == OP_MODE_COOL) &&
              (this->action != climate::CLIMATE_ACTION_COOLING)) {
            this->action = climate::CLIMATE_ACTION_COOLING;
            need_publish = true;
          } else if ((this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                     ((RXData[RX_C0_BYTE_OP_MODE] & 0xEF) == OP_MODE_FAN) &&
                     (this->action != climate::CLIMATE_ACTION_FAN)) {
            this->action = climate::CLIMATE_ACTION_FAN;
            need_publish = true;
          } else if ((this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                     ((RXData[RX_C0_BYTE_OP_MODE] & 0xEF) == OP_MODE_HEAT) &&
                     (this->action != climate::CLIMATE_ACTION_HEATING)) {
            this->action = climate::CLIMATE_ACTION_HEATING;
            need_publish = true;
          }

          if ((this->swing_mode != ClimateSwingMode::CLIMATE_SWING_OFF) !=
              (bool) (RXData[RX_C0_BYTE_MODE_FLAGS] & MODE_FLAG_SWING))
            need_publish = true;
          this->swing_mode = (RXData[RX_C0_BYTE_MODE_FLAGS] & MODE_FLAG_SWING)
                                 ? ClimateSwingMode::CLIMATE_SWING_VERTICAL
                                 : ClimateSwingMode::CLIMATE_SWING_OFF;
          if (this->preset != preset)
            need_publish = true;
          this->preset = preset;
        } else if (mode == ClimateMode::CLIMATE_MODE_OFF && (this->action != climate::CLIMATE_ACTION_OFF)) {
          this->action = climate::CLIMATE_ACTION_OFF;
          need_publish = true;
        }

        if (need_publish)
          this->publish_state();

        set_sensor(this->temperature_2a_sensor_, CalculateTemp(RXData[RX_C0_BYTE_T2A_TEMP]));
        set_sensor(this->temperature_2b_sensor_, CalculateTemp(RXData[RX_C0_BYTE_T2B_TEMP]));
        set_sensor(this->temperature_3_sensor_, CalculateTemp(RXData[RX_C0_BYTE_T3_TEMP]));
        set_sensor(this->current_sensor_, RXData[RX_C0_BYTE_CURRENT]);
        set_sensor(this->timer_start_sensor_, CalculateGetTime(RXData[RX_C0_BYTE_TIMER_START]));
        set_sensor(this->timer_stop_sensor_, CalculateGetTime(RXData[RX_C0_BYTE_TIMER_STOP]));
        uint16_t error_flags = (RXData[RX_C0_BYTE_ERROR_FLAGS1] << 0) | (RXData[RX_C0_BYTE_ERROR_FLAGS2] << 8);
        set_sensor(this->error_flags_sensor_, error_flags);
        uint16_t protect_flags = (RXData[RX_C0_BYTE_PROTECT_FLAGS1] << 0) | (RXData[RX_C0_BYTE_PROTECT_FLAGS2] << 8);
#ifdef USE_BINARY_SENSOR
        set_binary_sensor(this->defrost_sensor_, (protect_flags & 0x02) == 2);
#endif
        set_sensor(this->protect_flags_sensor_, protect_flags);
#ifdef USE_TEXT_SENSOR
        // Fan speed as text for Home Assistant: Off, Low, Medium, High
        const char *fan_speed_text = "Off";
        switch (current_fan_speed) {
          case FAN_MODE_LOW:
            fan_speed_text = "Low";
            break;
          case FAN_MODE_MEDIUM:
            fan_speed_text = "Medium";
            break;
          case FAN_MODE_HIGH:
            fan_speed_text = "High";
            break;
          case FAN_MODE_OFF:
          default:
            fan_speed_text = "Off";
            break;
        }
        set_text_sensor(this->fan_speed_sensor_, fan_speed_text);
#endif
        break;
      }
      case 0xC4:
        bool need_publish = false;
        set_sensor(this->outdoor_sensor_, CalculateTemp(RXData[21]));
        set_number(this->static_pressure_number_, 0x0F & RXData[24]);
        if (this->mode != ClimateMode::CLIMATE_MODE_OFF ||
            ForceReadNextCycle == 1)  // Don't update below states unless mode is an ON state
        {
          if ((this->use_fahrenheit_) && (this->queuedCommand != STATE_SEND_C3)) {
            float incoming_target_temp = this->read_target_temperature(RXData[RX_C4_BYTE_SET_TEMP], true);
            if (incoming_target_temp != this->target_temperature) {
              need_publish = true;
              update_property(this->target_temperature, incoming_target_temp, need_publish);
            }
          }
        }
        // Field 15 goes from 0x01 to 0x00 when Follow-Me data is processed / procecessing.
        if (need_publish)
          this->publish_state();
        ForceReadNextCycle = 0;
        break;
    }
  } else {
    ESP_LOGE(Constants::TAG, "Received invalid response from AC");
  }
}

uint8_t AirConditioner::CalculateSetTime(uint32_t time) {
  uint32_t current_time = time;
  uint8_t timeValue = 0;

  if (0 < (current_time / 960)) {
    timeValue |= 0x40;
    current_time = current_time % 960;
  }
  if (0 < (current_time / 480)) {
    timeValue |= 0x20;
    current_time = current_time % 480;
  }
  if (0 < (current_time / 240)) {
    timeValue |= 0x10;
    current_time = current_time % 240;
  }
  if (0 < (current_time / 120)) {
    timeValue |= 0x08;
    current_time = current_time % 120;
  }
  if (0 < (current_time / 60)) {
    timeValue |= 0x04;
    current_time = current_time % 60;
  }
  if (0 < (current_time / 30)) {
    timeValue |= 0x02;
    current_time = current_time % 30;
  }
  if (0 < (current_time / 15)) {
    timeValue |= 0x01;
    current_time = current_time % 15;
  }
  return timeValue;
}

uint32_t AirConditioner::CalculateGetTime(uint8_t time) {
  uint32_t timeValue = 0;

  if (time & 0x40) {
    timeValue += 960;
  }
  if (time & 0x20) {
    timeValue += 480;
  }
  if (time & 0x10) {
    timeValue += 240;
  }
  if (time & 0x08) {
    timeValue += 120;
  }
  if (time & 0x04) {
    timeValue += 60;
  }
  if (time & 0x02) {
    timeValue += 30;
  }
  if (time & 0x01) {
    timeValue += 15;
  }
  return timeValue;
}

float AirConditioner::CalculateTemp(uint8_t byte) { return (byte - 0x28) / 2.0; }

uint8_t AirConditioner::EncodeVrfTemp(float celsius) {
  if (celsius < 17.0f) {
    celsius = 17.0f;
  } else if (celsius > 30.0f) {
    celsius = 30.0f;
  }

  return static_cast<uint8_t>(lroundf((celsius - 24.0f) * 2.0f) + 0x80);
}

float AirConditioner::DecodeVrfTemp(uint8_t byte) { return 24.0f + (static_cast<int>(byte) - 0x80) / 2.0f; }

bool AirConditioner::EncodeVrfMode(ClimateMode mode, uint8_t &nibble) {
  switch (mode) {
    case ClimateMode::CLIMATE_MODE_FAN_ONLY:
      nibble = 0x01;
      return true;
    case ClimateMode::CLIMATE_MODE_COOL:
      nibble = 0x02;
      return true;
    case ClimateMode::CLIMATE_MODE_HEAT:
      nibble = 0x03;
      return true;
    case ClimateMode::CLIMATE_MODE_DRY:
      nibble = 0x06;
      return true;
    default:
      return false;
  }
}

bool AirConditioner::DecodeVrfMode(uint8_t nibble, ClimateMode &mode) {
  switch (nibble) {
    case 0x01:
      mode = ClimateMode::CLIMATE_MODE_FAN_ONLY;
      return true;
    case 0x02:
      mode = ClimateMode::CLIMATE_MODE_COOL;
      return true;
    case 0x03:
      mode = ClimateMode::CLIMATE_MODE_HEAT;
      return true;
    case 0x06:
      mode = ClimateMode::CLIMATE_MODE_DRY;
      return true;
    default:
      return false;
  }
}

climate::ClimateTraits AirConditioner::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  traits.set_visual_min_temperature(17);
  traits.set_visual_max_temperature(30);
  traits.set_visual_temperature_step(this->vrf_protocol_ ? 0.5 : 1.0);

  if (this->vrf_protocol_) {
    auto supported_modes = this->supported_modes_;
    supported_modes.erase(ClimateMode::CLIMATE_MODE_HEAT_COOL);
    if (supported_modes.empty()) {
      supported_modes.insert({ClimateMode::CLIMATE_MODE_COOL, ClimateMode::CLIMATE_MODE_HEAT,
                              ClimateMode::CLIMATE_MODE_DRY, ClimateMode::CLIMATE_MODE_FAN_ONLY});
    }
    traits.set_supported_modes(supported_modes);
    if (!traits.get_supported_modes().empty())
      traits.add_supported_mode(ClimateMode::CLIMATE_MODE_OFF);
    return traits;
  }

  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_supported_modes(this->supported_modes_);
  traits.set_supported_swing_modes(this->supported_swing_modes_);
  traits.set_supported_presets(this->supported_presets_);
  traits.set_supported_custom_presets(this->supported_custom_presets_);
  traits.set_supported_custom_fan_modes(this->supported_custom_fan_modes_);
  /* + MINIMAL SET OF CAPABILITIES */
  traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_HIGH);
  traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_OFF);  // Can't set it but will be reported

  if (!traits.get_supported_modes().empty())
    traits.add_supported_mode(ClimateMode::CLIMATE_MODE_OFF);
  if (!traits.get_supported_swing_modes().empty())
    traits.add_supported_swing_mode(ClimateSwingMode::CLIMATE_SWING_OFF);
  if (!traits.get_supported_presets().empty())
    traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);

  return traits;
}

void AirConditioner::dump_config() {
  ESP_LOGCONFIG(Constants::TAG, "MideaXYE:");
  ESP_LOGCONFIG(Constants::TAG, "  [x] Protocol: %s", this->vrf_protocol_ ? "VRF" : "XYE");
  ESP_LOGCONFIG(Constants::TAG, "  [x] Period: %dms", this->get_update_interval());
  ESP_LOGCONFIG(Constants::TAG, "  [x] Response timeout: %dms", this->response_timeout);
  ESP_LOGCONFIG(Constants::TAG, "  [x] Use Fahrenheit: %d", this->use_fahrenheit_);

#ifdef USE_REMOTE_TRANSMITTER
  ESP_LOGCONFIG(Constants::TAG, "  [x] Using RemoteTransmitter");
#endif
  this->dump_traits_(Constants::TAG);
}

/* ACTIONS */

void AirConditioner::do_follow_me(float temperature, bool beeper) {
  if (this->vrf_protocol_) {
    ESP_LOGW(Constants::TAG, "Follow-Me is not implemented for VRF protocol.");
    return;
  }
#ifdef USE_REMOTE_TRANSMITTER
  ESP_LOGI(Constants::TAG, "Setting Follow-Me temperature to %.1f with beeper %d and remote transmitter", temperature, beeper);
  IrFollowMeData data(static_cast<uint8_t>(lroundf(temperature)), beeper);
  this->transmitter_.transmit(data);
#else
  ESP_LOGI(Constants::TAG, "Setting Follow-Me temperature to %.1f with beeper %d", temperature, beeper);
  

  prepareTXData(0xC6);


  if (followMeInit) {
    TXData[10] = 2;
  } else {
    TXData[10] = 6;
    followMeInit = true;
  }
  lastFollowMeTemperature = static_cast<uint8_t>(lroundf(temperature));
  TXData[11] = lastFollowMeTemperature;
  TXData[14] = CalculateCRC(TXData, TX_LEN);
  // Only send if mode is something other than off.
  // Wired controller does not send 0xC6 when off.
  if (this->mode != ClimateMode::CLIMATE_MODE_OFF) {
    if (controlState != STATE_WAIT_DATA) {
      controlState = STATE_SEND_C6;
    } else {
      queuedCommand = STATE_SEND_C6;
    }
    ESP_LOGI(Constants::TAG, "Queued Follow-Me data.");
  }
#endif
}

void AirConditioner::set_static_pressure(uint8_t static_pressure) {
  if (this->vrf_protocol_) {
    ESP_LOGW(Constants::TAG, "Static pressure control is not implemented for VRF protocol.");
    return;
  }

  if (static_pressure > 15) {
    ESP_LOGW(Constants::TAG, "Cannot set static pressure %d > 15", static_pressure);
    return;
  }

  prepareTXData(0xC6);
  TXData[8] = 0x10 | (static_pressure & 0x0F);
  TXData[10] = 4;
  TXData[11] = lastFollowMeTemperature;
  TXData[14] = CalculateCRC(TXData, TX_LEN);

  if (this->mode == ClimateMode::CLIMATE_MODE_OFF) {
    if (controlState != STATE_WAIT_DATA) {
      controlState = STATE_SEND_C6;
    } else {
      queuedCommand = STATE_SEND_C6;
    }
    ESP_LOGI(Constants::TAG, "Queued setting static pressure to %d", static_pressure);
  } else {
    ESP_LOGW(Constants::TAG, "Cannot set static pressure while unit is running");
  }
}

void AirConditioner::do_swing_step() {
  if (this->vrf_protocol_) {
    ESP_LOGW(Constants::TAG, "Swing step is not implemented for VRF protocol.");
    return;
  }
#ifdef USE_REMOTE_TRANSMITTER
  IrSpecialData data(0x01);
  this->transmitter_.transmit(data);
#else
  ESP_LOGW(Constants::TAG, "Action needs remote_transmitter component");
#endif
}

void AirConditioner::do_display_toggle() {
#ifdef USE_REMOTE_TRANSMITTER
  IrSpecialData data(0x08);
  this->transmitter_.transmit(data);
#else
  ESP_LOGW(Constants::TAG, "Action needs remote_transmitter component");
#endif
}

}  // namespace ac
}  // namespace midea
}  // namespace esphome

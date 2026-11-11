#include <Arduino.h>
#include "driver/twai.h"
#include <math.h>

static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_27;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_26;
static constexpr uint8_t PIN_CAN_SPEED = 23;
static constexpr uint8_t PIN_BOOST_ENABLE = 16;
static constexpr uint8_t PIN_RS485_TX = 22;
static constexpr uint8_t PIN_RS485_RX = 21;
static constexpr uint8_t PIN_RS485_RE = 17;
static constexpr uint8_t PIN_RS485_ENABLE = 19;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t RS485_BAUD = 115200;
static constexpr uint32_t DM_PARAMETER_CAN_ID = 0x7FF;
static constexpr uint8_t DM_READ_OPCODE = 0x33;
static constexpr uint16_t SAFE_TEST_MOTOR_ID = 1;
static constexpr float DM_PMAX = 12.5F;
static constexpr float DM_VMAX = 30.0F;
static constexpr float DM_TMAX = 10.0F;
static constexpr float SAFE_MAX_INITIAL_SPEED_RAD_S = 0.20F;
static constexpr float SAFE_MAX_POSITION_CHANGE_RAD = 0.05F;
static constexpr uint8_t SAFE_MAX_TEMPERATURE_C = 60;
static constexpr uint32_t SAFE_ENABLE_DURATION_MS = 250;
static constexpr float TINY_MOVE_DELTA_RAD = 0.02F;
static constexpr float TINY_MOVE_KP = 10.0F;
static constexpr float TINY_MOVE_KD = 0.2F;
// The DM absolute output coordinate crossed one full revolution while the
// gripper mechanics stayed at the same pose.  All calibrated endpoints are
// therefore rebased by +2*pi; physical travel and direction are unchanged.
static constexpr float GRIPPER_OPEN_HARD_RAD = -0.147920209F;
// Provisional replacement-gripper close ceiling.  The previous gripper's
// measured endpoint (-1.071374075) still left a visible gap on this unit, so
// calibration advances only in operator-authorized 0.05 rad increments.
static constexpr float GRIPPER_CLOSED_HARD_RAD = 5.311811232F;
static constexpr float GRIPPER_OPEN_TARGET_RAD = -0.097920209F;
// Keep 0.01 rad before the operator-measured closed hard endpoint.  The old
// 0.05 rad margin left the replacement gripper visibly short of full closure.
static constexpr float GRIPPER_CLOSE_TARGET_RAD = 5.301811232F;
static constexpr float GRIPPER_REVOLUTION_RAD = 6.283185307179586F;
static_assert(GRIPPER_CLOSE_TARGET_RAD < GRIPPER_CLOSED_HARD_RAD,
              "close target must remain before the measured hard endpoint");
// Guarded 3x profile.  The setpoint increment is 0.30 rad; every increment
// still requires fresh motor feedback and all hard-position/temperature/error
// checks remain active.  The legacy build keeps the verified 1.1 profile.
#ifdef GRIPPER_LEGACY_PROFILE
static constexpr float GRIPPER_MOVE_STEP_RAD = 0.04F;
static constexpr uint32_t GRIPPER_MOVE_DELAY_MS = 15;
static constexpr uint8_t GRIPPER_HOLD_MAX_STEPS = 20;
#else
static constexpr float GRIPPER_MOVE_STEP_RAD = 0.30F;
static constexpr uint32_t GRIPPER_MOVE_DELAY_MS = 10;
static constexpr uint8_t GRIPPER_HOLD_MAX_STEPS = 10;
#endif
static constexpr uint8_t GRIPPER_HOLD_MIN_STEPS = 3;
static constexpr float GRIPPER_MOVE_KP = 10.0F;
static constexpr float GRIPPER_MOVE_KD = 0.2F;
// Conservative contact hold: 0.04 rad preload at kp=4 is approximately
// 0.16 N*m motor-side proportional demand before reducer multiplication.
static constexpr float GRIPPER_CONTACT_PRELOAD_RAD = 0.04F;
static constexpr float GRIPPER_CONTACT_HOLD_KP = 4.0F;
static constexpr float GRIPPER_CONTACT_HOLD_KD = 0.2F;
static constexpr uint32_t GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS = 750;
static constexpr uint32_t GRIPPER_HOLD_PERIOD_MS = 20;
// A transition can reach its final position while the measured velocity still
// reflects the last ramp sample.  Give the low-gain hold controller a bounded
// interval to remove that residual velocity.  All other feedback limits and
// the host heartbeat remain mandatory during this interval.
static constexpr uint32_t GRIPPER_HOLD_SETTLE_TIMEOUT_MS = 1200;
// The 3x profile can briefly report up to three times the former 3.2 rad/s
// validation ceiling.  This remains far below the configured DM VMAX=30.
static constexpr float GRIPPER_MAX_FEEDBACK_SPEED_RAD_S = 9.6F;
static constexpr uint32_t GRIPPER_MOVE_TIMEOUT_MS = 10000;
static constexpr uint32_t CAN_ALERT_MASK =
    TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR |
    TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
    TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;

// RS485 request: D5 4D 52 51 ID_LO ID_HI RID XOR(first 7 bytes)
static constexpr size_t REQUEST_SIZE = 8;
static constexpr uint8_t REQUEST_HEADER[4] = {0xD5, 0x4D, 0x52, 0x51};
static constexpr uint8_t STATUS_HEADER[4] = {0xD5, 0x4D, 0x53, 0x51};
// Fixed, deliberately non-generic active-test commands. Both require motor ID
// 1 and a guard byte. No raw CAN forwarding or position target is accepted.
static constexpr uint8_t SAFE_ENABLE_HEADER[4] = {0xD5, 0x4D, 0x45, 0x51};
static constexpr uint8_t SAFE_DISABLE_HEADER[4] = {0xD5, 0x4D, 0x44, 0x51};
static constexpr uint8_t IDENT_HEADER[4] = {0xD5, 0x4D, 0x49, 0x51};
static constexpr uint8_t TINY_MOVE_HEADER[4] = {0xD5, 0x4D, 0x4D, 0x51};
static constexpr uint8_t GRIPPER_OPEN_HEADER[4] = {0xD5, 0x4D, 0x4F, 0x51};
static constexpr uint8_t GRIPPER_CLOSE_HEADER[4] = {0xD5, 0x4D, 0x43, 0x51};
static constexpr uint8_t GRIPPER_HEARTBEAT_HEADER[4] = {0xD5, 0x4D, 0x48, 0x51};
static constexpr uint8_t SAFE_ENABLE_GUARD = 0xA5;
static constexpr uint8_t SAFE_DISABLE_GUARD = 0x5A;
static constexpr uint8_t IDENT_GUARD = 0x60;
static constexpr uint8_t TINY_MOVE_GUARD = 0xB6;
static constexpr uint8_t GRIPPER_OPEN_GUARD = 0xC1;
static constexpr uint8_t GRIPPER_CLOSE_GUARD = 0xC2;
static constexpr uint8_t GRIPPER_HEARTBEAT_GUARD = 0xC3;

HardwareSerial RS485(1);
uint8_t request_buffer[REQUEST_SIZE]{};
size_t request_count = 0;
bool can_installed = false;
bool can_started = false;
uint32_t current_can_kbps = 0;

struct MotorFeedback {
  bool valid = false;
  uint8_t error = 0xFF;
  float position_rad = 0.0F;
  float velocity_rad_s = 0.0F;
  float torque_nm = 0.0F;
  uint8_t mos_temperature_c = 0;
  uint8_t rotor_temperature_c = 0;
};

static uint8_t xor_checksum(const uint8_t *data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) checksum ^= data[i];
  return checksum;
}

static void print_hex_byte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static void print_bytes(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    print_hex_byte(data[i]);
    if (i + 1 < length) Serial.print(' ');
  }
}

static float uint_to_float(uint16_t value, float minimum, float maximum,
                           uint8_t bits) {
  return static_cast<float>(value) * (maximum - minimum) /
             static_cast<float>((1UL << bits) - 1UL) +
         minimum;
}

static uint16_t float_to_uint(float value, float minimum, float maximum,
                              uint8_t bits) {
  if (value < minimum) value = minimum;
  if (value > maximum) value = maximum;
  const float normalized = (value - minimum) / (maximum - minimum);
  return static_cast<uint16_t>(normalized * ((1UL << bits) - 1UL));
}

static bool decode_motor_feedback(const twai_message_t &frame,
                                  uint16_t expected_motor_id,
                                  MotorFeedback &feedback) {
  if (frame.extd || frame.rtr || frame.data_length_code != 8) return false;
  if ((frame.data[0] & 0x0F) != (expected_motor_id & 0x0F)) return false;
  const uint16_t position_raw =
      (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
  const uint16_t velocity_raw =
      (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const uint16_t torque_raw =
      (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];
  feedback.valid = true;
  feedback.error = (frame.data[0] >> 4) & 0x0F;
  feedback.position_rad =
      uint_to_float(position_raw, -DM_PMAX, DM_PMAX, 16);
  feedback.velocity_rad_s =
      uint_to_float(velocity_raw, -DM_VMAX, DM_VMAX, 12);
  feedback.torque_nm = uint_to_float(torque_raw, -DM_TMAX, DM_TMAX, 12);
  feedback.mos_temperature_c = frame.data[6];
  feedback.rotor_temperature_c = frame.data[7];
  return true;
}

static void log_feedback(const char *phase, const MotorFeedback &feedback) {
  Serial.printf(
      "SAFE_FEEDBACK phase=%s error=%u position_rad=%.6f velocity_rad_s=%.6f "
      "torque_nm=%.6f mos_c=%u rotor_c=%u\n",
      phase, feedback.error, feedback.position_rad, feedback.velocity_rad_s,
      feedback.torque_nm, feedback.mos_temperature_c,
      feedback.rotor_temperature_c);
}

static twai_timing_config_t timing_for_kbps(uint32_t kbps) {
  switch (kbps) {
    case 125: {
      twai_timing_config_t timing = TWAI_TIMING_CONFIG_125KBITS();
      return timing;
    }
    case 200:
      // 80 MHz / (BRP 20 * 20 time quanta) = 200 kbit/s.
      return twai_timing_config_t{20, 15, 4, 3, false};
    case 250: {
      twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
      return timing;
    }
    case 500: {
      twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
      return timing;
    }
    default: {
      twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
      return timing;
    }
  }
}

static void shutdown_can() {
  if (can_started) {
    twai_stop();
    can_started = false;
  }
  if (can_installed) {
    twai_driver_uninstall();
    can_installed = false;
  }
  current_can_kbps = 0;
}

static bool initialize_can(uint32_t kbps) {
  shutdown_can();
  pinMode(PIN_BOOST_ENABLE, OUTPUT);
  digitalWrite(PIN_BOOST_ENABLE, HIGH);
  pinMode(PIN_CAN_SPEED, OUTPUT);
  digitalWrite(PIN_CAN_SPEED, LOW);

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  general.tx_queue_len = 4;
  general.rx_queue_len = 32;
  general.alerts_enabled = CAN_ALERT_MASK;
  twai_timing_config_t timing = timing_for_kbps(kbps);
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  Serial.printf("CAN install=%s mode=NORMAL bitrate_kbps=%lu\n",
                esp_err_to_name(result), static_cast<unsigned long>(kbps));
  if (result != ESP_OK) return false;
  can_installed = true;
  result = twai_start();
  Serial.printf("CAN start=%s\n", esp_err_to_name(result));
  if (result != ESP_OK) return false;
  can_started = true;
  current_can_kbps = kbps;
  return true;
}

static const char *can_state_name(twai_state_t state) {
  switch (state) {
    case TWAI_STATE_STOPPED:
      return "STOPPED";
    case TWAI_STATE_RUNNING:
      return "RUNNING";
    case TWAI_STATE_BUS_OFF:
      return "BUS_OFF";
    case TWAI_STATE_RECOVERING:
      return "RECOVERING";
    default:
      return "UNKNOWN";
  }
}

static void print_can_status(const char *reason) {
  twai_status_info_t status{};
  esp_err_t result = twai_get_status_info(&status);
  if (result != ESP_OK) {
    Serial.printf("CAN_STATUS reason=%s result=%s\n", reason,
                  esp_err_to_name(result));
    return;
  }
  Serial.printf(
      "CAN_STATUS reason=%s state=%s tx_queue=%lu rx_queue=%lu tx_err=%lu "
      "rx_err=%lu tx_failed=%lu bus_errors=%lu arb_lost=%lu\n",
      reason, can_state_name(status.state),
      static_cast<unsigned long>(status.msgs_to_tx),
      static_cast<unsigned long>(status.msgs_to_rx),
      static_cast<unsigned long>(status.tx_error_counter),
      static_cast<unsigned long>(status.rx_error_counter),
      static_cast<unsigned long>(status.tx_failed_count),
      static_cast<unsigned long>(status.bus_error_count),
      static_cast<unsigned long>(status.arb_lost_count));
}

static void log_can_alerts(uint32_t alerts) {
  Serial.printf("CAN_ALERTS mask=0x%08lX", static_cast<unsigned long>(alerts));
  if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.print(" TX_SUCCESS");
  if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
  if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) Serial.print(" ABOVE_ERR_WARN");
  if (alerts & TWAI_ALERT_ERR_PASS) Serial.print(" ERR_PASS");
  if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
  if (alerts & TWAI_ALERT_ARB_LOST) Serial.print(" ARB_LOST");
  if (alerts & TWAI_ALERT_RX_DATA) Serial.print(" RX_DATA");
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.print(" RX_QUEUE_FULL");
  Serial.println();
  print_can_status("alert");
}

static void poll_can_alerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0) return;
  log_can_alerts(alerts);
}

static void initialize_rs485() {
  pinMode(PIN_RS485_ENABLE, OUTPUT);
  pinMode(PIN_RS485_RE, OUTPUT);
  digitalWrite(PIN_RS485_ENABLE, HIGH);
  digitalWrite(PIN_RS485_RE, HIGH);
  RS485.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  Serial.printf("RS485 started: %lu 8N1\n",
                static_cast<unsigned long>(RS485_BAUD));
}

static void send_gateway_event(uint8_t event, uint8_t result,
                               uint8_t motor_error, uint8_t flags,
                               float position_rad, float target_rad) {
  // Network-visible response over RS485/TCP60000:
  // D5 4D 47 50 EVENT RESULT ERROR FLAGS POS_F32 TARGET_F32 XOR
  uint8_t packet[17]{};
  packet[0] = 0xD5;
  packet[1] = 0x4D;
  packet[2] = 0x47;
  packet[3] = 0x50;
  packet[4] = event;
  packet[5] = result;
  packet[6] = motor_error;
  packet[7] = flags;
  memcpy(&packet[8], &position_rad, sizeof(position_rad));
  memcpy(&packet[12], &target_rad, sizeof(target_rad));
  packet[16] = xor_checksum(packet, 16);
  RS485.write(packet, sizeof(packet));
  RS485.flush();
}

static void send_identity_response() {
  // event=1, result=1, error=0, flags=0x10 identifies the protocol-compatible
  // guarded gateway build 1.x family.
  // The position field is diagnostic-only and reports the selected profile.
  // Existing hosts validate event/result/flags and remain protocol-compatible.
#ifdef GRIPPER_LEGACY_PROFILE
  send_gateway_event(0x01, 0x01, 0x00, 0x10, 1.1F, 0.0F);
#else
  send_gateway_event(0x01, 0x01, 0x00, 0x10, 3.7F, 0.0F);
#endif
}

static bool emergency_stop_requested() {
  if (RS485.available() < static_cast<int>(REQUEST_SIZE)) return false;
  uint8_t packet[REQUEST_SIZE]{};
  for (size_t i = 0; i < REQUEST_SIZE; ++i) {
    packet[i] = static_cast<uint8_t>(RS485.read());
  }
  const bool valid =
      memcmp(packet, SAFE_DISABLE_HEADER, sizeof(SAFE_DISABLE_HEADER)) == 0 &&
      packet[4] == 1 && packet[5] == 0 &&
      packet[6] == SAFE_DISABLE_GUARD &&
      xor_checksum(packet, REQUEST_SIZE - 1) == packet[7];
  if (valid) Serial.println("GRIPPER emergency_stop_request_received");
  return valid;
}

enum class HoldRequest : uint8_t {
  NONE = 0,
  STOP = 1,
  HEARTBEAT = 2,
  OPEN = 3,
  CLOSE = 4,
};

static HoldRequest read_hold_request() {
  if (RS485.available() < static_cast<int>(REQUEST_SIZE)) {
    return HoldRequest::NONE;
  }
  uint8_t packet[REQUEST_SIZE]{};
  for (size_t i = 0; i < REQUEST_SIZE; ++i) {
    packet[i] = static_cast<uint8_t>(RS485.read());
  }
  if (xor_checksum(packet, REQUEST_SIZE - 1) != packet[7] ||
      packet[4] != 1 || packet[5] != 0) {
    Serial.println("GRIPPER_HOLD ignored_invalid_request");
    return HoldRequest::NONE;
  }
  if (memcmp(packet, SAFE_DISABLE_HEADER, sizeof(SAFE_DISABLE_HEADER)) == 0 &&
      packet[6] == SAFE_DISABLE_GUARD) {
    Serial.println("GRIPPER_HOLD stop_request_received");
    return HoldRequest::STOP;
  }
  if (memcmp(packet, GRIPPER_HEARTBEAT_HEADER,
             sizeof(GRIPPER_HEARTBEAT_HEADER)) == 0 &&
      packet[6] == GRIPPER_HEARTBEAT_GUARD) {
    return HoldRequest::HEARTBEAT;
  }
  if (memcmp(packet, GRIPPER_OPEN_HEADER, sizeof(GRIPPER_OPEN_HEADER)) == 0 &&
      packet[6] == GRIPPER_OPEN_GUARD) {
    Serial.println("GRIPPER_HOLD controlled_open_request_received");
    return HoldRequest::OPEN;
  }
  if (memcmp(packet, GRIPPER_CLOSE_HEADER, sizeof(GRIPPER_CLOSE_HEADER)) == 0 &&
      packet[6] == GRIPPER_CLOSE_GUARD) {
    Serial.println("GRIPPER_HOLD controlled_close_request_received");
    return HoldRequest::CLOSE;
  }
  Serial.println("GRIPPER_HOLD ignored_non_hold_request");
  return HoldRequest::NONE;
}

static void drain_can_rx() {
  twai_message_t ignored{};
  while (twai_receive(&ignored, 0) == ESP_OK) {
  }
}

static bool transmit_and_receive_feedback(const twai_message_t &frame,
                                          uint16_t motor_id,
                                          MotorFeedback &feedback,
                                          const char *phase,
                                          uint32_t feedback_timeout_ms = 120) {
  drain_can_rx();
  const esp_err_t tx_result = twai_transmit(&frame, pdMS_TO_TICKS(20));
  Serial.printf("SAFE_TX phase=%s can_id=0x%03lX result=%s data=", phase,
                static_cast<unsigned long>(frame.identifier),
                esp_err_to_name(tx_result));
  print_bytes(frame.data, frame.data_length_code);
  Serial.println();
  if (tx_result != ESP_OK) return false;

  const uint32_t deadline = millis() + feedback_timeout_ms;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    twai_message_t response{};
    if (twai_receive(&response, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    Serial.printf("SAFE_RX phase=%s can_id=0x%03lX dlc=%u data=", phase,
                  static_cast<unsigned long>(response.identifier),
                  response.data_length_code);
    print_bytes(response.data, response.data_length_code);
    Serial.println();
    if (decode_motor_feedback(response, motor_id, feedback)) {
      log_feedback(phase, feedback);
      return true;
    }
  }
  Serial.printf("SAFE_ABORT phase=%s reason=no_valid_feedback\n", phase);
  return false;
}

static twai_message_t status_frame(uint16_t motor_id) {
  twai_message_t frame{};
  frame.identifier = DM_PARAMETER_CAN_ID;
  frame.extd = 0;
  frame.rtr = 0;
  frame.ss = 1;
  frame.data_length_code = 8;
  frame.data[0] = static_cast<uint8_t>(motor_id & 0xFF);
  frame.data[1] = static_cast<uint8_t>((motor_id >> 8) & 0x07);
  frame.data[2] = 0xCC;
  return frame;
}

static twai_message_t control_command_frame(uint16_t motor_id, uint8_t command) {
  twai_message_t frame{};
  frame.identifier = motor_id;
  frame.extd = 0;
  frame.rtr = 0;
  frame.ss = 1;
  frame.data_length_code = 8;
  for (uint8_t i = 0; i < 7; ++i) frame.data[i] = 0xFF;
  frame.data[7] = command;
  return frame;
}

static twai_message_t mit_frame(uint16_t motor_id, float position_rad,
                                float kp_value, float kd_value) {
  const uint16_t q = float_to_uint(position_rad, -DM_PMAX, DM_PMAX, 16);
  const uint16_t dq = float_to_uint(0.0F, -DM_VMAX, DM_VMAX, 12);
  const uint16_t kp = float_to_uint(kp_value, 0.0F, 500.0F, 12);
  const uint16_t kd = float_to_uint(kd_value, 0.0F, 5.0F, 12);
  const uint16_t tau = float_to_uint(0.0F, -DM_TMAX, DM_TMAX, 12);
  twai_message_t frame{};
  frame.identifier = motor_id;
  frame.extd = 0;
  frame.rtr = 0;
  frame.ss = 1;
  frame.data_length_code = 8;
  frame.data[0] = (q >> 8) & 0xFF;
  frame.data[1] = q & 0xFF;
  frame.data[2] = dq >> 4;
  frame.data[3] = ((dq & 0x0F) << 4) | ((kp >> 8) & 0x0F);
  frame.data[4] = kp & 0xFF;
  frame.data[5] = kd >> 4;
  frame.data[6] = ((kd & 0x0F) << 4) | ((tau >> 8) & 0x0F);
  frame.data[7] = tau & 0xFF;
  return frame;
}

static twai_message_t zero_torque_mit_frame(uint16_t motor_id,
                                            float current_position_rad) {
  // Position comes from fresh feedback but cannot create torque while kp=0.
  return mit_frame(motor_id, current_position_rad, 0.0F, 0.0F);
}

static bool send_disable(uint16_t motor_id, const char *phase) {
  MotorFeedback feedback{};
  const twai_message_t frame = control_command_frame(motor_id, 0xFD);
  // A failed first attempt is retried once because disabling is the safe
  // direction. Enable and MIT control frames are never automatically retried.
  if (transmit_and_receive_feedback(frame, motor_id, feedback, phase)) {
    return true;
  }
  delay(10);
  Serial.println("SAFE_DISABLE retry=1");
  return transmit_and_receive_feedback(frame, motor_id, feedback,
                                       "disable_retry");
}

static void run_zero_torque_enable_test(uint16_t motor_id) {
  Serial.println("SAFE_TEST_BEGIN type=zero_torque_enable_250ms");
  if (motor_id != SAFE_TEST_MOTOR_ID) {
    Serial.println("SAFE_TEST_RESULT REJECT reason=motor_id_not_fixed_1");
    return;
  }
  if (!initialize_can(1000)) {
    Serial.println("SAFE_TEST_RESULT FAIL reason=can_init");
    return;
  }

  MotorFeedback initial{};
  if (!transmit_and_receive_feedback(status_frame(motor_id), motor_id, initial,
                                     "preflight_status")) {
    Serial.println("SAFE_TEST_RESULT FAIL reason=preflight_no_feedback");
    send_disable(motor_id, "disable_after_preflight_failure");
    return;
  }
  if (initial.error != 0) {
    Serial.printf("SAFE_TEST_RESULT REJECT reason=preflight_error_%u\n",
                  initial.error);
    send_disable(motor_id, "disable_preflight_error");
    return;
  }
  if (fabsf(initial.velocity_rad_s) > SAFE_MAX_INITIAL_SPEED_RAD_S) {
    Serial.println("SAFE_TEST_RESULT REJECT reason=initial_speed_limit");
    send_disable(motor_id, "disable_initial_speed");
    return;
  }
  if (initial.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      initial.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C) {
    Serial.println("SAFE_TEST_RESULT REJECT reason=temperature_limit");
    send_disable(motor_id, "disable_temperature");
    return;
  }

  const twai_message_t neutral =
      zero_torque_mit_frame(motor_id, initial.position_rad);
  MotorFeedback neutral_feedback{};
  if (!transmit_and_receive_feedback(neutral, motor_id, neutral_feedback,
                                     "preload_zero_torque")) {
    Serial.println("SAFE_TEST_RESULT FAIL reason=neutral_preload");
    send_disable(motor_id, "disable_neutral_preload_failure");
    return;
  }

  MotorFeedback enable_feedback{};
  const twai_message_t enable_frame = control_command_frame(motor_id, 0xFC);
  if (!transmit_and_receive_feedback(enable_frame, motor_id, enable_feedback,
                                     "enable")) {
    Serial.println("SAFE_TEST_RESULT FAIL reason=enable_no_feedback");
    send_disable(motor_id, "disable_enable_failure");
    return;
  }

  bool active_ok = enable_feedback.error == 1;
  const uint32_t active_start = millis();
  uint8_t cycle = 0;
  while (active_ok && millis() - active_start < SAFE_ENABLE_DURATION_MS) {
    MotorFeedback live{};
    char phase[32]{};
    snprintf(phase, sizeof(phase), "zero_torque_%u", cycle++);
    if (!transmit_and_receive_feedback(neutral, motor_id, live, phase)) {
      active_ok = false;
      break;
    }
    if (live.error != 1 ||
        fabsf(live.position_rad - initial.position_rad) >
            SAFE_MAX_POSITION_CHANGE_RAD ||
        fabsf(live.velocity_rad_s) > 1.0F ||
        live.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
        live.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C) {
      Serial.println("SAFE_ABORT reason=live_limit");
      active_ok = false;
      break;
    }
    delay(10);
  }

  const bool disable_feedback = send_disable(motor_id, "disable_finally");
  delay(20);
  MotorFeedback final_status{};
  const bool final_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, final_status, "final_status");
  const bool safely_disabled = final_feedback && final_status.error == 0;
  Serial.printf(
      "SAFE_TEST_RESULT %s active_ok=%u disable_feedback=%u "
      "verified_disabled=%u position_change_rad=%.6f\n",
      (active_ok && disable_feedback && safely_disabled) ? "PASS" : "FAIL",
      active_ok, disable_feedback, safely_disabled,
      final_feedback ? final_status.position_rad - initial.position_rad : 999.0F);
}

static bool tiny_move_feedback_ok(const MotorFeedback &live,
                                  const MotorFeedback &initial) {
  return live.error == 1 &&
         fabsf(live.position_rad - initial.position_rad) <= 0.08F &&
         fabsf(live.velocity_rad_s) <= 2.0F &&
         live.mos_temperature_c < SAFE_MAX_TEMPERATURE_C &&
         live.rotor_temperature_c < SAFE_MAX_TEMPERATURE_C;
}

static void run_tiny_move_test(uint16_t motor_id) {
  Serial.println("TINY_MOVE_BEGIN delta_rad=0.02 kp=10 kd=0.2 return=1");
  if (motor_id != SAFE_TEST_MOTOR_ID || !initialize_can(1000)) {
    Serial.println("TINY_MOVE_RESULT FAIL reason=id_or_can_init");
    return;
  }

  MotorFeedback initial{};
  if (!transmit_and_receive_feedback(status_frame(motor_id), motor_id, initial,
                                     "move_preflight") ||
      initial.error != 0 ||
      fabsf(initial.velocity_rad_s) > SAFE_MAX_INITIAL_SPEED_RAD_S ||
      initial.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      initial.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      initial.position_rad + TINY_MOVE_DELTA_RAD > DM_PMAX) {
    Serial.println("TINY_MOVE_RESULT REJECT reason=preflight");
    send_disable(motor_id, "move_preflight_disable");
    return;
  }

  MotorFeedback feedback{};
  const twai_message_t neutral =
      zero_torque_mit_frame(motor_id, initial.position_rad);
  if (!transmit_and_receive_feedback(neutral, motor_id, feedback,
                                     "move_preload") ||
      !transmit_and_receive_feedback(control_command_frame(motor_id, 0xFC),
                                     motor_id, feedback, "move_enable") ||
      feedback.error != 1) {
    Serial.println("TINY_MOVE_RESULT FAIL reason=preload_or_enable");
    send_disable(motor_id, "move_enable_failure_disable");
    return;
  }

  bool active_ok = true;
  float maximum_excursion = 0.0F;
  uint8_t sequence = 0;
  // Ten small setpoint increments out, five settling frames, ten increments
  // back, and five return-settling frames. Maximum proportional demand from
  // the commanded 0.02 rad error is about 0.2 Nm.
  for (int step = 1; step <= 30 && active_ok; ++step) {
    float fraction = 0.0F;
    if (step <= 10) {
      fraction = static_cast<float>(step) / 10.0F;
    } else if (step <= 15) {
      fraction = 1.0F;
    } else if (step <= 25) {
      fraction = static_cast<float>(25 - step) / 10.0F;
    }
    const float target = initial.position_rad + TINY_MOVE_DELTA_RAD * fraction;
    const twai_message_t command =
        mit_frame(motor_id, target, TINY_MOVE_KP, TINY_MOVE_KD);
    char phase[32]{};
    snprintf(phase, sizeof(phase), "tiny_move_%u", sequence++);
    MotorFeedback live{};
    if (!transmit_and_receive_feedback(command, motor_id, live, phase) ||
        !tiny_move_feedback_ok(live, initial)) {
      Serial.println("TINY_MOVE_ABORT reason=feedback_limit");
      active_ok = false;
      break;
    }
    const float excursion = fabsf(live.position_rad - initial.position_rad);
    if (excursion > maximum_excursion) maximum_excursion = excursion;
    delay(15);
  }

  const bool disable_feedback = send_disable(motor_id, "move_disable_finally");
  delay(20);
  MotorFeedback final_status{};
  const bool final_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, final_status, "move_final_status");
  const bool safely_disabled = final_feedback && final_status.error == 0;
  const float final_offset =
      final_feedback ? final_status.position_rad - initial.position_rad : 999.0F;
  const bool motion_observed = maximum_excursion >= 0.003F;
  const bool returned = final_feedback && fabsf(final_offset) <= 0.03F;
  Serial.printf(
      "TINY_MOVE_RESULT %s active_ok=%u motion_observed=%u "
      "max_excursion_rad=%.6f final_offset_rad=%.6f verified_disabled=%u\n",
      (active_ok && motion_observed && returned && disable_feedback &&
       safely_disabled)
          ? "PASS"
          : "FAIL",
      active_ok, motion_observed, maximum_excursion, final_offset,
      safely_disabled);
}

static bool select_gripper_branch(float position_rad, float &branch_offset_rad) {
  // The same mechanical pose can be reported on an adjacent 2*pi branch after
  // a motor power cycle. Calibrated travel is shorter than one revolution, so
  // only one shifted interval can contain a valid pose.
  for (int branch = -2; branch <= 2; ++branch) {
    const float offset = branch * GRIPPER_REVOLUTION_RAD;
    if (position_rad >= GRIPPER_OPEN_HARD_RAD + offset - 0.10F &&
        position_rad <= GRIPPER_CLOSED_HARD_RAD + offset + 0.10F) {
      branch_offset_rad = offset;
      return true;
    }
  }
  return false;
}

static bool gripper_feedback_ok(const MotorFeedback &feedback,
                                float branch_offset_rad) {
  return feedback.error == 1 &&
         feedback.position_rad >=
             GRIPPER_OPEN_HARD_RAD + branch_offset_rad - 0.10F &&
         feedback.position_rad <=
             GRIPPER_CLOSED_HARD_RAD + branch_offset_rad + 0.10F &&
          fabsf(feedback.velocity_rad_s) <= GRIPPER_MAX_FEEDBACK_SPEED_RAD_S &&
         feedback.mos_temperature_c < SAFE_MAX_TEMPERATURE_C &&
         feedback.rotor_temperature_c < SAFE_MAX_TEMPERATURE_C;
}

// Persistent teleoperation session. Both OPEN and CLOSE/CONTACT are active
// low-gain hold states. Opposite commands transition directly while enabled;
// only STOP, watchdog loss, feedback fault, or a safety limit disables.
static void run_gripper_active_session(uint16_t motor_id,
                                       uint8_t initial_event,
                                       MotorFeedback feedback,
                                       float initial_hold_target_rad,
                                       float branch_offset_rad,
                                       bool initial_contact) {
  uint8_t current_event = initial_event;
  float hold_target_rad = initial_hold_target_rad;
  bool contact_state = initial_contact;
  bool healthy = true;
  bool stop_requested = false;
  uint32_t last_heartbeat_ms = millis();

  while (healthy && !stop_requested) {
    const uint32_t hold_settle_started_ms = millis();
    const twai_message_t hold_command = mit_frame(
        motor_id, hold_target_rad, GRIPPER_CONTACT_HOLD_KP,
        GRIPPER_CONTACT_HOLD_KD);
    if (!transmit_and_receive_feedback(hold_command, motor_id, feedback,
                                       "active_session_hold_start") ||
        !gripper_feedback_ok(feedback, branch_offset_rad)) {
      healthy = false;
      break;
    }
    bool hold_settling = fabsf(feedback.velocity_rad_s) > 1.0F;

    // result=4 is object contact; result=5 is an endpoint hold. flags bit1
    // means that the motor remains enabled under heartbeat supervision.
    send_gateway_event(current_event, contact_state ? 0x04 : 0x05, 0x00,
                       0x02, feedback.position_rad, hold_target_rad);

    bool transition_requested = false;
    uint8_t next_event = current_event;
    while (healthy && !stop_requested && !transition_requested) {
      const HoldRequest request = read_hold_request();
      if (request == HoldRequest::STOP) {
        stop_requested = true;
        break;
      }
      if (request == HoldRequest::HEARTBEAT) {
        last_heartbeat_ms = millis();
      } else if (request == HoldRequest::OPEN && current_event != 0x10) {
        next_event = 0x10;
        transition_requested = true;
        break;
      } else if (request == HoldRequest::CLOSE && current_event != 0x11) {
        next_event = 0x11;
        transition_requested = true;
        break;
      }
      if (millis() - last_heartbeat_ms > GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS) {
        Serial.println("GRIPPER_SESSION_ABORT reason=heartbeat_timeout");
        healthy = false;
        break;
      }
      MotorFeedback live{};
      if (!transmit_and_receive_feedback(hold_command, motor_id, live,
                                         "active_session_hold") ||
          !gripper_feedback_ok(live, branch_offset_rad)) {
        Serial.println("GRIPPER_SESSION_ABORT reason=hold_feedback_limit");
        healthy = false;
        break;
      }
      const float hold_speed = fabsf(live.velocity_rad_s);
      if (hold_settling) {
        if (hold_speed <= 1.0F) {
          hold_settling = false;
        } else if (millis() - hold_settle_started_ms >
                   GRIPPER_HOLD_SETTLE_TIMEOUT_MS) {
          Serial.println("GRIPPER_SESSION_ABORT reason=hold_settle_timeout");
          healthy = false;
          break;
        }
      } else if (hold_speed > 1.0F) {
        // Once stable, a new unexplained high-speed excursion is not a normal
        // transition and remains a fail-safe condition.
        Serial.println("GRIPPER_SESSION_ABORT reason=hold_speed_excursion");
        healthy = false;
        break;
      }
      feedback = live;
      delay(GRIPPER_HOLD_PERIOD_MS);
    }
    if (!healthy || stop_requested || !transition_requested) break;

    // Remove preload at the actual pose, then move directly to the opposite
    // state without disabling the motor.
    const twai_message_t unload_command = mit_frame(
        motor_id, feedback.position_rad, GRIPPER_CONTACT_HOLD_KP,
        GRIPPER_CONTACT_HOLD_KD);
    if (!transmit_and_receive_feedback(unload_command, motor_id, feedback,
                                       "active_session_unload")) {
      healthy = false;
      break;
    }
    delay(2 * GRIPPER_HOLD_PERIOD_MS);

    const float move_start_rad = feedback.position_rad;
    const float move_target_rad =
        (next_event == 0x10 ? GRIPPER_OPEN_TARGET_RAD
                            : GRIPPER_CLOSE_TARGET_RAD) +
        branch_offset_rad;
    const float delta = move_target_rad - move_start_rad;
    uint16_t steps =
        static_cast<uint16_t>(ceilf(fabsf(delta) / GRIPPER_MOVE_STEP_RAD));
    if (steps < 1) steps = 1;
    if (steps > 300) steps = 300;
    uint8_t stall_samples = 0;
    float previous_position = feedback.position_rad;
    bool contact_detected = false;

    for (uint16_t step = 1; step <= steps && healthy && !stop_requested;
         ++step) {
      const HoldRequest request = read_hold_request();
      if (request == HoldRequest::STOP) {
        stop_requested = true;
        break;
      }
      if (request == HoldRequest::HEARTBEAT) {
        last_heartbeat_ms = millis();
      }
      if (millis() - last_heartbeat_ms > GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS) {
        Serial.println("GRIPPER_SESSION_ABORT reason=transition_heartbeat");
        healthy = false;
        break;
      }
      const float command_position =
          move_start_rad + delta * static_cast<float>(step) /
                               static_cast<float>(steps);
      const float kp = next_event == 0x11 ? 6.0F : GRIPPER_MOVE_KP;
      const twai_message_t command =
          mit_frame(motor_id, command_position, kp, GRIPPER_MOVE_KD);
      MotorFeedback live{};
      if (!transmit_and_receive_feedback(command, motor_id, live,
                                         "active_session_transition") ||
          !gripper_feedback_ok(live, branch_offset_rad)) {
        healthy = false;
        break;
      }
      if (next_event == 0x11) {
        const float tracking_error = command_position - live.position_rad;
        const float progress = live.position_rad - previous_position;
        if (tracking_error >= 0.16F &&
            fabsf(live.velocity_rad_s) <= 0.50F && progress <= 0.02F) {
          ++stall_samples;
        } else {
          stall_samples = 0;
        }
        if (stall_samples >= 2) {
          contact_detected = true;
          feedback = live;
          break;
        }
      }
      previous_position = live.position_rad;
      feedback = live;
      delay(GRIPPER_MOVE_DELAY_MS);
    }
    if (!healthy || stop_requested) break;

    if (contact_detected) {
      hold_target_rad = fminf(
          move_target_rad,
          feedback.position_rad + GRIPPER_CONTACT_PRELOAD_RAD);
      contact_state = true;
    } else {
      // Let the low-gain state controller finish the last small tracking gap.
      hold_target_rad = move_target_rad;
      contact_state = false;
    }
    current_event = next_event;
  }

  send_disable(motor_id, "active_session_disable_finally");
  delay(50);
  MotorFeedback final_status{};
  const bool final_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, final_status,
      "active_session_final_status");
  const bool safely_disabled = final_feedback && final_status.error == 0;
  const bool clean_stop = stop_requested && healthy && safely_disabled;
  send_gateway_event(0x12, clean_stop ? 0x01 : 0x02, final_status.error,
                     safely_disabled ? 0x01 : 0x00,
                     final_feedback ? final_status.position_rad : 999.0F,
                     hold_target_rad);
}

static void run_gripper_contact_hold(uint16_t motor_id, uint8_t event,
                                     float contact_position_rad,
                                     float close_target_rad,
                                     float branch_offset_rad) {
  const float direction = close_target_rad >= contact_position_rad ? 1.0F : -1.0F;
  float hold_target_rad =
      contact_position_rad + direction * GRIPPER_CONTACT_PRELOAD_RAD;
  if (direction > 0.0F) {
    hold_target_rad = fminf(hold_target_rad, close_target_rad);
  } else {
    hold_target_rad = fmaxf(hold_target_rad, close_target_rad);
  }

  MotorFeedback feedback{};
  const twai_message_t neutral =
      zero_torque_mit_frame(motor_id, contact_position_rad);
  const bool preloaded = transmit_and_receive_feedback(
      neutral, motor_id, feedback, "contact_hold_preload");
  const bool enabled = preloaded && transmit_and_receive_feedback(
      control_command_frame(motor_id, 0xFC), motor_id, feedback,
      "contact_hold_enable");
  const twai_message_t hold_command = mit_frame(
      motor_id, hold_target_rad, GRIPPER_CONTACT_HOLD_KP,
      GRIPPER_CONTACT_HOLD_KD);
  const bool first_hold = enabled && feedback.error == 1 &&
      transmit_and_receive_feedback(hold_command, motor_id, feedback,
                                    "contact_hold_start");
  if (!first_hold || !gripper_feedback_ok(feedback, branch_offset_rad) ||
      fabsf(feedback.velocity_rad_s) > 1.0F) {
    send_disable(motor_id, "contact_hold_start_failure_disable");
    send_gateway_event(event, 0x02, feedback.error, 0,
                       feedback.position_rad, hold_target_rad);
    return;
  }

  run_gripper_active_session(motor_id, event, feedback, hold_target_rad,
                             branch_offset_rad, true);
  return;

  // flags bit1 means actively holding. A heartbeat watchdog guarantees that
  // loss of the host/controller connection releases the object within 750 ms.
  send_gateway_event(event, 0x04, 0x00, 0x02, feedback.position_rad,
                     hold_target_rad);
  Serial.printf("GRIPPER_HOLD_ACTIVE contact_rad=%.6f target_rad=%.6f "
                "kp=%.2f watchdog_ms=%lu\n",
                contact_position_rad, hold_target_rad,
                GRIPPER_CONTACT_HOLD_KP,
                static_cast<unsigned long>(GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS));

  bool stop_requested = false;
  bool release_requested = false;
  bool hold_healthy = true;
  uint32_t last_heartbeat_ms = millis();
  while (hold_healthy && !stop_requested) {
    const HoldRequest request = read_hold_request();
    if (request == HoldRequest::STOP) {
      stop_requested = true;
      break;
    }
    if (request == HoldRequest::OPEN) {
      release_requested = true;
      break;
    }
    if (request == HoldRequest::HEARTBEAT) {
      last_heartbeat_ms = millis();
    }
    if (millis() - last_heartbeat_ms > GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS) {
      Serial.println("GRIPPER_HOLD_ABORT reason=heartbeat_timeout");
      hold_healthy = false;
      break;
    }

    MotorFeedback live{};
    const bool received = transmit_and_receive_feedback(
        hold_command, motor_id, live, "contact_hold");
    if (!received || !gripper_feedback_ok(live, branch_offset_rad) ||
        fabsf(live.velocity_rad_s) > 1.0F ||
        fabsf(live.position_rad - contact_position_rad) > 0.20F) {
      Serial.println("GRIPPER_HOLD_ABORT reason=feedback_limit");
      hold_healthy = false;
      break;
    }
    feedback = live;
    delay(GRIPPER_HOLD_PERIOD_MS);
  }

  if (release_requested && hold_healthy) {
    // Intentional place/release transition: remove preload at the measured
    // contact pose, then ramp directly open without an intermediate disable.
    const float open_target_rad =
        GRIPPER_OPEN_TARGET_RAD + branch_offset_rad;
    const float release_start_rad = feedback.position_rad;
    const twai_message_t unload_command = mit_frame(
        motor_id, release_start_rad, GRIPPER_CONTACT_HOLD_KP,
        GRIPPER_CONTACT_HOLD_KD);
    bool release_ok = transmit_and_receive_feedback(
        unload_command, motor_id, feedback, "contact_release_unload");
    delay(2 * GRIPPER_HOLD_PERIOD_MS);
    const float release_delta = open_target_rad - release_start_rad;
    uint16_t release_steps = static_cast<uint16_t>(
        ceilf(fabsf(release_delta) / GRIPPER_MOVE_STEP_RAD));
    if (release_steps < 1) release_steps = 1;
    if (release_steps > 300) release_steps = 300;
    for (uint16_t step = 1; step <= release_steps && release_ok; ++step) {
      const HoldRequest request = read_hold_request();
      if (request == HoldRequest::STOP) {
        stop_requested = true;
        release_ok = false;
        break;
      }
      if (request == HoldRequest::HEARTBEAT) {
        last_heartbeat_ms = millis();
      }
      if (millis() - last_heartbeat_ms > GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS) {
        Serial.println("GRIPPER_RELEASE_ABORT reason=heartbeat_timeout");
        release_ok = false;
        break;
      }
      const float command_position =
          release_start_rad + release_delta * static_cast<float>(step) /
                                  static_cast<float>(release_steps);
      const twai_message_t release_command = mit_frame(
          motor_id, command_position, GRIPPER_MOVE_KP, GRIPPER_MOVE_KD);
      MotorFeedback live{};
      const bool received = transmit_and_receive_feedback(
          release_command, motor_id, live, "contact_release_open");
      if (!received || !gripper_feedback_ok(live, branch_offset_rad)) {
        Serial.println("GRIPPER_RELEASE_ABORT reason=feedback_limit");
        release_ok = false;
        break;
      }
      feedback = live;
      delay(GRIPPER_MOVE_DELAY_MS);
    }

    send_disable(motor_id, "contact_release_disable_finally");
    delay(50);
    MotorFeedback final_status{};
    const bool final_feedback = transmit_and_receive_feedback(
        status_frame(motor_id), motor_id, final_status,
        "contact_release_final_status");
    const bool safely_disabled = final_feedback && final_status.error == 0;
    if (stop_requested) {
      send_gateway_event(0x12, safely_disabled ? 0x01 : 0x02,
                         final_status.error, safely_disabled ? 0x01 : 0x00,
                         final_feedback ? final_status.position_rad : 999.0F,
                         open_target_rad);
      return;
    }
    const bool open_reached =
        final_feedback &&
        fabsf(final_status.position_rad - open_target_rad) <= 0.10F;
    const bool passed = release_ok && safely_disabled && open_reached;
    send_gateway_event(0x10, passed ? 0x01 : 0x02, final_status.error,
                       safely_disabled ? 0x01 : 0x00,
                       final_feedback ? final_status.position_rad : 999.0F,
                       open_target_rad);
    return;
  }

  send_disable(motor_id, "contact_hold_disable_finally");
  delay(50);
  MotorFeedback final_status{};
  const bool final_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, final_status,
      "contact_hold_final_status");
  const bool safely_disabled = final_feedback && final_status.error == 0;
  const bool clean_stop = stop_requested && hold_healthy && safely_disabled;
  send_gateway_event(0x12, clean_stop ? 0x01 : 0x02, final_status.error,
                     safely_disabled ? 0x01 : 0x00,
                     final_feedback ? final_status.position_rad : 999.0F,
                     hold_target_rad);
}

static void run_gripper_move(uint16_t motor_id, float target_rad,
                             uint8_t event, const char *name) {
  Serial.printf("GRIPPER_BEGIN action=%s target_rad=%.6f\n", name, target_rad);
  if (motor_id != SAFE_TEST_MOTOR_ID || !initialize_can(1000)) {
    send_gateway_event(event, 0x02, 0xFF, 0, 0.0F, target_rad);
    return;
  }

  MotorFeedback initial{};
  const bool initial_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, initial, "gripper_preflight", 500);
  float branch_offset_rad = 0.0F;
  const bool branch_valid =
      initial_feedback &&
      select_gripper_branch(initial.position_rad, branch_offset_rad);
  if (branch_valid) {
    target_rad += branch_offset_rad;
    Serial.printf("GRIPPER_BRANCH initial_rad=%.6f offset_rad=%.6f "
                  "shifted_target_rad=%.6f\n",
                  initial.position_rad, branch_offset_rad, target_rad);
  }
  if (!initial_feedback || !branch_valid ||
      initial.error != 0 ||
      fabsf(initial.velocity_rad_s) > SAFE_MAX_INITIAL_SPEED_RAD_S ||
      initial.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      initial.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C) {
    Serial.println("GRIPPER_RESULT REJECT reason=preflight");
    send_disable(motor_id, "gripper_preflight_disable");
    send_gateway_event(event, 0x03, initial.error, 0, initial.position_rad,
                       target_rad);
    return;
  }

  MotorFeedback feedback{};
  const twai_message_t neutral =
      zero_torque_mit_frame(motor_id, initial.position_rad);
  if (!transmit_and_receive_feedback(neutral, motor_id, feedback,
                                     "gripper_preload") ||
      !transmit_and_receive_feedback(control_command_frame(motor_id, 0xFC),
                                     motor_id, feedback, "gripper_enable") ||
      feedback.error != 1) {
    send_disable(motor_id, "gripper_enable_failure_disable");
    send_gateway_event(event, 0x02, feedback.error, 0, initial.position_rad,
                       target_rad);
    return;
  }

  const float delta = target_rad - initial.position_rad;
  uint16_t steps =
      static_cast<uint16_t>(ceilf(fabsf(delta) / GRIPPER_MOVE_STEP_RAD));
  if (steps < 1) steps = 1;
  if (steps > 300) steps = 300;
  bool active_ok = true;
  bool stop_requested = false;
  MotorFeedback live = feedback;
  const uint32_t move_start = millis();
  for (uint16_t step = 1; step <= steps; ++step) {
    if (emergency_stop_requested()) {
      stop_requested = true;
      active_ok = false;
      break;
    }
    if (millis() - move_start > GRIPPER_MOVE_TIMEOUT_MS) {
      Serial.println("GRIPPER_ABORT reason=timeout");
      active_ok = false;
      break;
    }
    const float command_position =
        initial.position_rad + delta * static_cast<float>(step) /
                                   static_cast<float>(steps);
    const twai_message_t command = mit_frame(
        motor_id, command_position, GRIPPER_MOVE_KP, GRIPPER_MOVE_KD);
    bool received = transmit_and_receive_feedback(command, motor_id, live,
                                                  "gripper_ramp");
    if (!received) {
      // Long moves contain hundreds of one-shot frames. Retry the exact same
      // bounded setpoint once; never advance the target without feedback.
      delay(5);
      received = transmit_and_receive_feedback(command, motor_id, live,
                                               "gripper_ramp_retry");
    }
    if (!received || !gripper_feedback_ok(live, branch_offset_rad)) {
      Serial.println("GRIPPER_ABORT reason=feedback_limit");
      active_ok = false;
      break;
    }
    delay(GRIPPER_MOVE_DELAY_MS);
  }

  for (uint8_t hold = 0; hold < GRIPPER_HOLD_MAX_STEPS && active_ok; ++hold) {
    if (emergency_stop_requested()) {
      stop_requested = true;
      active_ok = false;
      break;
    }
    const twai_message_t command =
        mit_frame(motor_id, target_rad, GRIPPER_MOVE_KP, GRIPPER_MOVE_KD);
    bool received = transmit_and_receive_feedback(command, motor_id, live,
                                                  "gripper_hold");
    if (!received) {
      delay(5);
      received = transmit_and_receive_feedback(command, motor_id, live,
                                               "gripper_hold_retry");
    }
    if (!received || !gripper_feedback_ok(live, branch_offset_rad)) {
      active_ok = false;
      break;
    }
    if (hold + 1 >= GRIPPER_HOLD_MIN_STEPS &&
        fabsf(live.position_rad - target_rad) <= 0.06F &&
        fabsf(live.velocity_rad_s) <= SAFE_MAX_INITIAL_SPEED_RAD_S) {
      break;
    }
    delay(GRIPPER_MOVE_DELAY_MS);
  }

  const bool active_target_reached =
      active_ok && !stop_requested &&
      fabsf(live.position_rad - target_rad) <= 0.12F;
  if (active_target_reached) {
    run_gripper_active_session(motor_id, event, live, target_rad,
                               branch_offset_rad, false);
    return;
  }

  const bool disable_feedback =
      send_disable(motor_id, "gripper_disable_finally");
  // Let the linkage settle after releasing contact force. Reading after only
  // 20 ms captured spring-back velocity and misclassified a valid grasp.
  delay(150);
  MotorFeedback final_status{};
  const bool final_feedback = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, final_status, "gripper_final_status");
  const bool safely_disabled = final_feedback && final_status.error == 0;
  const bool target_reached =
      final_feedback && fabsf(final_status.position_rad - target_rad) <= 0.08F;
  // A real object normally prevents CLOSE from reaching the calibrated empty-
  // jaw endpoint. That is a successful grasp contact when the move made clear
  // progress toward CLOSE and the settled, disabled state is fault-free and
  // within the calibrated branch. Never use this classification for OPEN.
  const float commanded_delta = target_rad - initial.position_rad;
  const float actual_delta =
      final_feedback ? final_status.position_rad - initial.position_rad : 0.0F;
  const bool close_action = event == 0x11;
  const bool progressed_toward_target =
      fabsf(actual_delta) >= 0.05F && actual_delta * commanded_delta > 0.0F;
  const bool final_stationary =
      final_feedback &&
      fabsf(final_status.velocity_rad_s) <= SAFE_MAX_INITIAL_SPEED_RAD_S;
  const bool final_position_safe =
      final_feedback &&
      final_status.position_rad >=
          GRIPPER_OPEN_HARD_RAD + branch_offset_rad - 0.10F &&
      final_status.position_rad <=
          GRIPPER_CLOSED_HARD_RAD + branch_offset_rad + 0.10F;
  const bool final_temperature_safe =
      final_feedback &&
      final_status.mos_temperature_c < SAFE_MAX_TEMPERATURE_C &&
      final_status.rotor_temperature_c < SAFE_MAX_TEMPERATURE_C;
  const bool contact_detected =
      close_action && safely_disabled && final_stationary &&
      final_position_safe && final_temperature_safe &&
      progressed_toward_target && !target_reached && !stop_requested;
  const bool passed = active_ok && disable_feedback && safely_disabled &&
                      target_reached && !stop_requested;
  const uint8_t result_code = passed ? 0x01 : (contact_detected ? 0x04 : 0x02);
  if (contact_detected) {
    run_gripper_contact_hold(motor_id, event, final_status.position_rad,
                             target_rad, branch_offset_rad);
    return;
  }
  Serial.printf(
      "GRIPPER_RESULT action=%s passed=%u contact=%u final_rad=%.6f "
      "target_rad=%.6f progress_rad=%.6f verified_disabled=%u stop=%u\n",
      name, passed, contact_detected, final_status.position_rad, target_rad,
      actual_delta, safely_disabled, stop_requested);
  send_gateway_event(event, result_code, final_status.error,
                     safely_disabled ? 0x01 : 0x00,
                     final_feedback ? final_status.position_rad : 999.0F,
                     target_rad);
  if (stop_requested) {
    send_gateway_event(0x12, safely_disabled ? 0x01 : 0x02,
                       final_status.error, safely_disabled ? 0x01 : 0x00,
                       final_status.position_rad, final_status.position_rad);
  }
}

static void send_read_parameter(uint16_t motor_id, uint8_t rid) {
  twai_message_t frame{};
  frame.identifier = DM_PARAMETER_CAN_ID;
  frame.extd = 0;
  frame.rtr = 0;
  // A missing ACK must not cause automatic retries. Exactly one CAN frame is
  // emitted for each valid RS485 request.
  frame.ss = 1;
  frame.data_length_code = 8;
  frame.data[0] = static_cast<uint8_t>(motor_id & 0xFF);
  frame.data[1] = static_cast<uint8_t>((motor_id >> 8) & 0x07);
  frame.data[2] = DM_READ_OPCODE;
  frame.data[3] = rid;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;
  // Default first, then all other Classic CAN rates documented by the V1.2
  // manual. Every attempt is the same read-only frame and is single-shot.
  static constexpr uint32_t scan_kbps[] = {1000, 500, 250, 200, 125};
  for (uint32_t kbps : scan_kbps) {
    if (!initialize_can(kbps)) {
      Serial.printf("CAN_SCAN_RESULT bitrate_kbps=%lu init_failed\n",
                    static_cast<unsigned long>(kbps));
      continue;
    }

    esp_err_t result = twai_transmit(&frame, pdMS_TO_TICKS(100));
    Serial.printf(
        "DM_READ_TX bitrate_kbps=%lu motor_id=0x%03X rid=0x%02X "
        "result=%s data=",
        static_cast<unsigned long>(kbps), motor_id, rid,
        esp_err_to_name(result));
    print_bytes(frame.data, 8);
    Serial.println();
    print_can_status("queued");
    if (result != ESP_OK) continue;

    const uint32_t deadline = millis() + 100;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
      uint32_t alerts = 0;
      if (twai_read_alerts(&alerts, pdMS_TO_TICKS(5)) == ESP_OK && alerts != 0) {
        log_can_alerts(alerts);
        if (alerts & TWAI_ALERT_TX_SUCCESS) {
          Serial.printf("CAN_SCAN_RESULT bitrate_kbps=%lu ACK\n",
                        static_cast<unsigned long>(kbps));
          return;
        }
        if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) break;
      }
    }
    Serial.printf("CAN_SCAN_RESULT bitrate_kbps=%lu NO_ACK\n",
                  static_cast<unsigned long>(kbps));
  }

  // Restore the documented default rate for passive receive after the scan.
  initialize_can(1000);
  Serial.println("CAN_SCAN_COMPLETE no_ack_all_classic_rates");
}

static void send_status_request(uint16_t motor_id) {
  if (!initialize_can(1000)) {
    Serial.println("DM_STATUS_TX init_failed");
    return;
  }
  twai_message_t frame{};
  frame.identifier = DM_PARAMETER_CAN_ID;
  frame.extd = 0;
  frame.rtr = 0;
  frame.ss = 1;
  frame.data_length_code = 8;
  frame.data[0] = static_cast<uint8_t>(motor_id & 0xFF);
  frame.data[1] = static_cast<uint8_t>((motor_id >> 8) & 0x07);
  frame.data[2] = 0xCC;

  esp_err_t result = twai_transmit(&frame, pdMS_TO_TICKS(100));
  Serial.printf("DM_STATUS_TX motor_id=0x%03X result=%s data=", motor_id,
                esp_err_to_name(result));
  print_bytes(frame.data, 8);
  Serial.println();
  if (result != ESP_OK) return;

  const uint32_t deadline = millis() + 100;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(5)) == ESP_OK && alerts != 0) {
      log_can_alerts(alerts);
      if (alerts & TWAI_ALERT_TX_SUCCESS) {
        Serial.println("DM_STATUS_TX ACK");
        return;
      }
      if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) break;
    }
  }
  Serial.println("DM_STATUS_TX NO_ACK");
}

static void process_request() {
  const bool parameter_request =
      memcmp(request_buffer, REQUEST_HEADER, sizeof(REQUEST_HEADER)) == 0;
  const bool status_request =
      memcmp(request_buffer, STATUS_HEADER, sizeof(STATUS_HEADER)) == 0;
  const bool safe_enable_request =
      memcmp(request_buffer, SAFE_ENABLE_HEADER, sizeof(SAFE_ENABLE_HEADER)) ==
      0;
  const bool safe_disable_request =
      memcmp(request_buffer, SAFE_DISABLE_HEADER,
             sizeof(SAFE_DISABLE_HEADER)) == 0;
  const bool ident_request =
      memcmp(request_buffer, IDENT_HEADER, sizeof(IDENT_HEADER)) == 0;
  const bool tiny_move_request =
      memcmp(request_buffer, TINY_MOVE_HEADER, sizeof(TINY_MOVE_HEADER)) == 0;
  const bool gripper_open_request =
      memcmp(request_buffer, GRIPPER_OPEN_HEADER,
             sizeof(GRIPPER_OPEN_HEADER)) == 0;
  const bool gripper_close_request =
      memcmp(request_buffer, GRIPPER_CLOSE_HEADER,
             sizeof(GRIPPER_CLOSE_HEADER)) == 0;
  if (!parameter_request && !status_request && !safe_enable_request &&
      !safe_disable_request && !ident_request && !tiny_move_request &&
      !gripper_open_request && !gripper_close_request) {
    Serial.print("RS485_REJECT bad_header data=");
    print_bytes(request_buffer, REQUEST_SIZE);
    Serial.println();
    return;
  }
  if (xor_checksum(request_buffer, REQUEST_SIZE - 1) != request_buffer[7]) {
    Serial.print("RS485_REJECT bad_checksum data=");
    print_bytes(request_buffer, REQUEST_SIZE);
    Serial.println();
    return;
  }
  uint16_t motor_id = static_cast<uint16_t>(request_buffer[4]) |
                      (static_cast<uint16_t>(request_buffer[5] & 0x07) << 8);
  if (gripper_open_request) {
    if (motor_id != SAFE_TEST_MOTOR_ID ||
        request_buffer[6] != GRIPPER_OPEN_GUARD) {
      send_gateway_event(0x10, 0x03, 0xFF, 0, 0.0F,
                         GRIPPER_OPEN_TARGET_RAD);
      return;
    }
    run_gripper_move(motor_id, GRIPPER_OPEN_TARGET_RAD, 0x10, "OPEN");
  } else if (gripper_close_request) {
    if (motor_id != SAFE_TEST_MOTOR_ID ||
        request_buffer[6] != GRIPPER_CLOSE_GUARD) {
      send_gateway_event(0x11, 0x03, 0xFF, 0, 0.0F,
                         GRIPPER_CLOSE_TARGET_RAD);
      return;
    }
    run_gripper_move(motor_id, GRIPPER_CLOSE_TARGET_RAD, 0x11, "CLOSE");
  } else if (tiny_move_request) {
    if (motor_id != SAFE_TEST_MOTOR_ID ||
        request_buffer[6] != TINY_MOVE_GUARD) {
      Serial.println("RS485_REJECT tiny_move_guard_or_id");
      return;
    }
    run_tiny_move_test(motor_id);
  } else if (ident_request) {
    if (motor_id != SAFE_TEST_MOTOR_ID || request_buffer[6] != IDENT_GUARD) {
      Serial.println("RS485_REJECT ident_guard_or_id");
      return;
    }
    Serial.println(
        "FIRMWARE_ID guarded_gateway_build_3.7 persistent_dual_hold_settle");
    send_identity_response();
  } else if (safe_enable_request) {
    if (request_buffer[6] != SAFE_ENABLE_GUARD) {
      Serial.println("RS485_REJECT safe_enable_guard");
      return;
    }
    run_zero_torque_enable_test(motor_id);
  } else if (safe_disable_request) {
    if (motor_id != SAFE_TEST_MOTOR_ID ||
        request_buffer[6] != SAFE_DISABLE_GUARD) {
      Serial.println("RS485_REJECT safe_disable_guard_or_id");
      return;
    }
    if (!initialize_can(1000)) {
      Serial.println("SAFE_DISABLE_RESULT FAIL reason=can_init");
      return;
    }
    const bool disabled = send_disable(motor_id, "host_requested_disable");
    Serial.printf("SAFE_DISABLE_RESULT %s\n", disabled ? "PASS" : "FAIL");
    send_gateway_event(0x12, disabled ? 0x01 : 0x02, 0,
                       disabled ? 0x01 : 0x00, 0.0F, 0.0F);
  } else if (status_request) {
    if (request_buffer[6] != 0) {
      Serial.println("RS485_REJECT status_reserved_byte_nonzero");
      return;
    }
    send_status_request(motor_id);
  } else {
    uint8_t rid = request_buffer[6];
    // Only parameter reads are emitted. No generic CAN forwarding exists.
    send_read_parameter(motor_id, rid);
  }
}

static void poll_rs485_requests() {
  while (RS485.available()) {
    uint8_t value = static_cast<uint8_t>(RS485.read());
    if (request_count == 0 && value != REQUEST_HEADER[0]) continue;
    request_buffer[request_count++] = value;
    if (request_count == REQUEST_SIZE) {
      process_request();
      request_count = 0;
    }
  }
}

static void send_response_to_rs485(const twai_message_t &frame) {
  // Response: D5 4D 52 50 ID_LO ID_HI FLAGS DLC DATA[8] XOR = 17 bytes.
  uint8_t packet[17]{};
  packet[0] = 0xD5;
  packet[1] = 0x4D;
  packet[2] = 0x52;
  packet[3] = 0x50;
  packet[4] = static_cast<uint8_t>(frame.identifier & 0xFF);
  packet[5] = static_cast<uint8_t>((frame.identifier >> 8) & 0xFF);
  packet[6] = (frame.extd ? 0x80 : 0) | (frame.rtr ? 0x40 : 0);
  packet[7] = frame.data_length_code;
  for (uint8_t i = 0; i < frame.data_length_code && i < 8; ++i) {
    packet[8 + i] = frame.data[i];
  }
  packet[16] = xor_checksum(packet, 16);
  RS485.write(packet, sizeof(packet));
  RS485.flush();
}

static void poll_can() {
  twai_message_t frame{};
  while (twai_receive(&frame, 0) == ESP_OK) {
    Serial.printf("CAN_RX id=0x%03lX ext=%u rtr=%u dlc=%u data=",
                  static_cast<unsigned long>(frame.identifier), frame.extd,
                  frame.rtr, frame.data_length_code);
    print_bytes(frame.data, frame.data_length_code);
    Serial.println();
    send_response_to_rs485(frame);
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);
  Serial.println();
#ifdef GRIPPER_LEGACY_PROFILE
  Serial.println("T-CAN485 DM4310 guarded gateway build 1.1 rollback");
#else
  Serial.println("T-CAN485 DM4310 guarded gateway build 3.7 dual-hold-settle");
#endif
  initialize_can(1000);
  initialize_rs485();
  Serial.println("ALLOWLIST: CAN 0x7FF parameter-read opcode 0x33 only");
  Serial.println("ALLOWLIST: CAN 0x7FF status-refresh opcode 0xCC only");
  Serial.println("ACTIVE_ALLOWLIST: motor 1 zero-torque enable 250ms then disable");
  Serial.println("ACTIVE_ALLOWLIST: motor 1 disable safety command");
  Serial.println("ACTIVE_ALLOWLIST: motor 1 +0.02 rad return test");
  Serial.println("ACTIVE_ALLOWLIST: calibrated gripper OPEN/CLOSE/STOP");
#ifdef GRIPPER_LEGACY_PROFILE
  Serial.println("GRIPPER_PROFILE: step=0.04rad feedback_each_step delay=15ms hold<=20");
#else
  Serial.println("GRIPPER_PROFILE: step=0.30rad feedback_each_step delay=10ms hold<=10");
#endif
  Serial.println("RUNTIME: RS485/TCP60000 only; USB is optional diagnostics");
  Serial.println("ACTIVE_DENY: no position delta, clear-fault, zeroing, parameter write");
  Serial.println("TX_POLICY: single-shot, no automatic retry");
  Serial.println("BITRATE_SCAN: 1000,500,250,200,125 kbit/s; stop on ACK");
}

void loop() {
  poll_rs485_requests();
  poll_can_alerts();
  poll_can();
  delay(1);
}

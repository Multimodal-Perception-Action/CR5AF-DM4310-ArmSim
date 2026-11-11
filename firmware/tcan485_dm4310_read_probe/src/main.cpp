#include <Arduino.h>
#include "driver/twai.h"
#include <math.h>
#include "gripper_session_logic.h"

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
// Replacement-gripper feedback ceiling.  After a commanded CLOSE is safely
// disabled, the mechanism passively settles at 5.6273 rad (20/20 identical
// read-only samples, motor error 0).  Accept that observed passive pose plus a
// small margin without increasing the commanded close target below.
static constexpr float GRIPPER_CLOSED_HARD_RAD = 5.647718013F;
static constexpr float GRIPPER_OPEN_TARGET_RAD = -0.097920209F;
// Retain the existing command target; feedback tolerance is not permission
// to increase the commanded closing travel.
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
#else
static constexpr float GRIPPER_MOVE_STEP_RAD = 0.30F;
static constexpr uint32_t GRIPPER_MOVE_DELAY_MS = 10;
#endif
static constexpr float GRIPPER_MOVE_KP = 10.0F;
static constexpr float GRIPPER_MOVE_KD = 0.2F;
// Conservative contact hold: 0.04 rad preload at kp=4 is approximately
// 0.16 N*m motor-side proportional demand before reducer multiplication.
static constexpr float GRIPPER_CONTACT_HOLD_KP = 4.0F;
static constexpr float GRIPPER_CONTACT_HOLD_KD = 0.2F;
static constexpr uint32_t GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS = 750;
static constexpr uint32_t GRIPPER_HOLD_PERIOD_MS = 20;
// A true runaway guard, not a normal-motion threshold.  The DM protocol is
// configured for VMAX=30 rad/s and the 3x gripper ramp can legitimately pass
// the former 9.6 rad/s ceiling.  Retain 20% headroom below the configured
// protocol maximum for a genuine hard-speed shutdown.
static constexpr float GRIPPER_MAX_FEEDBACK_SPEED_RAD_S = 24.0F;
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
  send_gateway_event(0x01, 0x01, 0x00, 0x10, 4.6F, 0.0F);
#endif
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

// Runtime MIT setpoints are idempotent: retry the exact same frame without
// advancing the trajectory.  A single lost reply is transport jitter, while
// three consecutive misses (about 0.4 s total) are treated as a real CAN loss.
static bool transmit_and_receive_feedback_guarded(
    const twai_message_t &frame, uint16_t motor_id, MotorFeedback &feedback,
    const char *phase, uint32_t feedback_timeout_ms = 120) {
  static constexpr uint8_t MAX_ATTEMPTS = 3;
  for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
    if (transmit_and_receive_feedback(frame, motor_id, feedback, phase,
                                      feedback_timeout_ms)) {
      return true;
    }
    if (attempt < MAX_ATTEMPTS) {
      Serial.printf("SAFE_FEEDBACK_RETRY phase=%s attempt=%u\n", phase,
                    static_cast<unsigned>(attempt + 1));
      delay(5);
    }
  }
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

// Encoded into the high nibble of a failed STOP event's flags byte.  Bit 0
// remains the independently verified-disabled flag.
static constexpr uint8_t GRIPPER_ABORT_NONE = 0;
static constexpr uint8_t GRIPPER_ABORT_HEARTBEAT = 1;
static constexpr uint8_t GRIPPER_ABORT_CAN_FEEDBACK = 2;
static constexpr uint8_t GRIPPER_ABORT_MOTOR_STATE = 3;
static constexpr uint8_t GRIPPER_ABORT_HARD_POSITION = 4;
static constexpr uint8_t GRIPPER_ABORT_HARD_SPEED = 5;
static constexpr uint8_t GRIPPER_ABORT_TEMPERATURE = 6;

static uint8_t gripper_feedback_abort_reason(const MotorFeedback &feedback,
                                             float branch_offset_rad) {
  if (feedback.error != 1) return GRIPPER_ABORT_MOTOR_STATE;
  if (feedback.position_rad <
          GRIPPER_OPEN_HARD_RAD + branch_offset_rad - 0.10F ||
      feedback.position_rad >
          GRIPPER_CLOSED_HARD_RAD + branch_offset_rad + 0.10F) {
    return GRIPPER_ABORT_HARD_POSITION;
  }
  if (fabsf(feedback.velocity_rad_s) > GRIPPER_MAX_FEEDBACK_SPEED_RAD_S) {
    return GRIPPER_ABORT_HARD_SPEED;
  }
  if (feedback.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      feedback.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C) {
    return GRIPPER_ABORT_TEMPERATURE;
  }
  return GRIPPER_ABORT_NONE;
}

static bool gripper_feedback_ok(const MotorFeedback &feedback,
                                float branch_offset_rad) {
  return gripper_feedback_abort_reason(feedback, branch_offset_rad) ==
         GRIPPER_ABORT_NONE;
}

// Persistent teleoperation session. Both OPEN and CLOSE/CONTACT are active
// low-gain hold states. Opposite commands transition directly while enabled;
// only STOP, watchdog loss, feedback fault, or a safety limit disables.
// Drain all available requests (bounded work per pass), not just one packet.
// The parser resynchronizes after a damaged byte and STOP cannot be overwritten.
static void service_gripper_inbox(GripperInbox &inbox) {
  for (unsigned i = 0; i < 128 && RS485.available(); ++i) {
    inbox.feed(static_cast<uint8_t>(RS485.read()), millis());
  }
}

static bool gripper_cancelled(GripperInbox &inbox, uint8_t &abort_reason) {
  service_gripper_inbox(inbox);
  if (inbox.stopped) return true;
  if (inbox.expired(millis(), GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS)) {
    abort_reason = GRIPPER_ABORT_HEARTBEAT;
    return true;
  }
  return false;
}

// During CAN retries continue servicing STOP/heartbeat. Never advance a
// setpoint on missing feedback; stop retransmitting an old target on reversal.
static bool session_exchange(const twai_message_t &frame, uint16_t motor_id,
                             MotorFeedback &feedback, GripperInbox &inbox,
                             uint8_t &abort_reason, const char *phase,
                             uint8_t expected_goal = 0,
                             bool *superseded = nullptr) {
  const uint32_t started_ms = millis();
  bool missing_feedback = false;
  bool read_fresh_status = false;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (gripper_cancelled(inbox, abort_reason)) return false;
    if (expected_goal && inbox.goal != expected_goal && superseded) {
      if (!missing_feedback) {
        *superseded = true;
        return false;
      }
      // A reply was lost while the operator changed direction. Cancel the old
      // setpoint, but use the remaining exchange budget to read fresh status.
      // Neither a single missed reply nor a new target is a confirmed bus loss.
      // Once selected, status refresh remains read-only even if the goal flips
      // back again, until measured position/state are available for replanning.
      read_fresh_status = true;
    }
    const bool received = transmit_and_receive_feedback(
        read_fresh_status ? status_frame(motor_id) : frame,
        motor_id, feedback, phase);
    if (gripper_cancelled(inbox, abort_reason)) return false;
    if (received) {
      if (read_fresh_status && superseded) *superseded = true;
      if (missing_feedback) {
        // Diagnostic only, never a motion/hold acknowledgement. Floats carry
        // elapsed milliseconds and total attempts, not motor coordinates.
        send_gateway_event(0x20, 1, 0, 0,
                           static_cast<float>(millis()-started_ms), attempt+1);
      }
      return true;
    }
    missing_feedback = true;
    delay(5);
  }
  send_gateway_event(0x20, 2, 0, 0,
                     static_cast<float>(millis()-started_ms), 3.0F);
  abort_reason = GRIPPER_ABORT_CAN_FEEDBACK;
  return false;
}

static void finish_gripper_session(uint16_t motor_id, GripperInbox &inbox,
                                   uint8_t abort_reason, float target) {
  send_disable(motor_id, "session_disable_finally");
  delay(50);
  MotorFeedback final_status{};
  const bool received = transmit_and_receive_feedback_guarded(
      status_frame(motor_id), motor_id, final_status, "session_final_status");
  const bool disabled = received && final_status.error == 0;
  const bool clean = inbox.stopped && abort_reason == GRIPPER_ABORT_NONE &&
                     disabled;
  if (!clean && abort_reason == GRIPPER_ABORT_NONE) {
    abort_reason = received ? GRIPPER_ABORT_MOTOR_STATE
                            : GRIPPER_ABORT_CAN_FEEDBACK;
  }
  Serial.printf("SESSION_END reason=%u heartbeat_age_ms=%lu heartbeats=%lu "
                "invalid_windows=%lu stop=%u\n", abort_reason,
                static_cast<unsigned long>(millis()-inbox.last_alive_ms),
                static_cast<unsigned long>(inbox.heartbeat_count),
                static_cast<unsigned long>(inbox.invalid_windows),
                inbox.stopped);
  send_gateway_event(
      0x12, clean ? 0x01 : 0x02, final_status.error,
      (disabled ? 0x01 : 0x00) | (clean ? 0 : (abort_reason << 4)),
      received ? final_status.position_rad : 999.0F, target);
}

static void run_gripper_active_session(uint16_t motor_id,
                                      MotorFeedback feedback,
                                      float branch_offset_rad,
                                      GripperInbox &inbox) {
  GripperMotion motion(
      inbox.goal, feedback.position_rad,
      GRIPPER_OPEN_TARGET_RAD+branch_offset_rad,
      GRIPPER_CLOSE_TARGET_RAD+branch_offset_rad, GRIPPER_MOVE_STEP_RAD);
  uint8_t abort_reason = GRIPPER_ABORT_NONE;
  uint32_t move_start_ms = millis();
  int announced_phase = -1;
  while (true) {
    if (gripper_cancelled(inbox, abort_reason)) break;
    abort_reason = gripper_feedback_abort_reason(feedback, branch_offset_rad);
    if (abort_reason != GRIPPER_ABORT_NONE) break;
    if (inbox.goal != motion.goal) {
      motion.reverse(inbox.goal, feedback.position_rad);
      move_start_ms = millis();
      announced_phase = -1;
    }
    if (!motion.holding() &&
        millis()-move_start_ms > GRIPPER_MOVE_TIMEOUT_MS) {
      abort_reason = 7;  // bounded move/settle timeout, never a successful grip
      break;
    }
    const float target = motion.next(feedback.position_rad);
    const auto command_phase = motion.phase;
    const float kp = motion.holding() ? GRIPPER_CONTACT_HOLD_KP :
                     (motion.goal == 0x11 ? 6.0F : GRIPPER_MOVE_KP);
    MotorFeedback live{};
    bool superseded = false;
    if (!session_exchange(
            mit_frame(motor_id, target, kp, GRIPPER_MOVE_KD),
            motor_id, live, inbox, abort_reason, "gripper_session",
            motion.goal, &superseded)) {
      if (superseded) continue;
      break;
    }
    abort_reason = gripper_feedback_abort_reason(live, branch_offset_rad);
    if (abort_reason != GRIPPER_ABORT_NONE) break;
    feedback = live;
    if (superseded) {
      // A recovered status is a measurement, not acknowledgement that the
      // old setpoint completed. Replan even if the trigger flipped back while
      // reading status; keep the deadline when the final goal is unchanged.
      if (inbox.goal != motion.goal) move_start_ms = millis();
      motion.reverse(inbox.goal, feedback.position_rad);
      announced_phase = -1;
      continue;
    }
    if (inbox.goal != motion.goal) continue;
    motion.observe(live.position_rad, live.velocity_rad_s);
    // Publish only after the selected low-gain hold command has been sent and
    // acknowledged, not when a ramp ends or before contact force is reduced.
    if (motion.holding() && motion.phase == command_phase &&
        announced_phase != static_cast<int>(motion.phase)) {
      send_gateway_event(motion.goal,
                         motion.phase == GripperMotion::CONTACT ? 4 : 5,
                         0, 0x02, live.position_rad, motion.command);
      announced_phase = static_cast<int>(motion.phase);
    }
    delay(motion.holding() ? GRIPPER_HOLD_PERIOD_MS : GRIPPER_MOVE_DELAY_MS);
  }
  finish_gripper_session(motor_id, inbox, abort_reason, motion.command);
}

static void run_gripper_move(uint16_t motor_id, float target_rad,
                             uint8_t event, const char *name) {
  Serial.printf("GRIPPER_BEGIN action=%s target_rad=%.6f\n", name, target_rad);
  if (motor_id != SAFE_TEST_MOTOR_ID || !initialize_can(1000)) {
    send_gateway_event(event, 0x02, 0xFF, 0, 0.0F, target_rad);
    return;
  }
  GripperInbox inbox(event, millis());
  MotorFeedback initial{};
  const bool received = transmit_and_receive_feedback(
      status_frame(motor_id), motor_id, initial, "gripper_preflight", 500);
  float branch_offset_rad = 0.0F;
  const bool branch_valid = received &&
      select_gripper_branch(initial.position_rad, branch_offset_rad);
  if (branch_valid) target_rad += branch_offset_rad;
  uint8_t abort_reason = GRIPPER_ABORT_NONE;
  if (gripper_cancelled(inbox, abort_reason)) {
    finish_gripper_session(motor_id, inbox, abort_reason, target_rad);
    return;
  }
  if (!received || !branch_valid || initial.error != 0 ||
      fabsf(initial.velocity_rad_s) > SAFE_MAX_INITIAL_SPEED_RAD_S ||
      initial.mos_temperature_c >= SAFE_MAX_TEMPERATURE_C ||
      initial.rotor_temperature_c >= SAFE_MAX_TEMPERATURE_C) {
    send_gateway_event(event, 0x03, initial.error, 0,
                       received ? initial.position_rad : 999.0F, target_rad);
    return;  // preflight has not enabled or sent a position command
  }
  MotorFeedback feedback{};
  if (!session_exchange(
          zero_torque_mit_frame(motor_id, initial.position_rad),
          motor_id, feedback, inbox, abort_reason, "gripper_preload") ||
      !session_exchange(control_command_frame(motor_id, 0xFC),
                        motor_id, feedback, inbox, abort_reason,
                        "gripper_enable")) {
    finish_gripper_session(motor_id, inbox, abort_reason, target_rad);
    return;
  }
  abort_reason = gripper_feedback_abort_reason(feedback, branch_offset_rad);
  if (abort_reason != GRIPPER_ABORT_NONE) {
    finish_gripper_session(motor_id, inbox, abort_reason, target_rad);
    return;
  }
  run_gripper_active_session(motor_id, feedback, branch_offset_rad, inbox);
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
        "FIRMWARE_ID guarded_gateway_build_4.6 reversal_feedback_recovery");
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
  Serial.println("T-CAN485 DM4310 guarded gateway build 4.6 reversal-feedback-recovery");
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
  Serial.println("TX_POLICY: CAN single-shot; active session <=3 guarded software attempts");
  Serial.println("BITRATE_SCAN: 1000,500,250,200,125 kbit/s; stop on ACK");
}

void loop() {
  poll_rs485_requests();
  poll_can_alerts();
  poll_can();
  delay(1);
}

// This fixture includes production functions verbatim; only hardware I/O and
// time are replaced. It is not an independent reimplementation of the loop.
#include "gripper_session_logic.h"
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#define REQUIRE(x) do { if (!(x)) throw std::runtime_error(#x); } while (0)

struct MotorFeedback {
  bool valid = true;
  uint8_t error = 0;
  float position_rad = 0, velocity_rad_s = 0, torque_nm = 0;
  uint8_t mos_temperature_c = 30, rotor_temperature_c = 28;
};
struct twai_message_t { int kind; float position, kp; uint8_t operation; };
struct FakeSerial {
  template<class... Args> void printf(const char *, Args...) {}
  void println(const char *) {}
} Serial;
struct FakeRS485 {
  std::deque<uint8_t> bytes;
  int available() const { return static_cast<int>(bytes.size()); }
  uint8_t read() { auto value = bytes.front(); bytes.pop_front(); return value; }
} RS485;
struct Event { uint8_t event, result, error, flags; float position, target; };
struct Sent { float position, kp; uint32_t at; };
static uint32_t now_ms, next_heartbeat;
static bool auto_heartbeat, enabled;
static float position, obstacle;
static int enable_count, disable_count, remaining_losses;
static bool runtime_loss_seen;
static unsigned recovery_status_reads;
static unsigned recovery_status_losses;
static int forced_recovery_error;
static std::vector<Event> events;
static std::vector<Event> diagnostics;
static std::vector<Sent> sent;
static std::function<void(const Event &)> event_hook;
static std::function<void(const twai_message_t &)> frame_hook;
static std::function<void()> time_hook;

static void queue(uint8_t op) {
  uint8_t guard = op == 0x44 ? 0x5A : op == 0x48 ? 0xC3 :
                  op == 0x4F ? 0xC1 : 0xC2;
  uint8_t packet[] = {0xD5, 0x4D, op, 0x51, 1, 0, guard, 0};
  for (unsigned i = 0; i < 7; ++i) packet[7] ^= packet[i];
  for (auto value : packet) RS485.bytes.push_back(value);
}
static uint32_t millis() { return now_ms; }
static void advance(uint32_t value) {
  now_ms += value;
  REQUIRE(now_ms < 20000);  // Detect an accidental infinite production loop.
  if (auto_heartbeat) {
    while (now_ms >= next_heartbeat) {
      queue(0x48);
      next_heartbeat += 100;
    }
  }
  if (time_hook) time_hook();
}
static void delay(uint32_t value) { advance(value); }
static bool initialize_can(uint32_t) { return true; }
static twai_message_t mit_frame(uint16_t, float target, float kp, float) {
  return {0, target, kp, 0};
}
static twai_message_t zero_torque_mit_frame(uint16_t id, float target) {
  return mit_frame(id, target, 0, 0);
}
static twai_message_t status_frame(uint16_t) { return {1, 0, 0, 0}; }
static twai_message_t control_command_frame(uint16_t, uint8_t command) {
  return {2, 0, 0, command};
}
static MotorFeedback feedback() {
  MotorFeedback result;
  result.error = enabled ? 1 : 0;
  result.position_rad = position;
  return result;
}
static bool transmit_and_receive_feedback(const twai_message_t &frame,
    uint16_t, MotorFeedback &result, const char *phase,
    uint32_t feedback_timeout_ms = 120) {
  if (frame.kind == 1 && enabled && runtime_loss_seen) {
    ++recovery_status_reads;
    if (recovery_status_losses > 0) {
      --recovery_status_losses;
      advance(feedback_timeout_ms);
      return false;
    }
  }
  if (frame.kind == 2 && frame.operation == 0xFC) {
    enabled = true;
    ++enable_count;
  }
  if (frame.kind == 0 && frame.kp > 0) {
    REQUIRE(enabled);
    sent.push_back({frame.position, frame.kp, now_ms});
    position = fminf(frame.position, obstacle);
  }
  if (frame_hook) frame_hook(frame);
  if (std::string(phase) == "gripper_session" && remaining_losses > 0) {
    runtime_loss_seen = true;
    --remaining_losses;
    advance(feedback_timeout_ms);
    return false;
  }
  advance(1);
  result = feedback();
  if (frame.kind == 1 && enabled && runtime_loss_seen && forced_recovery_error >= 0)
    result.error = static_cast<uint8_t>(forced_recovery_error);
  return true;
}
static bool transmit_and_receive_feedback_guarded(const twai_message_t &frame,
    uint16_t id, MotorFeedback &result, const char *phase,
    uint32_t timeout_ms = 120) {
  return transmit_and_receive_feedback(frame, id, result, phase, timeout_ms);
}
static bool send_disable(uint16_t, const char *) {
  ++disable_count;
  enabled = false;
  return true;
}
static void send_gateway_event(uint8_t event, uint8_t result, uint8_t error,
    uint8_t flags, float measured, float target) {
  Event record{event, result, error, flags, measured, target};
  if (event == 0x20) diagnostics.push_back(record);
  else events.push_back(record);
  if (event_hook) event_hook(record);
}

// Generated from main.cpp by the test runner, with production bodies unedited.
#include "gripper_runtime_extracted.h"

static void reset() {
  now_ms = 0;
  next_heartbeat = 100;
  auto_heartbeat = true;
  enabled = false;
  position = 2.0F;
  obstacle = 100;
  enable_count = disable_count = remaining_losses = 0;
  runtime_loss_seen = false;
  recovery_status_reads = 0;
  recovery_status_losses = 0;
  forced_recovery_error = -1;
  events.clear(); diagnostics.clear(); sent.clear(); RS485.bytes.clear();
  event_hook = nullptr; frame_hook = nullptr; time_hook = nullptr;
}
static void run(uint8_t event) {
  run_gripper_move(1, event == 0x11 ? GRIPPER_CLOSE_TARGET_RAD :
                                        GRIPPER_OPEN_TARGET_RAD,
                   event, event == 0x11 ? "CLOSE" : "OPEN");
}
static void require_clean_stop() {
  REQUIRE(!events.empty());
  REQUIRE(events.back().event == 0x12);
  REQUIRE(events.back().result == 1);
  REQUIRE(events.back().flags == 1);
  REQUIRE(disable_count == 1);
  REQUIRE(!enabled);
}

static void initial_contact_no_disable_reenable() {
  reset(); obstacle = 2.0F;
  event_hook = [](const Event &event) {
    if (event.event == 0x11 && event.result == 4) {
      REQUIRE(disable_count == 0);
      REQUIRE(enable_count == 1);
      REQUIRE(!sent.empty() && sent.back().kp == GRIPPER_CONTACT_HOLD_KP);
      REQUIRE(fabsf(event.target - 2.04F) < 0.0001F);
      queue(0x44);
    }
  };
  run(0x11);
  REQUIRE(events.front().result == 4);
  require_clean_stop();
}

static void contact_partial_release_then_regrip() {
  reset(); obstacle = 2.0F;
  unsigned contacts = 0;
  bool reversed = false;
  event_hook = [&](const Event &event) {
    if (event.event == 0x11 && event.result == 4) {
      ++contacts;
      REQUIRE(disable_count == 0);
      queue(contacts == 1 ? 0x4F : 0x44);
    }
  };
  frame_hook = [&](const twai_message_t &frame) {
    if (contacts == 1 && !reversed && frame.kind == 0 &&
        frame.kp > 0 && frame.position < 1.8F) {
      reversed = true;
      queue(0x43);
    }
  };
  run(0x11);
  REQUIRE(contacts == 2);
  REQUIRE(reversed);
  REQUIRE(enable_count == 1);
  for (const auto &event : events) REQUIRE(event.event != 0x10);
  require_clean_stop();
}

static void stop_during_first_ramp() {
  reset();
  bool requested = false;
  time_hook = [&]() {
    if (!requested && now_ms >= 35) { requested = true; queue(0x44); }
  };
  run(0x11);
  REQUIRE(requested);
  REQUIRE(events.size() == 1);
  require_clean_stop();
}

static void initial_ramp_reversal_is_not_a_stop() {
  reset();
  bool requested = false;
  time_hook = [&]() {
    if (!requested && now_ms >= 35) { requested = true; queue(0x4F); }
  };
  event_hook = [](const Event &event) {
    if (event.event == 0x10 && event.result == 5) queue(0x44);
  };
  run(0x11);
  REQUIRE(events.front().event == 0x10);
  REQUIRE(enable_count == 1);
  require_clean_stop();
}

static void missing_heartbeat_stops_hold() {
  reset(); auto_heartbeat = false;
  run(0x10);
  REQUIRE(events.front().event == 0x10);
  REQUIRE(events.back().event == 0x12);
  REQUIRE(events.back().result == 2);
  REQUIRE((events.back().flags >> 4) == GRIPPER_ABORT_HEARTBEAT);
  REQUIRE(disable_count == 1 && !enabled);
}

static void retry_keeps_heartbeats_alive_and_same_setpoint() {
  reset(); remaining_losses = 2;
  event_hook = [](const Event &event) {
    if (event.event == 0x10 && event.result == 5) queue(0x44);
  };
  run(0x10);
  REQUIRE(sent.size() > 3);
  REQUIRE(sent[0].position == sent[1].position);
  REQUIRE(sent[1].position == sent[2].position);
  require_clean_stop();
}

static void stop_interrupts_retry_before_resending() {
  reset(); remaining_losses = 3;
  bool requested = false;
  time_hook = [&]() {
    if (!requested && now_ms >= 50) { requested = true; queue(0x44); }
  };
  run(0x10);
  REQUIRE(sent.size() == 1);
  require_clean_stop();
}

static void real_can_loss_stops_after_three_attempts() {
  reset(); remaining_losses = 3;
  run(0x10);
  REQUIRE(sent.size() == 3);
  REQUIRE(events.back().result == 2);
  REQUIRE((events.back().flags >> 4) == GRIPPER_ABORT_CAN_FEEDBACK);
  REQUIRE(disable_count == 1 && !enabled);
}

static void first_ramp_lost_reply_and_reversal_recovers_fresh_status() {
  reset(); remaining_losses = 1;
  bool requested = false;
  frame_hook = [&](const twai_message_t &frame) {
    if (!requested && frame.kind == 0 && frame.kp > 0) {
      requested = true;
      queue(0x4F);
    }
  };
  event_hook = [](const Event &event) {
    if (event.event == 0x10 && event.result == 5) queue(0x44);
  };
  run(0x11);
  REQUIRE(requested);
  REQUIRE(runtime_loss_seen);
  REQUIRE(events.front().event == 0x10 && events.front().result == 5);
  REQUIRE(recovery_status_reads >= 1);
  REQUIRE(diagnostics.size() == 1 && diagnostics[0].result == 1);
  REQUIRE(diagnostics[0].target == 2.0F);
  REQUIRE(enable_count == 1);
  require_clean_stop();
}

static void contact_hold_lost_reply_and_release_recovers_fresh_status() {
  reset(); obstacle = 2.0F;
  bool contact = false, requested = false;
  event_hook = [&](const Event &event) {
    if (event.event == 0x11 && event.result == 4) {
      contact = true;
      remaining_losses = 1;
    }
    if (event.event == 0x10 && event.result == 5) queue(0x44);
  };
  frame_hook = [&](const twai_message_t &frame) {
    if (contact && !requested && frame.kind == 0 && frame.kp > 0) {
      requested = true;
      queue(0x4F);
    }
  };
  run(0x11);
  REQUIRE(contact && requested && runtime_loss_seen);
  REQUIRE(events.size() == 3);
  REQUIRE(events[0].event == 0x11 && events[0].result == 4);
  REQUIRE(events[1].event == 0x10 && events[1].result == 5);
  REQUIRE(recovery_status_reads >= 1);
  REQUIRE(enable_count == 1);
  require_clean_stop();
}

static void reversal_status_recovery_still_stops_after_three_misses() {
  reset(); remaining_losses = 1; recovery_status_losses = 10;
  bool requested = false;
  frame_hook = [&](const twai_message_t &frame) {
    if (!requested && frame.kind == 0 && frame.kp > 0) {
      requested = true;
      queue(0x4F);
    }
  };
  run(0x11);
  REQUIRE(sent.size() == 1);
  REQUIRE(recovery_status_reads == 2);
  REQUIRE(diagnostics.size() == 1 && diagnostics[0].result == 2);
  REQUIRE(diagnostics[0].target == 3.0F);
  REQUIRE(events.back().result == 2);
  REQUIRE((events.back().flags >> 4) == GRIPPER_ABORT_CAN_FEEDBACK);
  REQUIRE(disable_count == 1 && !enabled);
}

static void stop_interrupts_reversal_status_recovery() {
  reset(); remaining_losses = 1; recovery_status_losses = 10;
  bool reversed = false, stopped = false;
  frame_hook = [&](const twai_message_t &frame) {
    if (!reversed && frame.kind == 0 && frame.kp > 0) {
      reversed = true;
      queue(0x4F);
    }
  };
  time_hook = [&]() {
    if (!stopped && now_ms >= 150) { stopped = true; queue(0x44); }
  };
  run(0x11);
  REQUIRE(sent.size() == 1);
  REQUIRE(recovery_status_reads == 1);
  REQUIRE(stopped);
  require_clean_stop();
}

static void goal_flips_back_during_status_replans_from_fresh_pose() {
  reset(); remaining_losses = 1;
  bool reversed = false, flipped_back = false, checked_replanned_frame = false;
  frame_hook = [&](const twai_message_t &frame) {
    if (!reversed && frame.kind == 0 && frame.kp > 0) {
      reversed = true;
      queue(0x4F);
    } else if (!flipped_back && frame.kind == 1 && enabled && runtime_loss_seen) {
      flipped_back = true;
      position = 3.5F;  // Fresh measured pose differs from the last known pose.
      queue(0x43);
    } else if (flipped_back && !checked_replanned_frame &&
               frame.kind == 0 && frame.kp > 0) {
      checked_replanned_frame = true;
      // With a stale planner command the feedback clamp would command 3.2,
      // moving backward despite the latest CLOSE request.
      REQUIRE(frame.position >= 3.5F);
      REQUIRE(frame.position <= 3.80001F);
    }
  };
  event_hook = [](const Event &event) {
    if (event.event == 0x11 && event.result == 5) queue(0x44);
  };
  run(0x11);
  REQUIRE(flipped_back && checked_replanned_frame);
  REQUIRE(events.front().event == 0x11 && events.front().result == 5);
  require_clean_stop();
}

static void recovery_status_motor_fault_cannot_resume_motion() {
  reset(); remaining_losses = 1; forced_recovery_error = 9;
  bool reversed = false;
  frame_hook = [&](const twai_message_t &frame) {
    if (!reversed && frame.kind == 0 && frame.kp > 0) {
      reversed = true;
      queue(0x4F);
    }
  };
  run(0x11);
  REQUIRE(recovery_status_reads == 1);
  REQUIRE(sent.size() == 1);
  REQUIRE(events.back().result == 2);
  REQUIRE((events.back().flags >> 4) == GRIPPER_ABORT_MOTOR_STATE);
  REQUIRE(disable_count == 1 && !enabled);
}

static void late_contact_while_endpoint_holding() {
  reset();
  event_hook = [](const Event &event) {
    if (event.event == 0x11 && event.result == 5) {
      position = obstacle = 4.0F;
    } else if (event.event == 0x11 && event.result == 4) {
      REQUIRE(disable_count == 0);
      REQUIRE(fabsf(event.target - 4.04F) < 0.0001F);
      queue(0x44);
    }
  };
  run(0x11);
  REQUIRE(events.size() == 3);
  REQUIRE(events[0].result == 5 && events[1].result == 4);
  require_clean_stop();
}

int main() {
  struct Case { const char *name; void (*run)(); };
  const Case cases[] = {
    {"initial_contact_no_disable_reenable", initial_contact_no_disable_reenable},
    {"contact_partial_release_then_regrip", contact_partial_release_then_regrip},
    {"stop_during_first_ramp", stop_during_first_ramp},
    {"initial_ramp_reversal_is_not_a_stop", initial_ramp_reversal_is_not_a_stop},
    {"missing_heartbeat_stops_hold", missing_heartbeat_stops_hold},
    {"retry_keeps_heartbeats_alive_and_same_setpoint", retry_keeps_heartbeats_alive_and_same_setpoint},
    {"stop_interrupts_retry_before_resending", stop_interrupts_retry_before_resending},
    {"real_can_loss_stops_after_three_attempts", real_can_loss_stops_after_three_attempts},
    {"first_ramp_lost_reply_and_reversal_recovers_fresh_status", first_ramp_lost_reply_and_reversal_recovers_fresh_status},
    {"contact_hold_lost_reply_and_release_recovers_fresh_status", contact_hold_lost_reply_and_release_recovers_fresh_status},
    {"reversal_status_recovery_still_stops_after_three_misses", reversal_status_recovery_still_stops_after_three_misses},
    {"stop_interrupts_reversal_status_recovery", stop_interrupts_reversal_status_recovery},
    {"goal_flips_back_during_status_replans_from_fresh_pose", goal_flips_back_during_status_replans_from_fresh_pose},
    {"recovery_status_motor_fault_cannot_resume_motion", recovery_status_motor_fault_cannot_resume_motion},
    {"late_contact_while_endpoint_holding", late_contact_while_endpoint_holding},
  };
  unsigned failed = 0;
  for (const auto &test : cases) {
    try { test.run(); std::cout << "PASS " << test.name << '\n'; }
    catch (const std::exception &error) {
      ++failed;
      std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
    }
  }
  return failed ? 1 : 0;
}

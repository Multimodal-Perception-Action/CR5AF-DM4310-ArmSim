#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Byte-stream parser: a damaged/truncated frame must not misalign all future
// heartbeats. STOP latches for this session; OPEN/CLOSE retain the latest goal.
struct GripperInbox {
  uint8_t bytes[8]{};
  size_t count = 0;
  uint8_t goal;
  bool stopped = false;
  uint32_t last_alive_ms;
  uint32_t heartbeat_count = 0;
  uint32_t invalid_windows = 0;

  GripperInbox(uint8_t initial_goal, uint32_t now)
      : goal(initial_goal), last_alive_ms(now) {}

  void feed(uint8_t value, uint32_t now) {
    bytes[count++] = value;
    if (count != 8) return;
    uint8_t checksum = 0;
    for (size_t i = 0; i < 7; ++i) checksum ^= bytes[i];
    const uint8_t op = bytes[2];
    const uint8_t guard = bytes[6];
    const bool recognized =
        (op == 0x44 && guard == 0x5A) ||
        (op == 0x48 && guard == 0xC3) ||
        (op == 0x4F && guard == 0xC1) ||
        (op == 0x43 && guard == 0xC2);
    if (bytes[0] == 0xD5 && bytes[1] == 0x4D && bytes[3] == 0x51 &&
        bytes[4] == 1 && bytes[5] == 0 && checksum == bytes[7] &&
        recognized) {
      count = 0;
      if (op == 0x44) stopped = true;
      if (stopped) return;
      last_alive_ms = now;
      if (op == 0x48) ++heartbeat_count;
      if (op == 0x4F) goal = 0x10;
      if (op == 0x43) goal = 0x11;
      return;
    }
    ++invalid_windows;
    for (size_t i = 1; i < 8; ++i) bytes[i-1] = bytes[i];
    count = 7;
  }

  bool expired(uint32_t now, uint32_t timeout_ms) const {
    return static_cast<uint32_t>(now-last_alive_ms) > timeout_ms;
  }
};

// The same planner is used for the first action, reversals and contact holds.
// No enable/disable decisions belong to this class.
struct GripperMotion {
  enum Phase { RAMP, SETTLE, ENDPOINT, CONTACT };
  uint8_t goal;
  float open_target, close_target, target, command, previous_position;
  float step;
  Phase phase = RAMP;
  unsigned stall_samples = 0;
  unsigned reached_samples = 0;

  GripperMotion(uint8_t initial_goal, float position, float opened,
                float closed, float maximum_step)
      : goal(initial_goal), open_target(opened), close_target(closed),
        target(initial_goal == 0x10 ? opened : closed), command(position),
        previous_position(position), step(maximum_step) {}

  void reverse(uint8_t new_goal, float position) {
    goal = new_goal;
    target = goal == 0x10 ? open_target : close_target;
    command = previous_position = position;
    stall_samples = reached_samples = 0;
    phase = RAMP;
  }

  float next(float actual) {
    if (phase == CONTACT || phase == ENDPOINT) return command;
    float delta = target-command;
    float next_command = command + fmaxf(-step, fminf(step, delta));
    // Never accumulate a large position demand against an object. In both
    // directions the setpoint remains at most one ramp step ahead of feedback.
    next_command = fmaxf(actual-step, fminf(actual+step, next_command));
    // A passive pose may be outside the command endpoints but still inside
    // the validated feedback envelope. Approach from that pose gradually.
    command = fmaxf(fminf(open_target, actual),
                   fminf(fmaxf(close_target, actual), next_command));
    if (fabsf(target-command) < 0.0001F) phase = SETTLE;
    return command;
  }

  void observe(float position, float speed) {
    if (phase == CONTACT) return;
    const bool stationary = fabsf(speed) <= 0.50F &&
                            fabsf(position-previous_position) <= 0.02F;
    const bool blocked_close = goal == 0x11 && stationary &&
                               command-position >= 0.16F;
    stall_samples = blocked_close ? stall_samples+1 : 0;
    previous_position = position;
    if (stall_samples >= 2) {
      command = fminf(close_target, position+0.04F);
      phase = CONTACT;
      return;
    }
    if (phase == ENDPOINT) return;
    const bool reached = fabsf(target-position) <= 0.08F && stationary;
    reached_samples = reached ? reached_samples+1 : 0;
    if (reached_samples >= 2) {
      command = target;
      phase = ENDPOINT;
    }
  }

  bool holding() const { return phase == ENDPOINT || phase == CONTACT; }
};

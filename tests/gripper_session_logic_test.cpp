// Executable, hardware-free regression tests against the production header.
#include "gripper_session_logic.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#define REQUIRE(x) do { if (!(x)) throw std::runtime_error(#x); } while (0)

static std::vector<uint8_t> packet(uint8_t op, uint8_t guard,
                                   uint8_t id = 1) {
  std::vector<uint8_t> bytes{0xD5, 0x4D, op, 0x51, id, 0, guard, 0};
  for (unsigned i = 0; i < 7; ++i) bytes[7] ^= bytes[i];
  return bytes;
}

static void feed(GripperInbox &box, const std::vector<uint8_t> &bytes,
                 uint32_t now) {
  for (auto value : bytes) box.feed(value, now);
}

static void fragmented_request() {
  GripperInbox box(0x10, 0);
  auto close = packet(0x43, 0xC2);
  for (unsigned i = 0; i < 7; ++i) box.feed(close[i], 10);
  REQUIRE(box.goal == 0x10);
  box.feed(close[7], 20);
  REQUIRE(box.goal == 0x11);
  REQUIRE(box.last_alive_ms == 20);
}

static void resync_after_noise_and_bad_checksum() {
  GripperInbox box(0x10, 0);
  feed(box, {0xD5, 0x4D, 0xAA, 0xBB, 0xCC}, 10);
  auto bad = packet(0x43, 0xC2);
  bad.back() ^= 0xFF;
  feed(box, bad, 20);
  REQUIRE(box.goal == 0x10);
  feed(box, packet(0x48, 0xC3), 30);
  feed(box, packet(0x43, 0xC2), 40);
  REQUIRE(box.heartbeat_count == 1);
  REQUIRE(box.goal == 0x11);
  REQUIRE(box.invalid_windows > 0);
}

static void latest_goal_and_stop_priority() {
  GripperInbox box(0x11, 0);
  feed(box, packet(0x4F, 0xC1), 10);
  feed(box, packet(0x43, 0xC2), 20);
  REQUIRE(box.goal == 0x11);
  feed(box, packet(0x44, 0x5A), 30);
  feed(box, packet(0x4F, 0xC1), 40);
  feed(box, packet(0x48, 0xC3), 50);
  REQUIRE(box.stopped);
  REQUIRE(box.goal == 0x11);
  REQUIRE(box.last_alive_ms == 20);
}

static void wrong_id_and_guard_do_not_renew_watchdog() {
  GripperInbox box(0x10, 0);
  feed(box, packet(0x43, 0xC2, 2), 100);
  feed(box, packet(0x43, 0xC1), 200);
  REQUIRE(box.goal == 0x10);
  REQUIRE(box.last_alive_ms == 0);
  REQUIRE(box.expired(751, 750));
  feed(box, packet(0x48, 0xC3), 800);
  REQUIRE(!box.expired(801, 750));
}

static void watchdog_wraparound() {
  GripperInbox box(0x10, 0xFFFFFFF0U);
  REQUIRE(!box.expired(10, 750));
  REQUIRE(box.expired(1000, 750));
  feed(box, packet(0x48, 0xC3), 1001);
  REQUIRE(!box.expired(1100, 750));
}

static void initial_close_against_already_held_object() {
  GripperMotion motion(0x11, 2.0F, -0.09792F, 5.30181F, 0.30F);
  for (unsigned i = 0; i < 2; ++i) {
    REQUIRE(motion.next(2.0F) <= 2.30001F);
    motion.observe(2.0F, 0.0F);
  }
  REQUIRE(motion.phase == GripperMotion::CONTACT);
  REQUIRE(fabsf(motion.command - 2.04F) < 0.00001F);
  for (unsigned i = 0; i < 100; ++i) {
    REQUIRE(fabsf(motion.next(2.0F) - 2.04F) < 0.00001F);
    motion.observe(2.0F, 0.0F);
  }
}

static void contact_release_reclose_before_open_endpoint() {
  GripperMotion motion(0x11, 2.0F, -0.09792F, 5.30181F, 0.30F);
  for (unsigned i = 0; i < 2; ++i) {
    motion.next(2.0F);
    motion.observe(2.0F, 0.0F);
  }
  REQUIRE(motion.phase == GripperMotion::CONTACT);
  motion.reverse(0x10, 2.0F);
  float position = motion.next(2.0F);
  REQUIRE(position < 2.0F);
  REQUIRE(position > -0.09792F);
  motion.observe(position, -1.0F);
  motion.reverse(0x11, position);
  REQUIRE(motion.phase == GripperMotion::RAMP);
  for (unsigned i = 0; i < 10 && !motion.holding(); ++i) {
    float command = motion.next(position);
    position = fminf(command, 2.0F);  // Object is still present.
    motion.observe(position, 0.0F);
  }
  REQUIRE(motion.phase == GripperMotion::CONTACT);
  REQUIRE(fabsf(motion.command - 2.04F) < 0.00001F);
}

static void late_contact_at_last_ramp_sample() {
  GripperMotion motion(0x11, 4.9F, -0.09792F, 5.30181F, 0.30F);
  float position = 4.9F;
  for (unsigned i = 0; i < 10 && !motion.holding(); ++i) {
    position = fminf(motion.next(position), 5.12F);
    motion.observe(position, 0.0F);
  }
  REQUIRE(motion.phase == GripperMotion::CONTACT);
  REQUIRE(fabsf(motion.command - 5.16F) < 0.00001F);
}

static void open_is_never_object_contact() {
  GripperMotion motion(0x10, 2.0F, -0.09792F, 5.30181F, 0.30F);
  for (unsigned i = 0; i < 100; ++i) {
    REQUIRE(motion.next(2.0F) >= 1.69999F);
    motion.observe(2.0F, 0.0F);
  }
  REQUIRE(motion.phase != GripperMotion::CONTACT);
  REQUIRE(!motion.holding());
}

static void full_open_and_close_require_stable_feedback() {
  GripperMotion motion(0x10, 2.0F, -0.09792F, 5.30181F, 0.30F);
  float position = 2.0F;
  for (unsigned direction = 0; direction < 2; ++direction) {
    if (direction) motion.reverse(0x11, position);
    for (unsigned i = 0; i < 100 && !motion.holding(); ++i) {
      float previous = position;
      position = motion.next(position);
      REQUIRE(fabsf(position - previous) <= 0.30001F);
      motion.observe(position, 0.0F);
    }
    REQUIRE(motion.phase == GripperMotion::ENDPOINT);
    REQUIRE(fabsf(position - motion.target) <= 0.08001F);
  }
}

static void moving_feedback_cannot_confirm_contact() {
  GripperMotion motion(0x11, 2.0F, -0.09792F, 5.30181F, 0.30F);
  for (unsigned i = 0; i < 5; ++i) {
    motion.next(2.0F);
    motion.observe(2.0F, 2.0F);
  }
  REQUIRE(!motion.holding());
}

static void outside_endpoint_initial_pose_slews_toward_target() {
  for (uint8_t goal : {0x10, 0x11}) {
    GripperMotion motion(goal, 5.627718F, -0.09792F, 5.30181F, 0.30F);
    float command = motion.next(5.627718F);
    REQUIRE(fabsf(command - 5.627718F) <= 0.30001F);
    REQUIRE(command < 5.627718F);
  }
}

int main() {
  struct Case { const char *name; void (*run)(); };
  const Case cases[] = {
      {"fragmented_request", fragmented_request},
      {"resync_after_noise_and_bad_checksum", resync_after_noise_and_bad_checksum},
      {"latest_goal_and_stop_priority", latest_goal_and_stop_priority},
      {"wrong_id_and_guard_do_not_renew_watchdog", wrong_id_and_guard_do_not_renew_watchdog},
      {"watchdog_wraparound", watchdog_wraparound},
      {"initial_close_against_already_held_object", initial_close_against_already_held_object},
      {"contact_release_reclose_before_open_endpoint", contact_release_reclose_before_open_endpoint},
      {"late_contact_at_last_ramp_sample", late_contact_at_last_ramp_sample},
      {"open_is_never_object_contact", open_is_never_object_contact},
      {"full_open_and_close_require_stable_feedback", full_open_and_close_require_stable_feedback},
      {"moving_feedback_cannot_confirm_contact", moving_feedback_cannot_confirm_contact},
      {"outside_endpoint_initial_pose_slews_toward_target", outside_endpoint_initial_pose_slews_toward_target},
  };
  unsigned failed = 0;
  for (const auto &test : cases) {
    try { test.run(); std::cout << "PASS " << test.name << '\n'; }
    catch (const std::exception &e) {
      ++failed;
      std::cerr << "FAIL " << test.name << ": " << e.what() << '\n';
    }
  }
  return failed ? 1 : 0;
}

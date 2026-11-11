import sys
import unittest
import io
import json
import tempfile
import threading
import struct
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gripper import (  # noqa: E402
    decode_gateway_event,
    gateway_action_passed,
    normalize_gripper_position,
)
import gripper as g


def event_packet(event: int, result: int, motor_error: int = 0,
                 flags: int = 1) -> bytes:
    packet = bytearray(17)
    packet[:4] = b"\xD5\x4D\x47\x50"
    packet[4:8] = bytes((event, result, motor_error, flags))
    return bytes(packet)


def diagnostic_packet(result=1, elapsed_ms=125.5, attempts=2):
    packet = bytearray(event_packet(0x20, result, flags=0))
    packet[8:16] = struct.pack('<ff', elapsed_ms, attempts)
    packet[-1] = g.xor_checksum(packet[:-1])
    return bytes(packet)


class GripperContactTests(unittest.TestCase):
    def test_fault_free_close_contact_is_success(self):
        event = decode_gateway_event(event_packet(0x11, 0x04))
        self.assertTrue(event["contact_detected"])
        self.assertTrue(gateway_action_passed("close", event))

    def test_watchdog_contact_hold_is_success_without_disable(self):
        event = decode_gateway_event(event_packet(0x11, 0x04, flags=0x02))
        self.assertTrue(event["holding_active"])
        self.assertFalse(event["verified_disabled"])
        self.assertTrue(gateway_action_passed("close", event))

    def test_open_endpoint_hold_is_success_without_disable(self):
        event = decode_gateway_event(event_packet(0x10, 0x05, flags=0x02))
        self.assertTrue(event["holding_active"])
        self.assertFalse(event["verified_disabled"])
        self.assertTrue(gateway_action_passed("open", event))

    def test_contact_code_is_not_accepted_for_open(self):
        event = decode_gateway_event(event_packet(0x10, 0x04))
        self.assertFalse(gateway_action_passed("open", event))

    def test_motor_fault_is_never_accepted_as_contact(self):
        event = decode_gateway_event(event_packet(0x11, 0x04, motor_error=1))
        self.assertFalse(gateway_action_passed("close", event))

    def test_missing_verified_disable_is_rejected(self):
        event = decode_gateway_event(event_packet(0x11, 0x04, flags=0))
        self.assertFalse(gateway_action_passed("close", event))

    def test_power_cycle_adjacent_branch_normalizes_to_calibration(self):
        normalized, offset = normalize_gripper_position(
            -1.0900663767452503, -0.1479202089568297, 5.211811232257789
        )
        self.assertAlmostEqual(offset, -2.0 * 3.141592653589793)
        self.assertAlmostEqual(normalized, 5.193118930434336, places=6)

    def test_abort_reason_is_decoded_from_high_flags_nibble(self):
        event = decode_gateway_event(event_packet(0x12, 0x02, flags=0x51))
        self.assertTrue(event["verified_disabled"])
        self.assertEqual(event["abort_reason_code"], 5)
        self.assertEqual(event["abort_reason"], "hard_speed_limit")

    def test_identity_capability_flag_is_not_an_abort_reason(self):
        event = decode_gateway_event(event_packet(0x01, 0x01, flags=0x10))
        self.assertEqual(event["abort_reason_code"], 0)
        self.assertEqual(event["abort_reason"], "none")

    def test_initial_open_can_failure_reports_abort_reason(self):
        event = decode_gateway_event(event_packet(0x10, 0x02, flags=0x21))
        self.assertEqual(event["abort_reason_code"], 2)
        self.assertEqual(event["abort_reason"], "can_feedback_loss")

    def test_action_timeout_reason_is_decoded(self):
        event = decode_gateway_event(event_packet(0x12, 0x02, flags=0x71))
        self.assertEqual(event["abort_reason"], "action_timeout")

    def test_can_recovery_diagnostic_is_not_a_pose(self):
        event = decode_gateway_event(diagnostic_packet())
        self.assertEqual(event['diagnostic'], 'can_exchange')
        self.assertTrue(event['recovered'])
        self.assertEqual(event['attempt_count'], 2)
        self.assertEqual(event['elapsed_ms'], 125.5)
        self.assertNotIn('position_rad', event)
        self.assertNotIn('target_rad', event)

    def test_can_exhausted_diagnostic_is_not_an_action_result(self):
        event = decode_gateway_event(diagnostic_packet(2, 370.0, 3))
        self.assertFalse(event['recovered'])
        self.assertEqual(event['attempt_count'], 3)
        self.assertEqual(event['elapsed_ms'], 370.0)
        self.assertFalse(gateway_action_passed('close', event))


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class SessionSocket:
    """Deterministic network substitute; never opens a hardware connection."""
    def __init__(self, clock, stop, *, contact=False, interrupt=False,
                 acknowledge_stop=True, abort=False, cancel_identity=False,
                 startup_delay=0.0, diagnostics=False):
        self.clock, self.stop = clock, stop
        self.contact = contact
        self.interrupt = interrupt
        self.acknowledge_stop = acknowledge_stop
        self.abort = abort
        self.cancel_identity = cancel_identity
        self.startup_delay = startup_delay
        self.diagnostics = diagnostics
        self.queue = []
        self.sent = []
        self.closed = 0
        self.action_at = None
        self.hold_sent = False
        self.stop_requested_at = None

    @staticmethod
    def packet(event, result, flags):
        packet = bytearray(event_packet(event, result, flags=flags))
        packet[-1] = g.xor_checksum(packet[:-1])
        return bytes(packet)

    def settimeout(self, _):
        pass

    def sendall(self, data):
        self.sent.append((self.clock.now, data))
        if data == g.PACKETS['identity']:
            self.queue.append(self.packet(1, 1, 0x10))
            if self.cancel_identity:
                self.stop.set()
        elif data == g.PACKETS['close']:
            self.action_at = self.clock.now
            if self.diagnostics:
                self.queue.append(diagnostic_packet())
        elif data == g.PACKETS['stop'] and self.acknowledge_stop:
            if self.diagnostics:
                self.queue.append(diagnostic_packet())
            # A queued hold must not be mistaken for STOP confirmation.
            self.queue.append(self.packet(0x11, 4 if self.contact else 5, 2))
            self.queue.append(self.packet(0x12, 1, 1))

    def recv(self, _):
        if self.queue:
            return self.queue.pop(0)
        self.clock.now += .05
        if self.action_at is not None:
            elapsed = self.clock.now-self.action_at
            if not self.hold_sent and elapsed >= self.startup_delay:
                self.hold_sent = True
                return self.packet(0x11, 4 if self.contact else 5, 2)
            if elapsed >= 1.0 and self.stop_requested_at is None:
                self.stop_requested_at = self.clock.now
                if self.abort:
                    return self.packet(0x12, 2, 0x11)
                if self.interrupt:
                    raise KeyboardInterrupt
                self.stop.set()
        raise g.socket.timeout()

    def close(self):
        self.closed += 1


class GripperCliLifecycleTests(unittest.TestCase):
    def run_session(self, *, hold_seconds=None, **kwargs):
        clock = FakeClock()
        stop = threading.Event()
        sock = SessionSocket(clock, stop, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(g, 'ROOT', Path(directory)), \
                 patch.object(g.socket, 'create_connection', return_value=sock), \
                 patch.object(g.time, 'monotonic', clock.monotonic), \
                 patch.object(g.sys, 'stdout', io.StringIO()), \
                 patch.object(g.sys, 'stderr', io.StringIO()):
                code = g.run_action('offline-test', 'close', 3.0,
                                    hold_seconds=hold_seconds, stop_event=stop)
            paths = list((Path(directory)/'logs').glob('*.json'))
            report = json.loads(paths[0].read_text(encoding='utf-8'))
        self.assertEqual(sock.closed, 1)
        return code, report, sock

    def test_contact_and_endpoint_hold_wait_for_operator_stop(self):
        for contact in (False, True):
            with self.subTest(contact=contact):
                code, report, sock = self.run_session(contact=contact)
                self.assertEqual(code, 0)
                self.assertTrue(report['disabled_verified'])
                self.assertEqual(report['completion_state'], 'stopped_disabled')
                self.assertEqual(report['stop_reason'], 'operator_stop')
                self.assertGreaterEqual(report['heartbeat_count'], 4)
                stops = [t for t, p in sock.sent if p == g.PACKETS['stop']]
                self.assertEqual(len(stops), 1)
                self.assertGreaterEqual(stops[0], sock.stop_requested_at)

    def test_explicit_timed_hold_sends_and_verifies_stop(self):
        code, report, sock = self.run_session(hold_seconds=.45)
        self.assertEqual(code, 0)
        self.assertEqual(report['stop_reason'], 'explicit_hold_duration_elapsed')
        self.assertTrue(report['stop_result']['verified_disabled'])
        self.assertIsNone(sock.stop_requested_at)

    def test_keyboard_interrupt_stops_and_verifies_disable(self):
        code, report, _ = self.run_session(contact=True, interrupt=True)
        self.assertEqual(code, 0)
        self.assertEqual(report['stop_reason'], 'keyboard_interrupt')
        self.assertTrue(report['disabled_verified'])

    def test_missing_stop_confirmation_is_not_success(self):
        code, report, _ = self.run_session(acknowledge_stop=False)
        self.assertEqual(code, 1)
        self.assertFalse(report['passed'])
        self.assertFalse(report['disabled_verified'])
        self.assertIsNone(report['holding_active'])
        self.assertIn('stop_error', report)

    def test_unsolicited_watchdog_is_failure_even_after_contact(self):
        code, report, _ = self.run_session(contact=True, abort=True)
        self.assertEqual(code, 1)
        self.assertFalse(report['passed'])
        self.assertIn('heartbeat_timeout', report['error'])
        self.assertTrue(report['disabled_verified'])

    def test_cancellation_during_identity_never_sends_motion(self):
        code, report, sock = self.run_session(cancel_identity=True)
        self.assertEqual(code, 1)
        self.assertTrue(report['cancelled_before_action'])
        self.assertEqual([p for _, p in sock.sent], [g.PACKETS['identity']])

    def test_heartbeats_cover_startup_before_first_hold_event(self):
        code, report, sock = self.run_session(startup_delay=.65)
        self.assertEqual(code, 0)
        before_hold = [t for t, p in sock.sent
                       if p == g.PACKETS['hold_heartbeat'] and t < .65]
        self.assertGreaterEqual(len(before_hold), 3)

    def test_can_diagnostics_do_not_interrupt_hold_or_replace_stop(self):
        code, report, sock = self.run_session(contact=True, diagnostics=True)
        self.assertEqual(code, 0)
        diagnostics = [event for event in report['events']
                       if event.get('diagnostic') == 'can_exchange']
        self.assertEqual(len(diagnostics), 2)
        self.assertEqual(report['stop_reason'], 'operator_stop')
        self.assertEqual(report['stop_result']['event'], g.EVENTS['stop'])
        stops = [t for t, p in sock.sent if p == g.PACKETS['stop']]
        self.assertEqual(len(stops), 1)
        self.assertGreaterEqual(stops[0], sock.stop_requested_at)


if __name__ == "__main__":
    unittest.main()

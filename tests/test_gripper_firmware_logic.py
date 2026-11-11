"""Compile and execute the actual portable C++ firmware state machine.

Uses an existing native C++ compiler, or existing Ubuntu_24.04 WSL g++ on
Windows. No toolchain is installed and no robot/socket/serial is accessed.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tests/gripper_session_logic_test.cpp'
INCLUDE = ROOT / 'firmware/tcan485_dm4310_read_probe/src'


class FirmwareLogicTests(unittest.TestCase):
    def compile_and_run(self, source, expected_cases, runtime=False):
        compiler = shutil.which('g++') or shutil.which('clang++')
        with tempfile.TemporaryDirectory(prefix='gripper_cpp_') as folder:
            folder = Path(folder)
            if runtime:
                self.extract_runtime(folder / 'gripper_runtime_extracted.h')
            output = Path(folder) / ('gripper_logic.exe' if os.name == 'nt'
                                      else 'gripper_logic')
            if compiler:
                command = [compiler, '-std=c++11', '-Wall', '-Wextra',
                           '-Werror', '-I', str(INCLUDE), '-I', str(folder), str(source),
                           '-o', str(output)]
                run = [str(output)]
            elif os.name == 'nt' and shutil.which('wsl.exe'):
                prefix = ['wsl.exe', '-d', 'Ubuntu_24.04', '--exec']
                probe = subprocess.run(prefix + ['test', '-x', '/usr/bin/g++'],
                                       capture_output=True, timeout=60)
                if probe.returncode:
                    self.skipTest('No installed native/WSL C++ compiler')

                def linux_path(path):
                    result = subprocess.run(prefix + ['wslpath', '-a',
                                                       str(path)],
                                            capture_output=True, text=True,
                                            check=True, timeout=10)
                    return result.stdout.strip()

                command = prefix + ['/usr/bin/g++', '-std=c++11', '-Wall',
                                    '-Wextra', '-Werror', '-I', linux_path(INCLUDE),
                                    '-I', linux_path(folder),
                                    linux_path(source), '-o', linux_path(output)]
                run = prefix + [linux_path(output)]
            else:
                self.skipTest('No installed native/WSL C++ compiler')
            built = subprocess.run(command, capture_output=True, text=True,
                                   timeout=60)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            completed = subprocess.run(run, capture_output=True, text=True,
                                       timeout=30)
            self.assertEqual(completed.returncode, 0,
                             completed.stdout + completed.stderr)
            self.assertEqual(completed.stdout.count('PASS '), expected_cases,
                             completed.stdout)
            print(completed.stdout, end='')

    def extract_runtime(self, destination):
        text = (INCLUDE / 'main.cpp').read_text(encoding='utf-8')
        names = [
            'SAFE_TEST_MOTOR_ID', 'SAFE_MAX_INITIAL_SPEED_RAD_S',
            'SAFE_MAX_TEMPERATURE_C', 'GRIPPER_OPEN_HARD_RAD',
            'GRIPPER_CLOSED_HARD_RAD', 'GRIPPER_OPEN_TARGET_RAD',
            'GRIPPER_CLOSE_TARGET_RAD', 'GRIPPER_REVOLUTION_RAD',
            'GRIPPER_MOVE_STEP_RAD', 'GRIPPER_MOVE_KP', 'GRIPPER_MOVE_KD',
            'GRIPPER_CONTACT_HOLD_KP', 'GRIPPER_HOLD_HEARTBEAT_TIMEOUT_MS',
            'GRIPPER_HOLD_PERIOD_MS', 'GRIPPER_MOVE_DELAY_MS',
            'GRIPPER_MAX_FEEDBACK_SPEED_RAD_S', 'GRIPPER_MOVE_TIMEOUT_MS',
            'GRIPPER_ABORT_NONE', 'GRIPPER_ABORT_HEARTBEAT',
            'GRIPPER_ABORT_CAN_FEEDBACK', 'GRIPPER_ABORT_MOTOR_STATE',
            'GRIPPER_ABORT_HARD_POSITION', 'GRIPPER_ABORT_HARD_SPEED',
            'GRIPPER_ABORT_TEMPERATURE',
        ]
        declarations = []
        for name in names:
            matches = re.findall(r'static constexpr [^;\n]*\b' + name +
                                 r'\b[^;]*;', text)
            self.assertTrue(matches, f'Production constant missing: {name}')
            # The final declaration is the current non-legacy profile.
            declarations.append(matches[-1])
        branch = text[text.index('static bool select_gripper_branch('):
                      text.index('// Encoded into the high nibble')]
        safety = text[text.index('static uint8_t gripper_feedback_abort_reason('):
                      text.index('static bool gripper_feedback_ok(')]
        runtime = text[text.index('static void service_gripper_inbox('):
                       text.index('static void send_read_parameter(')]
        destination.write_text('\n'.join(declarations) + '\n' + branch +
                               safety + runtime, encoding='utf-8')

    def test_production_cpp_state_machine(self):
        self.compile_and_run(SOURCE, 12)

    def test_production_runtime_loop(self):
        self.compile_and_run(ROOT / 'tests/gripper_session_runtime_test.cpp',
                             15, runtime=True)


if __name__ == '__main__':
    unittest.main()

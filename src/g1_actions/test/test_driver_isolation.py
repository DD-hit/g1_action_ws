"""Checks driver origin without starting any process or ROS node."""
from pathlib import Path
import tempfile
import unittest
from g1_actions.driver_runtime import driver_runtime_match


class DriverIsolationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.prefix = self.root / 'install/brsd_upbody_control'
        self.exe = self.prefix / 'lib/brsd_upbody_control/app_run_component'
        self.library = self.prefix / 'lib/libbrsd_upbody_control.so'
        self.exe.parent.mkdir(parents=True)
        self.exe.touch()
        self.library.touch()
        self.proc = self.root / 'proc/42'
        self.proc.mkdir(parents=True)
        (self.proc / 'exe').symlink_to(self.exe)
        self.map_library(self.library)

    def tearDown(self):
        self.temp.cleanup()

    def map_library(self, path):
        (self.proc / 'maps').write_text('1-2 r-xp 0000 00:00 1 ' + str(path) + '\n')

    def test_accepts_owned_executable_and_library(self):
        self.assertTrue(driver_runtime_match(self.proc, self.prefix))

    def test_rejects_old_executable_even_with_owned_library(self):
        old = self.root / 'old_app'
        old.touch()
        (self.proc / 'exe').unlink()
        (self.proc / 'exe').symlink_to(old)
        self.assertFalse(driver_runtime_match(self.proc, self.prefix))

    def test_rejects_old_library_even_with_owned_executable(self):
        old = self.root / 'old/libbrsd_upbody_control.so'
        old.parent.mkdir()
        old.touch()
        self.map_library(old)
        self.assertFalse(driver_runtime_match(self.proc, self.prefix))

    def test_requires_restart_if_mapped_library_was_replaced(self):
        self.map_library(str(self.library) + ' (deleted)')
        self.assertFalse(driver_runtime_match(self.proc, self.prefix))


if __name__ == '__main__':
    unittest.main()

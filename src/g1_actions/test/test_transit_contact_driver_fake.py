"""Replay the recorded down-button collision with an isolated fake SDK.

Run explicitly with ELE_TEST_DRIVER_INTEGRATION=1. This test never selects
real hardware; the existing runner enforces ROS domain 186 and fake_mode.
"""

import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(
    os.environ.get("ELE_TEST_DRIVER_INTEGRATION") == "1",
    "Requires built C++ driver; enable ELE_TEST_DRIVER_INTEGRATION=1",
)
class TransitContactReplayTests(unittest.TestCase):
    def replay(self, direction):
        env = {k: v for k, v in os.environ.items() if not k.startswith("ELE_TEST_")}
        env.update(
            ELE_TEST_ARM="right",
            ELE_TEST_DIRECTION=direction,
            ELE_TEST_TARGET_POINT=json.dumps(
                [0.4200504472498612, -0.043533724482143384, 0.1174095341439857]
            ),
            ELE_TEST_TARGET_NORMAL=json.dumps(
                [-0.9979611495758302, -0.055947714460051416, -0.03071477142980323]
            ),
            ELE_TEST_INITIAL_ARM_JOINTS=json.dumps(
                [0.2447059, -0.2759248, 0.1006675, 0.7844993,
                 0.0144889, 0.0072864, -0.0027444]
            ),
        )
        with tempfile.TemporaryDirectory(prefix="transit_contact_replay_") as output:
            env["ELE_TEST_OUTPUT"] = output
            run = subprocess.run(
                [sys.executable, str(ROOT / "test/test_two_stage_driver_fake.py")],
                env=env, cwd=ROOT, capture_output=True, text=True, timeout=120,
            )
            self.assertEqual(run.returncode, 0, (run.stdout + run.stderr)[-6000:])
            suffix = "_right_down" if direction == "down" else "_right"
            report = json.loads(
                (Path(output) / f"retrace_driver_fake{suffix}.json").read_text()
            )
            log = (Path(output) / f"retrace_driver_fake{suffix}.log").read_text()
        self.assertEqual(report["result"], "passed")
        self.assertFalse(report["hardware_commands_sent"])
        self.assertTrue(report["authority_released"])
        self.assertTrue(report["withdrawal_reuses_recorded_trajectory_without_replanning"])
        goals = re.findall(r"\[ElevatorJointGoal\].*positions=\[([^]]+)\]", log)
        self.assertEqual(len(goals), 1)
        self.assertIn("deterministic_ik=true contact_prechecked=true", log)
        precheck = log.index("reason=complete LIN validated")
        dispatch = next(
            match.start() for match in re.finditer(r"\[MotionPlanned\][^\n]*command=MovePJ", log)
        )
        self.assertLess(precheck, dispatch)
        if direction == "down":
            self.assertIn("accepted=false reason=PILZ_PATH_COLLISION", log)
            self.assertEqual(report["transit_planner"], "PilzPTP")
        return goals[0]

    def test_recorded_down_collision_uses_repeatable_ptp_posture(self):
        self.assertEqual(self.replay("down"), self.replay("down"))

    def test_up_remains_executable(self):
        self.replay("up")


if __name__ == "__main__":
    unittest.main()

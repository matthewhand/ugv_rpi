#!/usr/bin/env python3
"""RoArm-M2-S forward kinematics vs Waveshare firmware (EEMode=0 clamp).

Numbers from waveshareteam/roarm_m2 RoArm-M2_config.h / RoArm-M2_module.h:
  polarToCartesian(l2, π/2 − (shoulder + t2rad))
  polarToCartesian(l3, π/2 − (elbow + shoulder))
  then yaw base (X+ forward, Y+ left, Z+ up from the shoulder).
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import roarm_ctrl  # noqa: E402


class TestRoArmM2Kinematics(unittest.TestCase):
    def test_link_lengths_match_waveshare_firmware(self):
        self.assertAlmostEqual(roarm_ctrl.ARM_L1_MM, 126.06, places=2)
        self.assertAlmostEqual(roarm_ctrl.ARM_L2A_MM, 236.82, places=2)
        self.assertAlmostEqual(roarm_ctrl.ARM_L2B_MM, 30.00, places=2)
        self.assertAlmostEqual(roarm_ctrl.ARM_L3A_MM, 280.15, places=2)
        self.assertAlmostEqual(roarm_ctrl.ARM_INIT_X_MM, 310.15, places=2)

    def test_home_is_inverted_l_reaching_forward(self):
        # T:100 / T:102 home: base=0, shoulder=0, elbow=π/2
        fk = roarm_ctrl.forward_kinematics(0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(fk["y"], 0.0, places=6)
        # Reach = l2B + l3 ≈ 310.15 mm (firmware initX)
        self.assertAlmostEqual(fk["x"] * 1000.0, roarm_ctrl.ARM_INIT_X_MM, delta=0.3)
        # EEMode=0 FK z from shoulder is L2A (236.82 mm), not the IK seed L2A-L3B
        self.assertAlmostEqual(fk["z"] * 1000.0, roarm_ctrl.ARM_L2A_MM, delta=0.3)
        self.assertAlmostEqual(
            fk["z_world"] * 1000.0,
            roarm_ctrl.ARM_L1_MM + roarm_ctrl.ARM_L2A_MM,
            delta=0.3,
        )

    def test_positive_base_yaws_left(self):
        # Wiki: increasing base turns LEFT = +Y
        fk = roarm_ctrl.forward_kinematics(math.pi / 2, 0.0, math.pi / 2)
        self.assertAlmostEqual(fk["x"], 0.0, delta=1e-4)
        self.assertGreater(fk["y"], 0.2)

    def test_positive_shoulder_leans_forward(self):
        home = roarm_ctrl.forward_kinematics(0.0, 0.0, math.pi / 2)
        fwd = roarm_ctrl.forward_kinematics(0.0, 0.35, math.pi / 2)
        self.assertGreater(fwd["x"], home["x"])
        self.assertLess(fwd["z"], home["z"])

    def test_travel_tuck_is_behind_home_not_a_long_reach(self):
        home = roarm_ctrl.forward_kinematics(0.0, 0.0, math.pi / 2)
        tuck = roarm_ctrl.POSES["travel_tuck"]
        fk = roarm_ctrl.forward_kinematics(tuck["base"], tuck["shoulder"], tuck["elbow"])
        # Shoulder lean-back + elbow < π/2 folds the forearm up/back (wiki: smaller elbow = up).
        self.assertLess(fk["x"], home["x"])
        self.assertLess(fk["x"], 0.12)  # not the inverted-L stick-out
        self.assertLess(abs(fk["y"]), 1e-9)

    def test_e_z_r_default_is_home_joints(self):
        j = roarm_ctrl.e_z_r_to_joints(60, 24, 0)
        self.assertAlmostEqual(j["base"], 0.0, places=5)
        self.assertAlmostEqual(j["shoulder"], 0.0, places=5)
        self.assertAlmostEqual(j["elbow"], 1.5708, places=3)


if __name__ == "__main__":
    unittest.main()

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

    def test_e_z_r_reach_extends_elbow(self):
        home = roarm_ctrl.e_z_r_to_joints(60, 24, 0)
        far = roarm_ctrl.e_z_r_to_joints(200, 24, 0)
        self.assertLess(far["elbow"], home["elbow"])

    def test_e_z_r_r_yaws_left(self):
        j = roarm_ctrl.e_z_r_to_joints(60, 24, 45)
        self.assertGreater(j["base"], 0.5)

    def test_kinematics_public_matches_constants(self):
        pub = roarm_ctrl.kinematics_public()
        self.assertAlmostEqual(pub["l1_mm"], roarm_ctrl.ARM_L1_MM)
        self.assertAlmostEqual(pub["init_x_mm"], 310.15, places=2)


def _planar_mm(fk):
    """Signed (reach, height) in mm from a forward_kinematics result."""
    return fk["r"] * 1000.0, fk["z"] * 1000.0


class TestPlanarIK(unittest.TestCase):
    """Closed-form planar IK must be the exact inverse of the firmware FK."""

    def test_ik_home_target_recovers_home_joints(self):
        sol = roarm_ctrl.planar_ik(
            roarm_ctrl.ARM_INIT_X_MM, roarm_ctrl.ARM_L2A_MM,
            seed_shoulder=0.0, seed_elbow=math.pi / 2,
        )
        self.assertIsNotNone(sol)
        self.assertAlmostEqual(sol["shoulder"], 0.0, places=4)
        self.assertAlmostEqual(sol["elbow"], math.pi / 2, places=3)

    def test_ik_fk_roundtrip_across_workspace(self):
        home = roarm_ctrl.POSES["home"]
        for r_mm, z_mm in [
            (310.15, 236.82),   # home
            (200.0, 300.0),
            (350.0, 150.0),
            (250.0, 120.0),
            (280.0, 80.0),
            (300.0, 200.0),
            (-41.16, 480.91),   # travel_tuck leans BACKWARD (signed reach)
        ]:
            sol = roarm_ctrl.planar_ik(
                r_mm, z_mm,
                seed_shoulder=home["shoulder"], seed_elbow=home["elbow"],
            )
            self.assertIsNotNone(sol, f"unreachable sample ({r_mm},{z_mm})")
            self.assertFalse(sol["clamped"], f"sample ({r_mm},{z_mm}) should fit safe limits")
            fk = roarm_ctrl.forward_kinematics(0.0, sol["shoulder"], sol["elbow"])
            got_r, got_z = _planar_mm(fk)
            self.assertAlmostEqual(got_r, r_mm, delta=0.5, msg=f"r miss at ({r_mm},{z_mm})")
            self.assertAlmostEqual(got_z, z_mm, delta=0.5, msg=f"z miss at ({r_mm},{z_mm})")

    def test_ik_clamped_solution_is_honest(self):
        # Reachable in theory, but only with elbow beyond the safe limit.
        sol = roarm_ctrl.planar_ik(250.0, 50.0)
        self.assertIsNotNone(sol)
        self.assertTrue(sol["clamped"])
        fk = roarm_ctrl.forward_kinematics(0.0, sol["shoulder"], sol["elbow"])
        got_r, got_z = _planar_mm(fk)
        # Best-effort pose stays close and inside limits
        self.assertLess(math.hypot(got_r - 250.0, got_z - 50.0), 60.0)
        self.assertLessEqual(sol["elbow"], roarm_ctrl._ELBOW_LIM[1] + 1e-9)

    def test_ik_branch_is_continuous_with_seed(self):
        # From travel_tuck, solving near the tuck pose must stay near it
        tuck = roarm_ctrl.POSES["travel_tuck"]
        fk = roarm_ctrl.forward_kinematics(0.0, tuck["shoulder"], tuck["elbow"])
        r0, z0 = _planar_mm(fk)
        sol = roarm_ctrl.planar_ik(
            r0 + 20.0, z0,
            seed_shoulder=tuck["shoulder"], seed_elbow=tuck["elbow"],
        )
        self.assertLess(abs(sol["shoulder"] - tuck["shoulder"]), 0.35)
        self.assertLess(abs(sol["elbow"] - tuck["elbow"]), 0.6)

    def test_ik_unreachable_target_returns_none(self):
        far = (roarm_ctrl.ARM_L2_MM + roarm_ctrl.ARM_L3_MM) * 1000.0 + 40.0
        self.assertIsNone(roarm_ctrl.planar_ik(far, 0.0))
        too_in = abs(roarm_ctrl.ARM_L2_MM - roarm_ctrl.ARM_L3_MM) * 1000.0 - 40.0
        if too_in > 0:
            self.assertIsNone(roarm_ctrl.planar_ik(too_in, 0.0))

    def test_ik_respects_joint_limits_or_reports_clamped(self):
        # Straight out at full extension needs shoulder≈0 elbow≈t2 (< limit lo 0.85)
        sol = roarm_ctrl.planar_ik(
            (roarm_ctrl.ARM_L2_MM + roarm_ctrl.ARM_L3_MM) * 1000.0 - 5.0, 0.0
        )
        if sol is not None:
            self.assertTrue(sol.get("clamped"), "near-full extension must flag clamped")
        for key, lim in (("shoulder", roarm_ctrl._SHOULDER_LIM), ("elbow", roarm_ctrl._ELBOW_LIM)):
            if sol is not None and not sol.get("clamped"):
                self.assertGreaterEqual(sol[key], lim[0])
                self.assertLessEqual(sol[key], lim[1])


class TestRelativeMove(unittest.TestCase):
    """relative_move: reach changes keep height; lift changes keep reach."""

    def test_reach_forward_keeps_height(self):
        start = dict(roarm_ctrl.POSES["scan_ready"])
        res = roarm_ctrl.relative_move(dr_mm=30.0, dz_mm=0.0, joints=start)
        self.assertTrue(res["ok"])
        fk0 = roarm_ctrl.forward_kinematics(0.0, start["shoulder"], start["elbow"])
        fk1 = roarm_ctrl.forward_kinematics(0.0, res["joints"]["shoulder"], res["joints"]["elbow"])
        r0, z0 = _planar_mm(fk0)
        r1, z1 = _planar_mm(fk1)
        self.assertAlmostEqual(r1 - r0, 30.0, delta=2.0)
        self.assertAlmostEqual(z1, z0, delta=2.0, msg="reach move must not change gripper height")
        self.assertAlmostEqual(res["joints"]["base"], start["base"], places=6)
        self.assertEqual(res["joints"]["hand"], start["hand"])

    def test_reach_back_keeps_height(self):
        start = dict(roarm_ctrl.POSES["home"])
        res = roarm_ctrl.relative_move(dr_mm=-25.0, joints=start)
        self.assertTrue(res["ok"])
        fk0 = roarm_ctrl.forward_kinematics(0.0, start["shoulder"], start["elbow"])
        fk1 = roarm_ctrl.forward_kinematics(0.0, res["joints"]["shoulder"], res["joints"]["elbow"])
        r0, z0 = _planar_mm(fk0)
        r1, z1 = _planar_mm(fk1)
        self.assertAlmostEqual(r1 - r0, -25.0, delta=2.0)
        self.assertAlmostEqual(z1, z0, delta=2.0)

    def test_lift_keeps_reach(self):
        start = dict(roarm_ctrl.POSES["scan_ready"])
        res = roarm_ctrl.relative_move(dr_mm=0.0, dz_mm=20.0, joints=start)
        self.assertTrue(res["ok"])
        fk0 = roarm_ctrl.forward_kinematics(0.0, start["shoulder"], start["elbow"])
        fk1 = roarm_ctrl.forward_kinematics(0.0, res["joints"]["shoulder"], res["joints"]["elbow"])
        r0, z0 = _planar_mm(fk0)
        r1, z1 = _planar_mm(fk1)
        self.assertAlmostEqual(z1 - z0, 20.0, delta=2.0)
        self.assertAlmostEqual(r1, r0, delta=2.0, msg="lift move must not change gripper reach")

    def test_yaw_moves_base_only(self):
        start = dict(roarm_ctrl.POSES["travel_tuck"])
        res = roarm_ctrl.relative_move(dyaw_deg=10.0, joints=start)
        self.assertTrue(res["ok"])
        self.assertAlmostEqual(
            res["joints"]["base"],
            start["base"] + math.radians(10.0),
            delta=1e-9,
        )
        self.assertAlmostEqual(res["joints"]["shoulder"], start["shoulder"], places=9)
        self.assertAlmostEqual(res["joints"]["elbow"], start["elbow"], places=9)

    def test_result_within_limits_and_flagged_when_clamped(self):
        start = dict(roarm_ctrl.POSES["home"])
        res = roarm_ctrl.relative_move(dr_mm=400.0, joints=start)  # beyond full reach
        for key, lim in (("shoulder", roarm_ctrl._SHOULDER_LIM), ("elbow", roarm_ctrl._ELBOW_LIM)):
            self.assertGreaterEqual(res["joints"][key], lim[0] - 1e-9)
            self.assertLessEqual(res["joints"][key], lim[1] + 1e-9)
        if not res.get("clamped"):
            self.assertTrue(res["ok"])

    def test_default_seed_is_travel_tuck(self):
        res = roarm_ctrl.relative_move(dz_mm=5.0)
        self.assertTrue(res["ok"])
        self.assertIsNotNone(res["joints"])

    def test_diagonal_move_changes_both_axes(self):
        start = dict(roarm_ctrl.POSES["scan_ready"])
        res = roarm_ctrl.relative_move(dr_mm=20.0, dz_mm=-15.0, joints=start)
        self.assertTrue(res["ok"])
        fk0 = roarm_ctrl.forward_kinematics(0.0, start["shoulder"], start["elbow"])
        fk1 = roarm_ctrl.forward_kinematics(0.0, res["joints"]["shoulder"], res["joints"]["elbow"])
        r0, z0 = _planar_mm(fk0)
        r1, z1 = _planar_mm(fk1)
        self.assertAlmostEqual(r1 - r0, 20.0, delta=2.5)
        self.assertAlmostEqual(z1 - z0, -15.0, delta=2.5)


class TestArmMoveEndpointGate(unittest.TestCase):
    """POST /api/arm/move exists and is hangar-gated like the T:144 path."""

    def test_route_and_gate_present(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def api_arm_move")
        self.assertGreater(idx, 0, "/api/arm/move route missing")
        body = src[idx : idx + 2400]
        self.assertIn("arm_usb_enabled()", body)
        self.assertIn("relative_move", body)
        # gate check happens before any motion is commanded
        gate_pos = body.find("arm_usb_enabled()")
        move_pos = body.find("relative_move")
        self.assertLess(gate_pos, move_pos)

    def test_ui_calls_arm_move_for_stick_reach(self):
        src = Path(ROOT, "templates", "control.js").read_text(encoding="utf-8")
        self.assertIn("/api/arm/move", src)
        # wheel reach must hit the IK queue before any stick-drag requirement
        wheel_idx = src.find("addEventListener('wheel'")
        self.assertGreater(wheel_idx, 0, "wheel listener missing")
        wheel_body = src[wheel_idx : wheel_idx + 1600]
        jog_pos = wheel_body.find("roarmWheelJog")
        self.assertGreater(jog_pos, 0, "wheel must call roarmWheelJog")
        drag_pos = wheel_body.find("isDragging")
        if drag_pos != -1:
            # legacy PTZ path may keep its gate, but only AFTER the IK branch
            self.assertLess(jog_pos, drag_pos)
        self.assertIn("function roarmWheelJog", src)
        self.assertIn("roarmQueueMove(yaw, lift, reach)", src)
        # stick drives yaw+lift deltas through the same queue
        self.assertIn("roarmQueueStick(ddx, ddy)", src)
        self.assertIn("-ddx", src)
        self.assertIn("roarmPrevStickX", src)


if __name__ == "__main__":
    unittest.main()

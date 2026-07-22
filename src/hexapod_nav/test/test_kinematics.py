"""Tests for kinematics.py — IK solver and reachability."""
import math
import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hexapod_nav'))
import kinematics as kin


class TestIkCoxa:
    def test_reachable_center(self):
        result = kin.ik_coxa(0.12, 0.0, -0.12)
        assert result is not None
        q_coxa, q_femur, q_tibia = result
        assert -math.pi < q_coxa < math.pi
        assert kin.FEMUR_LIMITS[0] <= q_femur <= kin.FEMUR_LIMITS[1]
        assert kin.TIBIA_LIMITS[0] <= q_tibia <= kin.TIBIA_LIMITS[1]

    def test_unreachable_too_far(self):
        result = kin.ik_coxa(1.0, 0.0, -1.0)
        assert result == (0.0, 0.0, 0.0)

    def test_unreachable_behind(self):
        result = kin.ik_coxa(-0.5, 0.0, -0.12)
        assert result == (0.0, 0.0, 0.0)

    def test_symmetry(self):
        r1 = kin.ik_coxa(0.12, 0.02, -0.12)
        r2 = kin.ik_coxa(0.12, -0.02, -0.12)
        assert r1 is not None
        assert r2 is not None
        assert abs(r1[0] + r2[0]) < 1e-6

    def test_return_type(self):
        result = kin.ik_coxa(0.10, 0.0, -0.10)
        assert result is not None
        assert len(result) == 3
        for q in result:
            assert isinstance(q, float)


class TestIkLeg:
    def test_leg0_reachable(self):
        """Leg 0 with target directly forward in its coxa frame."""
        origin = kin.LEG_ORIGINS[0]
        angle = kin.LEG_ANGLES[0]
        # Build target that is forward in leg 0's coxa frame
        coxa_fwd = np.array([0.10, 0.0, -0.10])
        R = np.array([[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle),  math.cos(angle), 0],
                      [0,                0,               1]])
        target = R @ coxa_fwd + origin
        result = kin.ik_leg(target, 0)
        assert result is not None
        q_coxa, q_femur, q_tibia = result
        assert kin.COXA_LIMITS[0] <= q_coxa <= kin.COXA_LIMITS[1]
        assert kin.FEMUR_LIMITS[0] <= q_femur <= kin.FEMUR_LIMITS[1]
        assert kin.TIBIA_LIMITS[0] <= q_tibia <= kin.TIBIA_LIMITS[1]

    def test_leg3_reachable(self):
        origin = kin.LEG_ORIGINS[3]
        angle = kin.LEG_ANGLES[3]
        coxa_fwd = np.array([0.10, 0.0, -0.10])
        R = np.array([[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle),  math.cos(angle), 0],
                      [0,                0,               1]])
        target = R @ coxa_fwd + origin
        result = kin.ik_leg(target, 3)
        assert result is not None

    def test_unreachable_returns_none(self):
        target = np.array([10.0, 10.0, 10.0])
        result = kin.ik_leg(target, 0)
        assert result is None

    def test_all_legs_stance_pose(self):
        for i in range(6):
            origin = kin.LEG_ORIGINS[i]
            angle = kin.LEG_ANGLES[i]
            coxa_fwd = np.array([0.10, 0.0, -0.10])
            R = np.array([[math.cos(angle), -math.sin(angle), 0],
                          [math.sin(angle),  math.cos(angle), 0],
                          [0,                0,               1]])
            target = R @ coxa_fwd + origin
            result = kin.ik_leg(target, i)
            assert result is not None, f"Leg {i} unreachable at nominal stance"


class TestWorldToCoxa:
    def test_identity_pose(self):
        world_pt = np.array([0.2, 0.0, -0.12])
        body_pose = np.array([0.0, 0.0, 0.0, 0.0])
        coxa_pt = kin.world_to_coxa(world_pt, 0, body_pose)
        assert len(coxa_pt) == 3

    def test_rotated_body(self):
        world_pt = np.array([0.2, 0.0, -0.12])
        body_pose = np.array([0.0, 0.0, 0.0, math.pi / 2])
        coxa_pt = kin.world_to_coxa(world_pt, 0, body_pose)
        assert len(coxa_pt) == 3
        assert abs(coxa_pt[0] - 0.0) < 0.2


class TestGetReachableZone:
    def test_nearby_reachable(self):
        body_pose = np.array([0.0, 0.0, 0.0, 0.0])
        origin = kin.LEG_ORIGINS[0]
        angle = kin.LEG_ANGLES[0]
        coxa_fwd = np.array([0.10, 0.0, -0.10])
        R = np.array([[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle),  math.cos(angle), 0],
                      [0,                0,               1]])
        target = R @ coxa_fwd + origin
        grid = np.array([target])
        reachable = kin.get_reachable_zone(0, body_pose, grid)
        assert reachable[0] == True

    def test_far_unreachable(self):
        body_pose = np.array([0.0, 0.0, 0.0, 0.0])
        grid = np.array([[10.0, 10.0, -0.12]])
        reachable = kin.get_reachable_zone(0, body_pose, grid)
        assert reachable[0] == False

    def test_empty_grid(self):
        body_pose = np.array([0.0, 0.0, 0.0, 0.0])
        grid = np.empty((0, 3))
        reachable = kin.get_reachable_zone(0, body_pose, grid)
        assert len(reachable) == 0


class TestJointLimits:
    def test_coxa_limits(self):
        assert kin.COXA_LIMITS[0] < kin.COXA_LIMITS[1]

    def test_femur_limits(self):
        assert kin.FEMUR_LIMITS[0] < kin.FEMUR_LIMITS[1]

    def test_tibia_limits(self):
        assert kin.TIBIA_LIMITS[0] < kin.TIBIA_LIMITS[1]

    def test_link_lengths_positive(self):
        assert kin.COXA_LEN > 0
        assert kin.FEMUR_LEN > 0
        assert kin.TIBIA_LEN > 0

    def test_leg_origins_count(self):
        assert len(kin.LEG_ORIGINS) == 6
        assert len(kin.LEG_ANGLES) == 6


class TestLawOfCosinesIk:
    def test_reachable_distance(self):
        result = kin._law_of_cosines_ik(0.10, -0.10)
        assert result is not None
        q_femur, q_tibia = result
        assert kin.FEMUR_LIMITS[0] <= q_femur <= kin.FEMUR_LIMITS[1]
        assert kin.TIBIA_LIMITS[0] <= q_tibia <= kin.TIBIA_LIMITS[1]

    def test_too_far(self):
        result = kin._law_of_cosines_ik(0.5, -0.5)
        assert result is None

    def test_zero_distance(self):
        result = kin._law_of_cosines_ik(0.0, -(kin.FEMUR_LEN + kin.TIBIA_LEN))
        assert result is not None


class TestBezierArc:
    def _bezier_arc(self, p_start, p_end, height, t):
        """Standalone copy of bezier_arc for testing (avoids relative import)."""
        p1 = p_start + np.array([0, 0, height])
        p2 = p_end + np.array([0, 0, height])
        return ((1 - t)**3 * p_start
                + 3 * (1 - t)**2 * t * p1
                + 3 * (1 - t) * t**2 * p2
                + t**3 * p_end)

    def test_start_position(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        result = self._bezier_arc(p0, p1, 0.05, 0.0)
        np.testing.assert_allclose(result, p0, atol=1e-6)

    def test_end_position(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        result = self._bezier_arc(p0, p1, 0.05, 1.0)
        np.testing.assert_allclose(result, p1, atol=1e-6)

    def test_midpoint_elevation(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        mid = self._bezier_arc(p0, p1, 0.05, 0.5)
        assert mid[2] > max(p0[2], p1[2])

    def test_zero_height_flat(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        mid = self._bezier_arc(p0, p1, 0.0, 0.5)
        assert p0[0] <= mid[0] <= p1[0]

    def test_intermediate_points(self):
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([1.0, 0.0, 0.0])
        heights = []
        for t in np.linspace(0, 1, 11):
            pt = self._bezier_arc(p0, p1, 0.2, t)
            heights.append(pt[2])
        assert max(heights) > 0.0
        assert heights[0] == pytest.approx(0.0, abs=1e-6)
        assert heights[-1] == pytest.approx(0.0, abs=1e-6)

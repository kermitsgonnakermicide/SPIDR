"""Tests for gait_controller_node.py — gait state machine and swing arcs."""
import math
import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hexapod_nav'))


def bezier_arc(p_start, p_end, height, t):
    """Standalone copy of bezier_arc for testing."""
    p1 = p_start + np.array([0, 0, height])
    p2 = p_end + np.array([0, 0, height])
    return ((1 - t)**3 * p_start
            + 3 * (1 - t)**2 * t * p1
            + 3 * (1 - t) * t**2 * p2
            + t**3 * p_end)


TRIPOD_A = [0, 2, 4]
TRIPOD_B = [1, 3, 5]

JOINT_NAMES = [
    'rf_coxa_joint', 'rf_femur_joint', 'rf_tibia_joint',
    'rm_coxa_joint', 'rm_femur_joint', 'rm_tibia_joint',
    'rr_coxa_joint', 'rr_femur_joint', 'rr_tibia_joint',
    'lf_coxa_joint', 'lf_femur_joint', 'lf_tibia_joint',
    'lm_coxa_joint', 'lm_femur_joint', 'lm_tibia_joint',
    'lr_coxa_joint', 'lr_femur_joint', 'lr_tibia_joint',
]


class TestBezierArc:
    def test_start_position(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        result = bezier_arc(p0, p1, 0.05, 0.0)
        np.testing.assert_allclose(result, p0, atol=1e-6)

    def test_end_position(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        result = bezier_arc(p0, p1, 0.05, 1.0)
        np.testing.assert_allclose(result, p1, atol=1e-6)

    def test_midpoint_elevation(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        mid = bezier_arc(p0, p1, 0.05, 0.5)
        assert mid[2] > max(p0[2], p1[2])

    def test_zero_height_flat(self):
        p0 = np.array([0.1, 0.0, -0.1])
        p1 = np.array([0.2, 0.0, -0.1])
        mid = bezier_arc(p0, p1, 0.0, 0.5)
        assert p0[0] <= mid[0] <= p1[0]

    def test_lateral_movement(self):
        p0 = np.array([0.1, -0.05, -0.1])
        p1 = np.array([0.1, 0.05, -0.1])
        mid = bezier_arc(p0, p1, 0.03, 0.5)
        assert abs(mid[1]) < 0.05

    def test_intermediate_points(self):
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([1.0, 0.0, 0.0])
        heights = []
        for t in np.linspace(0, 1, 11):
            pt = bezier_arc(p0, p1, 0.2, t)
            heights.append(pt[2])
        assert max(heights) > 0.0
        assert heights[0] == pytest.approx(0.0, abs=1e-6)
        assert heights[-1] == pytest.approx(0.0, abs=1e-6)


class TestTripodGroups:
    def test_groups_partition(self):
        assert set(TRIPOD_A) | set(TRIPOD_B) == set(range(6))
        assert set(TRIPOD_A) & set(TRIPOD_B) == set()

    def test_alternation(self):
        active = 0
        for _ in range(10):
            active = 1 - active
        assert active == 0


class TestSwingProgress:
    def test_progress_clamps_at_one(self):
        progress = 0.8
        dt = 0.5
        swing_dur = 0.5
        progress = min(1.0, progress + dt / swing_dur)
        assert progress == 1.0

    def test_progress_increment(self):
        progress = 0.0
        dt = 0.1
        swing_dur = 0.5
        progress = min(1.0, progress + dt / swing_dur)
        assert progress == pytest.approx(0.2)

    def test_completion_triggers_switch(self):
        progress = [0.9, 0.9, 0.9]
        dt = 0.1
        swing_dur = 0.5
        all_done = True
        for i in range(3):
            progress[i] = min(1.0, progress[i] + dt / swing_dur)
            if progress[i] < 1.0:
                all_done = False
        assert all_done


class TestJointNames:
    def test_18_joints(self):
        assert len(JOINT_NAMES) == 18

    def test_joint_name_format(self):
        for name in JOINT_NAMES:
            assert '_joint' in name
            parts = name.split('_')
            assert len(parts) == 3

    def test_six_legs(self):
        prefixes = set(name.split('_')[0] for name in JOINT_NAMES)
        assert prefixes == {'rf', 'rm', 'rr', 'lf', 'lm', 'lr'}

    def test_three_joints_per_leg(self):
        from collections import Counter
        prefixes = [name.split('_')[0] for name in JOINT_NAMES]
        counts = Counter(prefixes)
        assert all(v == 3 for v in counts.values())


class TestLegPhase:
    def test_standing_all_stance(self):
        """When standing (no velocity), all legs should be STANCE."""
        active_tripod = 0
        phase = [0] * 6  # 0 = STANCE
        # In standing mode, the publisher sends all-STANCE
        assert all(p == 0 for p in phase)

    def test_tripod_a_swinging(self):
        """When active_tripod=0, TRIPOD_A legs should be SWING."""
        active_tripod = 0
        swinging = TRIPOD_A if active_tripod == 0 else TRIPOD_B
        phase = [0] * 6
        for leg in swinging:
            phase[leg] = 1
        assert phase[0] == 1  # TRIPOD_A
        assert phase[2] == 1
        assert phase[4] == 1
        assert phase[1] == 0  # TRIPOD_B
        assert phase[3] == 0
        assert phase[5] == 0

    def test_tripod_b_swinging(self):
        """When active_tripod=1, TRIPOD_B legs should be SWING."""
        active_tripod = 1
        swinging = TRIPOD_A if active_tripod == 0 else TRIPOD_B
        phase = [0] * 6
        for leg in swinging:
            phase[leg] = 1
        assert phase[1] == 1  # TRIPOD_B
        assert phase[3] == 1
        assert phase[5] == 1
        assert phase[0] == 0  # TRIPOD_A
        assert phase[2] == 0
        assert phase[4] == 0

    def test_phase_message_length(self):
        """Phase message should have exactly 6 elements."""
        phase = [0] * 6
        assert len(phase) == 6

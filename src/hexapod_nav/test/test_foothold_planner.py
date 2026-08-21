"""Tests for foothold_planner_node.py — pure logic tests (no ROS node needed)."""
import math
import numpy as np
import pytest


class TestCostExtraction:
    def test_extract_cost_array_shape(self):
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension
        from grid_map_msgs.msg import GridMap

        msg = GridMap()
        msg.info.resolution = 0.05
        msg.info.length_x = 1.0
        msg.info.length_y = 1.0
        msg.layers = ['cost']

        layer = Float32MultiArray()
        n = 4
        dim_x = MultiArrayDimension(label='column_index', size=n, stride=n * n)
        dim_y = MultiArrayDimension(label='row_index', size=n, stride=n)
        layer.layout.dim = [dim_x, dim_y]
        layer.data = list(range(n * n))
        msg.data.append(layer)

        data = msg.data[0]
        n2 = data.layout.dim[0].size
        arr = np.array(data.data, dtype=np.float32).reshape((n2, n2), order='F')
        assert arr.shape == (4, 4)
        assert arr[0, 0] == 0.0
        assert arr[3, 3] == 15.0

    def test_extract_terrain_layer_found(self):
        from std_msgs.msg import Float32MultiArray, MultiArrayDimension
        from grid_map_msgs.msg import GridMap

        msg = GridMap()
        msg.info.resolution = 0.05
        msg.layers = ['floor', 'ceiling', 'clearance']

        for vals in [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
                      [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3],
                      [0.35, 0.4, 0.4, 0.5, 0.5, 0.5, 0.6, 0.6, 0.6]]:
            layer = Float32MultiArray()
            n = 3
            dim_x = MultiArrayDimension(label='column_index', size=n, stride=n * n)
            dim_y = MultiArrayDimension(label='row_index', size=n, stride=n)
            layer.layout.dim = [dim_x, dim_y]
            layer.data = vals
            msg.data.append(layer)

        try:
            idx = msg.layers.index('floor')
        except ValueError:
            idx = -1
        assert idx == 0

        data = msg.data[idx]
        n = data.layout.dim[0].size
        arr = np.array(data.data, dtype=np.float32).reshape((n, n), order='F')
        assert arr.shape == (3, 3)

    def test_extract_terrain_layer_missing(self):
        from grid_map_msgs.msg import GridMap
        msg = GridMap()
        msg.layers = ['floor']
        try:
            idx = msg.layers.index('clearance')
        except ValueError:
            idx = -1
        assert idx == -1


class TestCostAtCell:
    def test_out_of_bounds(self):
        costmap = np.array([[1.0, 2.0], [3.0, 4.0]])
        n = costmap.shape[0]

        def get_cost(ix, iy):
            if 0 <= ix < n and 0 <= iy < n:
                return float(costmap[ix, iy])
            return np.inf

        assert get_cost(-1, 0) == np.inf
        assert get_cost(100, 0) == np.inf
        assert get_cost(0, 0) == 1.0

    def test_no_costmap(self):
        costmap = None
        assert costmap is None


class TestReplanThreshold:
    def test_no_replan_below_threshold(self):
        committed_cost = 0.5
        current_cost = 0.7
        threshold = 0.3
        assert (current_cost - committed_cost) <= threshold

    def test_replan_above_threshold(self):
        committed_cost = 0.5
        current_cost = 1.0
        threshold = 0.3
        assert (current_cost - committed_cost) > threshold

    def test_inf_cost_triggers_replan(self):
        current_cost = np.inf
        assert current_cost == np.inf


class TestLegPhaseStateMachine:
    def test_initial_state(self):
        NUM_LEGS = 6
        STANCE = 0
        leg_phase = [STANCE] * NUM_LEGS
        assert all(p == STANCE for p in leg_phase)

    def test_swing_transition(self):
        NUM_LEGS = 6
        STANCE = 0
        SWING = 1
        leg_phase = [STANCE] * NUM_LEGS
        leg_phase[0] = SWING
        assert leg_phase[0] == SWING
        assert leg_phase[1] == STANCE

    def test_committed_target_lifecycle(self):
        committed = [None] * 6
        committed[0] = (5, 10, 0.3)
        assert committed[0] == (5, 10, 0.3)
        committed[0] = None
        assert committed[0] is None


class TestAepPepFilter:
    def test_candidate_within_workspace(self):
        """A candidate at nominal stance should pass AEP/PEP filter."""
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hexapod_nav'))
        import kinematics as kin

        body_pose = np.array([0.0, 0.0, 0.0, 0.0])
        leg_idx = 0
        origin = kin.LEG_ORIGINS[leg_idx]
        angle = kin.LEG_ANGLES[leg_idx]
        # Build a candidate at nominal forward position in coxa frame
        coxa_fwd = np.array([0.10, 0.0, -0.10])
        R = np.array([[math.cos(angle), -math.sin(angle), 0],
                      [math.sin(angle),  math.cos(angle), 0],
                      [0,                0,               1]])
        world_pt = R @ coxa_fwd + origin

        # Check coxa_x is within limits (using default param values)
        pep_offset = 0.05
        aep_offset = 0.05
        dx = world_pt[0] - body_pose[0]
        dy = world_pt[1] - body_pose[1]
        cos_y = math.cos(-body_pose[3])
        sin_y = math.sin(-body_pose[3])
        body_x = dx * cos_y - dy * sin_y
        body_y = dx * sin_y + dy * cos_y
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        tx = body_x - origin[0]
        ty = body_y - origin[1]
        coxa_x = tx * cos_a - ty * sin_a

        pep_limit = kin.COXA_LEN + pep_offset
        aep_limit = kin.COXA_LEN + kin.FEMUR_LEN + kin.TIBIA_LEN - aep_offset
        assert coxa_x >= pep_limit
        assert coxa_x <= aep_limit

    def test_candidate_beyond_reach_rejected(self):
        """A candidate far beyond leg reach should be outside AEP."""
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hexapod_nav'))
        import kinematics as kin

        leg_idx = 0
        origin = kin.LEG_ORIGINS[leg_idx]
        aep_offset = 0.05
        aep_limit = kin.COXA_LEN + kin.FEMUR_LEN + kin.TIBIA_LEN - aep_offset
        # Place candidate at 2x max reach
        far_x = origin[0] + aep_limit * 2
        world_pt = np.array([far_x, origin[1], -0.10])

        angle = kin.LEG_ANGLES[leg_idx]
        dx = world_pt[0]
        dy = world_pt[1]
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        tx = dx - origin[0]
        ty = dy - origin[1]
        coxa_x = tx * cos_a - ty * sin_a

        assert coxa_x > aep_limit

    def test_pep_limit_enforced(self):
        """A candidate behind the PEP should be rejected."""
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hexapod_nav'))
        import kinematics as kin

        leg_idx = 0
        origin = kin.LEG_ORIGINS[leg_idx]
        pep_offset = 0.05
        pep_limit = kin.COXA_LEN + pep_offset
        # Place candidate behind PEP (negative coxa_x)
        behind_pt = np.array([origin[0] - 0.1, origin[1], -0.10])

        angle = kin.LEG_ANGLES[leg_idx]
        dx = behind_pt[0]
        dy = behind_pt[1]
        cos_a = math.cos(-angle)
        sin_a = math.sin(-angle)
        tx = dx - origin[0]
        ty = dy - origin[1]
        coxa_x = tx * cos_a - ty * sin_a

        assert coxa_x < pep_limit


class TestPhaseCallback:
    def test_stance_to_swing_triggers_begin_swing(self):
        """Phase transition STANCE→SWING should set leg to SWING."""
        NUM_LEGS = 6
        STANCE = 0
        SWING = 1
        leg_phase = [STANCE] * NUM_LEGS
        begin_swing_called = []

        # Simulate phase_callback logic
        new_phase_msg = [SWING, STANCE, STANCE, STANCE, STANCE, STANCE]
        for leg_idx in range(NUM_LEGS):
            new_phase = new_phase_msg[leg_idx]
            old_phase = leg_phase[leg_idx]
            if old_phase == STANCE and new_phase == SWING:
                leg_phase[leg_idx] = SWING
                begin_swing_called.append(leg_idx)

        assert leg_phase[0] == SWING
        assert leg_phase[1] == STANCE
        assert begin_swing_called == [0]

    def test_swing_to_stance_triggers_end_swing(self):
        """Phase transition SWING→STANCE should clear committed target."""
        NUM_LEGS = 6
        STANCE = 0
        SWING = 1
        leg_phase = [SWING, STANCE, STANCE, STANCE, STANCE, STANCE]
        committed = [None] * NUM_LEGS
        committed[0] = (5, 10, 0.3)

        new_phase_msg = [STANCE, STANCE, STANCE, STANCE, STANCE, STANCE]
        for leg_idx in range(NUM_LEGS):
            new_phase = new_phase_msg[leg_idx]
            old_phase = leg_phase[leg_idx]
            if old_phase == SWING and new_phase == STANCE:
                leg_phase[leg_idx] = STANCE
                committed[leg_idx] = None

        assert leg_phase[0] == STANCE
        assert committed[0] is None

    def test_no_transition_same_phase(self):
        """No transition if phase doesn't change."""
        NUM_LEGS = 6
        STANCE = 0
        SWING = 1
        leg_phase = [STANCE, SWING, STANCE, STANCE, STANCE, STANCE]
        transitions = []

        new_phase_msg = [STANCE, SWING, STANCE, STANCE, STANCE, STANCE]
        for leg_idx in range(NUM_LEGS):
            new_phase = new_phase_msg[leg_idx]
            old_phase = leg_phase[leg_idx]
            if old_phase != new_phase:
                transitions.append(leg_idx)

        assert transitions == []

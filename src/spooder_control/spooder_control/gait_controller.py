import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np
from spooder_control.ik_solver import IKSolver

class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/spooder_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscribe to terrain height from point cloud analysis
        self.terrain_height_subscription = self.create_subscription(
            Float32,
            '/perception/terrain_height',
            self.terrain_height_callback,
            10
        )
        
        self.get_logger().info("Gait Controller Initialized with 3D Adaptive Step Height")

        
        self.ik = IKSolver(coxa_len=0.05, femur_len=0.15, tibia_len=0.25)
        
        self.timer_period = 0.05 # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.vel_x = 0.0
        self.vel_yaw = 0.0
        
      
        self.phase = 0.0
        self.gait_speed = 4.0 # Lowered for stability
        

        self.body_length = 0.50
        self.body_width = 0.80
        self.body_radius = 0.15 # Legacy radius used in any relative calc
     
        
        self.default_z = -0.28 # Slightly higher than floor (0.30) to avoid IK limits
        self.last_joint_positions = [0.0] * 18
        self.first_run = True
        
        # Adaptive gait parameters
        self.obstacle_ahead = False
        self.obstacle_height = 0.0
        self.base_step_height = 0.05  # Normal step height
        self.elevated_step_height = 0.15  # Step height when climbing
        self.current_step_height = self.base_step_height
        self.step_height_transition_rate = 0.02  # Smooth transition
        
        # Climbable obstacle thresholds
        self.min_climbable_height = 0.1  # Below this, treat as ground
        self.max_climbable_height = 0.25  # Above this, avoid/go around
        self.obstacle_detect_range = 0.8  # meters ahead to check
        
        # 3D terrain tracking
        self.terrain_max_height = 0.0
        
    def terrain_height_callback(self, msg):
        """Update max terrain height detected by point cloud analyzer"""
        self.terrain_max_height = msg.data
        
    def scan_callback(self, msg):
        """Analyze laser scan to detect climbable obstacles ahead"""
        if len(msg.ranges) == 0:
            return
        
        # Check forward arc (±30 degrees)
        num_readings = len(msg.ranges)
        center_idx = num_readings // 2
        arc_width = int(num_readings * 0.15)  # 30 degrees total
        
        start_idx = max(0, center_idx - arc_width)
        end_idx = min(num_readings, center_idx + arc_width)
        
        forward_ranges = []
        for i in range(start_idx, end_idx):
            if msg.range_min < msg.ranges[i] < msg.range_max:
                forward_ranges.append(msg.ranges[i])
        
        if not forward_ranges:
            self.obstacle_ahead = False
            return
        
        # Find closest obstacle in forward arc
        min_distance = min(forward_ranges)
        
        # If obstacle is close and within climbable range
        if min_distance < self.obstacle_detect_range:
            # Estimate height from lidar plane (at ~0.15m) detecting obstacle
            # If obstacle is detected by lidar at 0.15m height, it's at least that tall
            # Use simple heuristic: closer objects that block lidar are likely low obstacles
            estimated_height = 0.15  # Lidar is at this height
            
            if self.min_climbable_height < estimated_height < self.max_climbable_height:
                self.obstacle_ahead = True
                self.obstacle_height = estimated_height
            else:
                self.obstacle_ahead = False
        else:
            self.obstacle_ahead = False
    
    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_yaw = msg.angular.z
        
    def timer_callback(self):
        if self.first_run:
            self.get_logger().info(f"First Heartbeat: Standing up at z={self.default_z}")
            self.first_run = False
            
        # Adaptively adjust step height based on 3D terrain and laser scan
        # Prioritize 3D terrain from point cloud
        if self.terrain_max_height > self.min_climbable_height:
             target_step_height = min(self.elevated_step_height, self.terrain_max_height + 0.05) # Lift slightly above obstacle
             self.obstacle_ahead = True
        elif self.obstacle_ahead: # Fallback/Latcher if laser detected something but PC missed (less likely now)
             target_step_height = self.elevated_step_height
        else:
             target_step_height = self.base_step_height
             self.obstacle_ahead = False
        
        # Smooth transition
        if self.current_step_height < target_step_height:
            self.current_step_height = min(self.current_step_height + self.step_height_transition_rate, target_step_height)
        elif self.current_step_height > target_step_height:
            self.current_step_height = max(self.current_step_height - self.step_height_transition_rate, target_step_height)
        
        if self.obstacle_ahead:
            self.get_logger().info(
                f"Obstacle detected! Elevated step: {self.current_step_height:.3f}m",
                throttle_duration_sec=2.0
            )
            
        msg = Float64MultiArray()
        
        # Increment phase
        if abs(self.vel_x) > 0.001 or abs(self.vel_yaw) > 0.001:
            self.phase += self.gait_speed * self.timer_period
        
        amplitude_x = 0.05 * self.vel_x
        amplitude_z = self.current_step_height # Use adaptive step height
        joint_positions = []
        

        
        # 6 Legs Arrangement
        legs = ['rf', 'rm', 'rr', 'lf', 'lm', 'lr']
        leg_yaws = [-0.7853, -1.5708, -2.3561, 0.7853, 1.5708, 2.3561]
        
        # Tripod gait: groups of 3 legs
        # Group 0: RF, RR, LM
        # Group 1: LF, LR, RM
        tripod_groups = [0, 1, 0, 1, 0, 1] 
        
        for i, leg in enumerate(legs):
            group_offset = math.pi if tripod_groups[i] == 1 else 0.0
            current_phase = self.phase + group_offset
            
            speed_linear = self.vel_x
            speed_turn = self.vel_yaw
            
            if abs(speed_linear) < 0.001 and abs(speed_turn) < 0.001:
                 # Default Standing Pose
                 x_val = 0.25 
                 y_val = 0.0
                 z_val = self.default_z # -0.15
                 
                 t1, t2, t3 = self.ik.solve(x_val, y_val, z_val)
                 
                 # Clamping to URDF limits
                 t1_cmd = max(-0.7, min(0.7, t1))
                 t2_cmd = max(-1.5, min(1.5, t2))
                 t3_cmd = max(-2.5, min(0.5, t3))
                 
                 joint_positions.extend([t1_cmd, t2_cmd, t3_cmd])
                 continue

            stride_amp = 0.07 
            cycle_val = math.cos(current_phase)
            
            cos_yaw = math.cos(leg_yaws[i])
            sin_yaw = math.sin(leg_yaws[i])
            
            input_vx = -self.vel_x * stride_amp
            input_vy = 0.0 
            
            local_vx = input_vx * cos_yaw + input_vy * sin_yaw
            local_vy = -input_vx * sin_yaw + input_vy * cos_yaw
            
            x_off = local_vx * cycle_val
            y_off = local_vy * cycle_val
            
            turn_off = -self.vel_yaw * 0.1 * cycle_val 
            y_off += turn_off

            z_val = self.default_z # Stance height
            if math.sin(current_phase) > 0:
                 # Lift leg using adaptive step height
                 z_val += self.current_step_height * math.sin(current_phase)

            x_target = 0.25 + x_off
            y_target = y_off
            
            t1, t2, t3 = self.ik.solve(x_target, y_target, z_val)
            
            # If IK fails, use neutral pose instead of jumping to (0,0,0)
            if t1 == 0.0 and t2 == 0.0 and t3 == 0.0:
                # Re-solve for default standing pose for this leg
                t1, t2, t3 = self.ik.solve(0.25, 0.0, self.default_z)

            # Clamping to URDF limits
            t1_cmd = max(-0.7, min(0.7, t1))
            t2_cmd = max(-1.5, min(1.5, t2))
            t3_cmd = max(-2.5, min(0.5, t3))
            
            joint_positions.extend([t1_cmd, t2_cmd, t3_cmd])

        self.last_joint_positions = joint_positions

        # msg.data = joint_positions
        # self.publisher_.publish(msg)
        # # Print debug angles for the first leg (RF)
        # print(f"DEBUG: RF Leg Angles: Coxa={joint_positions[0]:.2f}, Femur={joint_positions[1]:.2f}, Tibia={joint_positions[2]:.2f}")

        msg.data = joint_positions
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gait_controller = GaitController()
    rclpy.spin(gait_controller)
    gait_controller.destroy_node()
    rclpy.shutdown()

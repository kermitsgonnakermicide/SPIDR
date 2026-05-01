import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
import math
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

        self.ik = IKSolver(coxa_len=0.043, femur_len=0.060, tibia_len=0.104)
        
        self.timer_period = 0.05 # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.vel_x = 0.0
        self.vel_yaw = 0.0
        
        self.phase = 0.0
        self.gait_speed = 4.0 

        self.body_length = 0.167
        self.body_width = 0.126
        
        self.default_z = -0.12 
        self.first_run = True
        
        # Adaptive gait parameters
        self.obstacle_ahead = False
        self.base_step_height = 0.05  
        self.elevated_step_height = 0.18  
        self.current_step_height = self.base_step_height
        self.step_height_transition_rate = 0.02  
        
        self.min_climbable_height = 0.1  
        
        # 3D terrain tracking
        self.terrain_max_height = 0.0
        self.current_body_lift = 0.0
        self.target_body_lift = 0.0
        self.body_lift_smooth_rate = 0.01  
        
    def terrain_height_callback(self, msg):
        self.terrain_max_height = msg.data
        
    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_yaw = msg.angular.z
        
    def timer_callback(self):
        if self.first_run:
            self.get_logger().info(f"First Heartbeat: Standing up at z={self.default_z}")
            self.first_run = False
            
        # Adaptively adjust step height AND body height based on 3D terrain
        if self.terrain_max_height > self.min_climbable_height:
             target_step_height = min(0.18, self.terrain_max_height + 0.06)
             self.target_body_lift = min(0.06, self.terrain_max_height * 0.25)
             self.obstacle_ahead = True
        else:
             target_step_height = self.base_step_height
             self.target_body_lift = 0.0
             self.obstacle_ahead = False
        
        # Smooth transitions
        if self.current_step_height < target_step_height:
            self.current_step_height = min(self.current_step_height + self.step_height_transition_rate, target_step_height)
        elif self.current_step_height > target_step_height:
            self.current_step_height = max(self.current_step_height - self.step_height_transition_rate, target_step_height)
            
        if self.current_body_lift < self.target_body_lift:
            self.current_body_lift = min(self.current_body_lift + self.body_lift_smooth_rate, self.target_body_lift)
        elif self.current_body_lift > self.target_body_lift:
            self.current_body_lift = max(self.current_body_lift - self.body_lift_smooth_rate, self.target_body_lift)
        
        msg = Float64MultiArray()
        
        # Increment phase
        if abs(self.vel_x) > 0.001 or abs(self.vel_yaw) > 0.001:
            self.phase += self.gait_speed * self.timer_period
        
        joint_positions = []
        
        legs = ['rf', 'rm', 'rr', 'lf', 'lm', 'lr']
        leg_yaws = [-0.7853, -1.5708, -2.3561, 0.7853, 1.5708, 2.3561]
        tripod_groups = [0, 1, 0, 1, 0, 1] 
        
        for i, leg in enumerate(legs):
            group_offset = math.pi if tripod_groups[i] == 1 else 0.0
            current_phase = self.phase + group_offset
            
            if abs(self.vel_x) < 0.001 and abs(self.vel_yaw) < 0.001:
                 # Default Standing Pose - Adapt to body lift
                 x_val = 0.12 
                 y_val = 0.0
                 z_val = self.default_z - self.current_body_lift 
                 t1, t2, t3 = self.ik.solve(x_val, y_val, z_val)
                 joint_positions.extend([max(-0.7, min(0.7, t1)), max(-1.5, min(1.5, t2)), max(-2.5, min(0.5, t3))])
                 continue

            stride_amp = 0.07 
            cycle_val = math.cos(current_phase)
            cos_yaw = math.cos(leg_yaws[i])
            sin_yaw = math.sin(leg_yaws[i])
            
            input_vx = -self.vel_x * stride_amp
            local_vx = input_vx * cos_yaw
            local_vy = -input_vx * sin_yaw
            
            x_off = local_vx * cycle_val
            y_off = local_vy * cycle_val
            y_off += -self.vel_yaw * 0.1 * cycle_val 

            z_val = self.default_z - self.current_body_lift 
            if math.sin(current_phase) > 0:
                 z_val += self.current_step_height * math.sin(current_phase)

            x_target = 0.12 + x_off
            y_target = y_off
            
            t1, t2, t3 = self.ik.solve(x_target, y_target, z_val)
            if t1 == 0.0 and t2 == 0.0 and t3 == 0.0:
                t1, t2, t3 = self.ik.solve(0.12, 0.0, self.default_z)

            joint_positions.extend([max(-0.7, min(0.7, t1)), max(-1.5, min(1.5, t2)), max(-2.5, min(0.5, t3))])

        msg.data = joint_positions
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gait_controller = GaitController()
    rclpy.spin(gait_controller)
    gait_controller.destroy_node()
    rclpy.shutdown()

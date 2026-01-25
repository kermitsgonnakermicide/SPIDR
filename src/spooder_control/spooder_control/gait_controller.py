import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math
import time
from spooder_control.ik_solver import IKSolver

class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/spooder_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("Gait Controller Initialized. Starting control loop...")

        
        self.ik = IKSolver(coxa_len=0.05, femur_len=0.15, tibia_len=0.25)
        
        self.timer_period = 0.05 # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.vel_x = 0.0
        self.vel_yaw = 0.0
        
      
        self.phase = 0.0
        self.gait_speed = 6.0 
        

        self.body_length = 0.50
        self.body_width = 0.80
        self.body_radius = 0.15 # Legacy radius used in any relative calc
     
        
        self.default_z = -0.20 # Increased from -0.15 to ensure feet reach the floor
        self.first_run = True
        
    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_yaw = msg.angular.z
        
    def timer_callback(self):
        if self.first_run:
            self.get_logger().info(f"First Heartbeat: Standing up at z={self.default_z}")
            self.first_run = False
            
        msg = Float64MultiArray()
        
        # Increment phase
        if abs(self.vel_x) > 0.001 or abs(self.vel_yaw) > 0.001:
            self.phase += self.gait_speed * self.timer_period
        
   
        
        amplitude_x = 0.05 * self.vel_x
        amplitude_z = 0.05 
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
                 z_val += 0.05 * math.sin(current_phase)

            x_target = 0.25 + x_off
            y_target = y_off
            
            t1, t2, t3 = self.ik.solve(x_target, y_target, z_val)
            
            # URDF specific adjustments:
            # Our URDF is now set such that positive femur rotation is DOWN.
            # My IK solver returns positive theta2 for downward extension.
            # Positive tibia rotation is UP, but we want it to bend IN (negative).
            # So we keep t1, t2 as is, and keep t3 negative.
            
            # Clamping to URDF limits
            t1_cmd = max(-0.7, min(0.7, t1))
            t2_cmd = max(-1.5, min(1.5, t2))
            t3_cmd = max(-2.5, min(0.5, t3))
            
            joint_positions.extend([t1_cmd, t2_cmd, t3_cmd])

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

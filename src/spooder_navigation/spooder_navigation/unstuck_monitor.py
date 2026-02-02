#!/usr/bin/env python3
"""
Unstuck Monitor for SPIDR
Detects stuck conditions and triggers progressive recovery behaviors
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import Spin, BackUp
import numpy as np
from collections import deque
import math


class UnstuckMonitor(Node):
    def __init__(self):
        super().__init__('unstuck_monitor')
        
        # Parameters (aggressive tuning for dynamic environments)
        self.declare_parameter('stuck_timeout', 4.0)  # Reduced from 8.0 for faster response
        self.declare_parameter('min_velocity_threshold', 0.015)  # Slightly higher threshold
        self.declare_parameter('oscillation_radius', 0.2)  # Wider to detect more oscillation
        self.declare_parameter('recovery_timeout', 30.0)
        self.declare_parameter('position_history_size', 50)
        
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.min_vel_threshold = self.get_parameter('min_velocity_threshold').value
        self.oscillation_radius = self.get_parameter('oscillation_radius').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.history_size = self.get_parameter('position_history_size').value
        
        # State tracking
        self.position_history = deque(maxlen=self.history_size)
        self.cmd_vel_history = deque(maxlen=20)
        self.last_movement_time = self.get_clock().now()
        self.last_position = None
        self.current_velocity = None
        self.is_recovering = False
        self.recovery_level = 0
        self.stuckness_score = 0.0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/spooder/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(
            String,
            '/unstuck_monitor/status',
            10
        )
        
        # Action clients for recovery
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.backup_client = ActionClient(self, BackUp, '/backup')
        
        # Timer for stuck detection (check more frequently)
        self.check_timer = self.create_timer(0.5, self.check_stuck)  # Check every 0.5s instead of 1.0s
        
        self.get_logger().info('Unstuck Monitor initialized')
        self.get_logger().info(f'  Stuck timeout: {self.stuck_timeout}s')
        self.get_logger().info(f'  Min velocity: {self.min_vel_threshold} m/s')
        self.get_logger().info(f'  Oscillation radius: {self.oscillation_radius}m')
    
    def odom_callback(self, msg):
        """Track position and velocity"""
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
        
        self.current_velocity = np.linalg.norm(velocity)
        
        # Update position history
        if self.last_position is not None:
            movement = np.linalg.norm(position - self.last_position)
            if movement > 0.03:  # Reduced threshold for more sensitive detection
                self.last_movement_time = self.get_clock().now()
        
        self.last_position = position
        self.position_history.append(position)
    
    def cmd_vel_callback(self, msg):
        """Track commanded velocities"""
        cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.cmd_vel_history.append(cmd_vel)
    
    def check_stuck(self):
        """Check if robot is stuck and trigger recovery"""
        if self.is_recovering:
            return
        
        if len(self.position_history) < 10:
            return  # Not enough data yet
        
        # Calculate stuckness score
        self.stuckness_score = self.calculate_stuckness()
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Stuck Score: {self.stuckness_score:.2f}, Level: {self.recovery_level}'
        self.status_pub.publish(status_msg)
        
        # Trigger recovery if stuck (lower threshold for faster response)
        if self.stuckness_score > 0.25:  # Reduced from 0.3
            self.get_logger().warn(f'STUCK DETECTED! Score: {self.stuckness_score:.2f}')
            self.trigger_recovery()
    
    def calculate_stuckness(self):
        """Calculate how stuck the robot is (0.0 = free, 1.0 = completely stuck)"""
        score = 0.0
        
        # Factor 1: Time since last movement
        time_since_movement = (self.get_clock().now() - self.last_movement_time).nanoseconds / 1e9
        if time_since_movement > self.stuck_timeout:
            score += 0.4
        elif time_since_movement > self.stuck_timeout * 0.5:
            score += 0.2
        
        # Factor 2: Oscillation detection
        if len(self.position_history) >= 10:
            recent_positions = np.array(list(self.position_history)[-10:])
            centroid = np.mean(recent_positions, axis=0)
            max_dist = np.max(np.linalg.norm(recent_positions - centroid, axis=1))
            
            if max_dist < self.oscillation_radius:
                score += 0.3
        
        # Factor 3: Commanded vs actual velocity mismatch
        if self.current_velocity is not None and len(self.cmd_vel_history) > 0:
            recent_cmds = np.array(list(self.cmd_vel_history)[-5:])
            avg_cmd_vel = np.linalg.norm(recent_cmds[:, :2], axis=1).mean()
            
            if avg_cmd_vel > 0.05 and self.current_velocity < self.min_vel_threshold:
                score += 0.3
        
        return min(score, 1.0)
    
    def trigger_recovery(self):
        """Trigger appropriate recovery behavior based on stuckness level"""
        self.is_recovering = True
        
        # Determine recovery level (adjusted thresholds)
        if self.stuckness_score >= 0.6:  # Lower threshold
            self.recovery_level = 3  # Emergency
        elif self.stuckness_score >= 0.4:  # Lower threshold
            self.recovery_level = 2  # Aggressive
        else:
            self.recovery_level = 1  # Moderate
        
        self.get_logger().info(f'Triggering Level {self.recovery_level} recovery')
        
        if self.recovery_level == 1:
            self.execute_moderate_recovery()
        elif self.recovery_level == 2:
            self.execute_aggressive_recovery()
        else:
            self.execute_emergency_recovery()
    
    def execute_moderate_recovery(self):
        """Level 1: Moderate backup + random spin"""
        self.get_logger().info('Executing MODERATE recovery')
        
        # Backup 0.5m (increased from 0.3m)
        backup_goal = BackUp.Goal()
        backup_goal.target.x = -0.5
        backup_goal.speed = 0.15  # Faster backup
        
        self.backup_client.wait_for_server()
        future = self.backup_client.send_goal_async(backup_goal)
        future.add_done_callback(self.on_moderate_backup_done)
    
    def on_moderate_backup_done(self, future):
        """After moderate backup, do larger random spin"""
        # Random spin 90-180 degrees (increased from 60-120)
        angle = np.random.uniform(1.57, 3.14)  # radians
        
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = angle
        
        self.spin_client.wait_for_server()
        spin_future = self.spin_client.send_goal_async(spin_goal)
        spin_future.add_done_callback(self.on_recovery_complete)
    
    def execute_aggressive_recovery(self):
        """Level 2: Aggressive backup + large random spin"""
        self.get_logger().warn('Executing AGGRESSIVE recovery')
        
        # Backup 0.8m (increased from 0.5m)
        backup_goal = BackUp.Goal()
        backup_goal.target.x = -0.8
        backup_goal.speed = 0.2  # Much faster backup
        
        self.backup_client.wait_for_server()
        future = self.backup_client.send_goal_async(backup_goal)
        future.add_done_callback(self.on_aggressive_backup_done)
    
    def on_aggressive_backup_done(self, future):
        """After aggressive backup, do full rotation"""
        # Random spin 180-270 degrees (increased)
        angle = np.random.uniform(3.14, 4.71)  # radians
        
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = angle
        
        self.spin_client.wait_for_server()
        spin_future = self.spin_client.send_goal_async(spin_goal)
        spin_future.add_done_callback(self.on_recovery_complete)
    
    def execute_emergency_recovery(self):
        """Level 3: Maximum backup + full rotation"""
        self.get_logger().error('Executing EMERGENCY recovery!')
        
        # Backup 1.2m (increased from 0.8m)
        backup_goal = BackUp.Goal()
        backup_goal.target.x = -1.2
        backup_goal.speed = 0.25  # Very fast backup
        
        self.backup_client.wait_for_server()
        future = self.backup_client.send_goal_async(backup_goal)
        future.add_done_callback(self.on_emergency_backup_done)
    
    def on_emergency_backup_done(self, future):
        """After emergency backup, do full 360 rotation"""
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = 6.28  # Full circle
        
        self.spin_client.wait_for_server()
        spin_future = self.spin_client.send_goal_async(spin_goal)
        spin_future.add_done_callback(self.on_recovery_complete)
    
    def on_recovery_complete(self, future):
        """Recovery sequence complete"""
        self.get_logger().info('Recovery sequence complete')
        self.is_recovering = False
        self.recovery_level = 0
        self.stuckness_score = 0.0
        self.last_movement_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = UnstuckMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

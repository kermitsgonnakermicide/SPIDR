import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.forward_speed = 0.1  # Slow forward motion
        self.turn_speed = 0.3
        self.obstacle_distance = 0.8  # Stop if obstacle within 0.8m
        
        self.current_state = 'forward'
        self.turn_duration = 0
        self.turn_counter = 0
        
        self.min_front_distance = float('inf')
        
        self.get_logger().info('Simple Explorer initialized')
    
    def scan_callback(self, msg):
        # Check front 60 degrees for obstacles
        ranges = msg.ranges
        num_readings = len(ranges)
        
        # Front 60 degrees (±30 degrees)
        front_start = int(num_readings * 0.42)  # -30 degrees
        front_end = int(num_readings * 0.58)    # +30 degrees
        
        front_ranges = [r for r in ranges[front_start:front_end] if r > 0.1 and r < msg.range_max]
        
        if front_ranges:
            self.min_front_distance = min(front_ranges)
        else:
            self.min_front_distance = msg.range_max
    
    def timer_callback(self):
        msg = Twist()
        
        if self.current_state == 'forward':
            if self.min_front_distance < self.obstacle_distance:
                # Obstacle detected, switch to turning
                self.current_state = 'turning'
                self.turn_duration = random.randint(15, 30)  # Turn for 1.5-3 seconds
                self.turn_counter = 0
                self.get_logger().info(f'Obstacle at {self.min_front_distance:.2f}m, turning...')
            else:
                # Move forward
                msg.linear.x = self.forward_speed
                msg.angular.z = 0.0
        
        elif self.current_state == 'turning':
            self.turn_counter += 1
            
            if self.turn_counter >= self.turn_duration:
                # Done turning, go forward
                self.current_state = 'forward'
                self.get_logger().info('Resuming forward motion')
            else:
                # Turn in place
                msg.linear.x = 0.0
                msg.angular.z = self.turn_speed if random.random() > 0.5 else -self.turn_speed
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    explorer = SimpleExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

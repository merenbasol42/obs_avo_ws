import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Lidar verileri için abone
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10
        )
        
        # Robot hareket komutları için yayıncı
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Engel parametreleri
        self.min_obstacle_distance = 0.5  # 50 cm
        self.obstacle_angle_range = 30  # derece
        
        # Hareket parametreleri
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Durum makinesi değişkenleri
        self.current_state = 'MOVING_FORWARD'
        
    def lidar_callback(self, msg):
        """
        Lidar verilerini işleyen ve engel kaçınma kararı veren fonksiyon
        """
        ranges = np.array(msg.ranges)
        
        # Ön taraftaki aralığı kontrol et
        front_start = int((0 - self.obstacle_angle_range/2) * len(ranges) / 360)
        front_end = int((0 + self.obstacle_angle_range/2) * len(ranges) / 360)
        
        front_ranges = ranges[front_start:front_end]
        min_distance = np.min(front_ranges)
        
        # Durum makinesi
        if self.current_state == 'MOVING_FORWARD':
            if min_distance < self.min_obstacle_distance:
                self.stop_robot()
                self.current_state = 'AVOIDING_OBSTACLE'
            else:
                self.move_forward()
        
        elif self.current_state == 'AVOIDING_OBSTACLE':
            # Rastgele sağa veya sola dönme
            turn_direction = random.choice([-1, 1])
            self.turn_robot(turn_direction)
            self.current_state = 'TURNING'
        
        elif self.current_state == 'TURNING':
            # Dönüş tamamlandıktan sonra tekrar ileri hareket
            self.move_forward()
            self.current_state = 'MOVING_FORWARD'
    
    def move_forward(self):
        """Robot ileri hareket ettirir"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('İleri hareket')
    
    def stop_robot(self):
        """Robotu durdurur"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot durdu')
    
    def turn_robot(self, direction):
        """Robotu belirtilen yönde döndürür"""
        twist = Twist()
        twist.angular.z = direction * self.angular_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Dönüş: {"Sağa" if direction > 0 else "Sola"}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

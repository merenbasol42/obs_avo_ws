import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Lidar verilerini dinleyen abone
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10  # QoS profili
        )
        
        # Engel tespiti için parametreler
        self.min_obstacle_distance = 0.5  # 50 cm
        self.obstacle_angle_range = 30  # derece
        
    def lidar_callback(self, msg):
        """
        Lidar verilerini işleyen geri çağırma fonksiyonu
        Engelleri tespit eder ve bilgileri yayınlar
        """
        # Lidar verilerini numpy dizisine dönüştür
        ranges = np.array(msg.ranges)
        
        # Boş veya geçersiz veri kontrolü
        if len(ranges) == 0:
            self.get_logger().warn('Lidar verisi boş!')
            return
        
        # Ön taraftaki aralığı kontrol et
        front_start = max(0, int((0 - self.obstacle_angle_range/2) * len(ranges) / 360))
        front_end = min(len(ranges), int((0 + self.obstacle_angle_range/2) * len(ranges) / 360))
        
        # Ön taraftaki aralığı al
        front_ranges = ranges[front_start:front_end]
        
        # Boş ön aralık kontrolü
        if len(front_ranges) == 0:
            self.get_logger().warn('Ön taraftaki Lidar aralığı boş!')
            return
        
        # Geçerli mesafeleri filtrele (sonsuz ve sıfır değerlerini çıkar)
        valid_ranges = front_ranges[np.isfinite(front_ranges) & (front_ranges > 0)]
        
        # Geçerli mesafe yoksa çık
        if len(valid_ranges) == 0:
            self.get_logger().warn('Geçerli Lidar mesafesi bulunamadı!')
            return
        
        # En yakın engeli bul
        min_distance = np.min(valid_ranges)
        
        if min_distance < self.min_obstacle_distance:
            self.get_logger().warn(f'Engel tespit edildi! Mesafe: {min_distance:.2f} m')
        else:
            self.get_logger().info('Önde engel yok')

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

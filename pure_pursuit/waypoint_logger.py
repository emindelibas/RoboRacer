#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # Rotayı kaydedeceğimiz dosya.
        self.file_name = 'levine_rotasi.csv'
        self.file = open(self.file_name, 'w')
        
        # Simülatörden (veya arabadan) gelen kesin konumu dinliyoruz
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.get_logger().info(f'Kayıt basladi! Araci manuel olarak surun. Cikmak icin Ctrl+C yapin.')

    def odom_callback(self, msg):
        # Arabanın anlık X ve Y konumlarını al
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Virgülle ayırarak CSV dosyasına alt alta yaz (Örn: 1.5, -2.3)
        self.file.write(f"{x},{y}\n")

def main(args=None):
    rclpy.init(args=args)
    logger = WaypointLogger()
    
    try:
        rclpy.spin(logger) # Araba hareket ettikçe kaydetmeye devam et
    except KeyboardInterrupt:
        # Sen Ctrl+C'ye bastığında döngüden çık
        pass
    finally:
        # Dosyayı güvenli bir şekilde kapat ve bitir
        logger.get_logger().info(f'Kayit tamamlandi! Rota "{logger.file_name}" dosyasina kaydedildi.')
        logger.file.close()
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

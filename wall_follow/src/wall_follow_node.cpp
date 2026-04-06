#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath> // Matematik işlemleri (sin, cos, atan, isinf, isnan) için eklendi
#include <algorithm> // std::clamp için eklendi

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
            
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

    }

private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    double kp = 5.0; //oransal kazanç
    // TODO: double kd =
    double kd = 0.009;//derivative kazanç
    // TODO: double ki =
    double ki= 0.0005;//integral kazanç

    //double servo_offset = 0.0;
    
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Hedef değerler
    double desired_distance = 1.0; // Sağ duvara 1.0 metre mesafeden git
    double lookahead_dist = 0.8;   // Aracın L (İleri bakış) mesafesi

    // Lidar özellikleri (Callback içinde güncellenecek)
    double angle_min_ = 0.0;
    double angle_increment_ = 0.0;
    int ranges_size_ = 0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    // Subscriber ve Publisher objeleri
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
        
        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // Açıyı dizi indeksine çevir
        int index = std::round((angle - angle_min_) / angle_increment_);
        // round: yuvarlama yapma için kullanılır
        
        // İndeks sınırlarını kontrol et
        if (index < 0 || index >= ranges_size_) {
            return 0.0; //get_range'i sıfırlar
        }

        double range = range_data[index];
        
        // Sonsuz veya geçersiz (NaN) değerleri filtrele
        if (std::isnan(range) || std::isinf(range)) {
            return 0.0;
        }
        
        return range;
        // TODO: implement
        
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // sağ duvarı takip edeceğimiz için negatif açılar kullanıyoruz.
        // b: Tam sağ (-90 derece = -pi/2 radyan)
        // a: Sağda biraz daha ileri bakan ışın (örn: 45 derece = -pi/4 radyan)
        double theta = M_PI / 4.0; // 45 derece radyan cinsinden
        
        double a = get_range(range_data, - M_PI / 2.0 + theta); // 45 derecedeki ışın
        double b = get_range(range_data, - M_PI / 2.0);         // 90 derecedeki ışın (tam sol)

        // Eğer lidar buralarda duvar görmediyse, ufak bir kontrol
        if (a == 0.0 || b == 0.0) return 0.0;

        // Alfa hesaplama: Aracın duvara göre yönelimi
        double alpha = std::atan((a * std::cos(theta) - b) / (a * std::sin(theta)));

        // Mevcut mesafe (Dt) ve gelecekteki mesafe (Dt+1)
        double current_dist = b * std::cos(alpha);
        double future_dist = current_dist + lookahead_dist * std::sin(alpha);// - olabilir

        // Hata (Error) hesaplama
        return dist - future_dist;


        // TODO:implement
        //return 0.0;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        //double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller

        // PID Denklemi
        integral += error;
        double derivative = error - prev_error;

        // Direksiyon açısı hesaplama
        // Not: Sol duvarda hata pozitifse (duvara çok yakınsak), sağa kırmalıyız (negatif açı).
        // Bu yüzden formülün başına eksi (-) koyuyoruz.
        double steering_angle =  (kp * error + kd * derivative + ki * integral);

        // Direksiyon açısını F1TENTH sınırlarına (genelde -0.34 ile +0.34 radyan) kısıtla
        steering_angle = std::clamp(steering_angle, -0.34, 0.34);

        prev_error = error;


        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;
        
        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        //double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

        // Lidar metadatalarını kaydet (get_range içinde indeks hesabı için gerekli)
        angle_min_ = scan_msg->angle_min;
        angle_increment_ = scan_msg->angle_increment;
        ranges_size_ = scan_msg->ranges.size();

        // 1. Hatayı hesapla
        // ranges vektörünün içindeki verinin pointer'ını (float*) geçiyoruz.
        double current_error = get_error(const_cast<float*>(scan_msg->ranges.data()), desired_distance);

        // 2. Hız profilini belirle (Direksiyon açısına veya hataya göre dinamik hız)
        
        // Hata küçükse hızlı git, hata büyükse (viraj dönüyorsa) yavaşla
        if (std::abs(current_error) < 0.1) {
            velocity = 1.5;
        } else if (std::abs(current_error) < 0.3) {
            velocity = 1.0;
        } else {
            velocity = 0.5;
        }

        // 3. Aracı hareket ettir
        pid_control(current_error, velocity);

    }

    
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
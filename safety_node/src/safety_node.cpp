#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;    //Callback fonksiyonlarına parametre gönderirken kullandığımız kısa yol

class Safety : public rclcpp::Node { // safety adında kendi node'umuzu oluşturduk   
// The class that handles emergency braking

public:

    double speed2 = 1.00;
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers

        sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Safety::scan_callback, this, _1));

        sub_odom_= this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));


        pub_drive= this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        
    }

private:
    
    /// TODO: create ROS subscribers and publishers
    double speed = 0.0;
    // const olası yanlışlıkla yapılacak değişiklikleri önlemek için var, veri sadece okunabilir.
    // mesela msg->twist.twist.linear.x = 50; yazarsam derleyici hata verir
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) 
    {
        //ConstSharedPtr: hafızadaki adresini akıllı bir işaretçi (SharedPtr) ile yollar. Bu veriye başka nodelar da bakabilir. içindeki veri değiştirilmez
        // iş bittiğinde sharede pointer bu veriyi güvenle siler
        /// TODO: update current speed
        
        this->speed = msg->twist.twist.linear.x;
        
        
        
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        //LiDAR sensörünün "Ben etrafı taradım, sonuçlar bunlar" diyerek scan_callback fonksiyonuna gönderdiği veri paketidir (Pointer olarak gelir)

        // Threshold for Time-To-Collision (in seconds)
        double ttc_threshold = 0.5; 
        bool emergency_brake = false;
        //double fov_limit = 15.0 * M_PI / 180.0;
        double car_half_width = 0.2; //araba yarı genişliğini 20cm varsaydım

        //double son_mesafe;
        /// TODO: calculate TTC
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            double range = scan_msg->ranges[i];
            
            // Ignore NaN or out of bounds measurements
            //Geçersiz (NaN) veya sensörün okuma menzilinin dışındaki değerleri continue diyerek es geçer
            if (std::isnan(range) || range > scan_msg->range_max || range < scan_msg->range_min) {
                continue;
            }
            
            // Calculate the angle of the current laser beam
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            
        
            // 2. KORUMA: Engelin yanal (Y ekseni) mesafesini hesaplar    
            double y_distance = range * std::sin(angle);
            if (std::abs(y_distance) > car_half_width) {
                continue; 
            }
            
            
            
            // Calculate the rate of closure (range rate)
            // r_dot = v * cos(theta)
            // aracın hızının o engele doğru olan bileşeni
            double range_rate = - this->speed * std::cos(angle);


            /*
            if (std::abs(angle) > fov_limit){
                continue;
            }
            */

            // Calculate instantaneous Time to Collision (iTTC)

            //range rate 0dan küçükse hesaplama yapar. araba engele yaklaşıyor demektir
            if (range_rate < 0) {
                double iTTC = range / - range_rate;
                
                // Check if iTTC falls below the safety threshold
                if (iTTC < ttc_threshold) {
                    emergency_brake = true;
                    RCLCPP_INFO_STREAM(this->get_logger(),"iTTC verimiz    " << iTTC);
                    break; // Brake immediately, no need to check remaining beams
                }
            }
        }

        /// TODO: publish drive/brake message

        if (emergency_brake) {
            // Create the brake message using AckermannDriveStamped
            ackermann_msgs::msg::AckermannDriveStamped brake_msg;
            brake_msg.drive.speed = 0.0;
            
            // Publish the brake command using pub_drive
            this->pub_drive->publish(brake_msg);

        }

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv); //ros sistemini başlatır
    rclcpp::spin(std::make_shared<Safety>()); //Safety düğümünü ayağa kaldırır, sonsuz döngüye sokar. scan ve odom dinlmeye devam etmesi için
    rclcpp::shutdown();
    return 0;
}
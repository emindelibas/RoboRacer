#include "rclcpp/rclcpp.hpp"//ROS2 nin C++ için temel kütüphanesi
#include <chrono> //zaman ve süre ile ilgili işlemler için(ms cinsinden bekleme sğreleri gibi) 
#include <memory> //smart pointers(std:shared_ptr gibi) kullanmak için gerekli. Ros2 bellek yönetimini güvenli yapmak için akıllı işaretçileri yoğun bir şekilde kullanır
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // F1TENT simülatörüne göndereceğimiz mesajın tipini tanımlayan başlık dosyası
using namespace std::chrono_literals; //zaman birimlerini uzun uzun değilde kısa yazmamamıza olanak sağlar. ms, s, h gibi
using std::placeholders::_1; //Gelen mesajın (verinin), drive_callback fonksiyonuna ilk parametre olarak eksiksiz 
//bir şekilde iletilmesini sağlayan bir C++ özelliğidir.

class Relay : public rclcpp::Node {

public:
    Relay() : Node("relay"){

        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10, std::bind(&Relay::drive_callback, this, _1)); // dinleyeceğimiz topic /drive

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_relay", 10);
        //drive_relay: mesajların yayınlanacağı yeni topicin adı
    }

private:

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const { //drive topicinden her yeni mesaj geldiğinde tetiklenen bir fonksiyon
        //SharedPtr: gelen mesajın bellekteki yerini işaret eden akıllı bir pointer.
        //içinde Talker node'undan hız ve açı verileri bulunur
        
        // sondaki const: Sınıfın durumunu korur. Eğer yanlışlıkla drive_callback fonksiyonunun içine this->publisher_ = nullptr; gibi sınıfın temelini 
        //bozacak bir kod yazarsam, sondaki bu const sayesinde derleyici anında "Dur! Sen bu değişkenleri değiştirmeyeceğine söz vermiştin!" diyerek kodu 
        //derlemeyi reddeder ve olası bir bug'dan kurtarır.
        
        auto new_msg= ackermann_msgs::msg::AckermannDriveStamped(); //drive_relay topicine göndereceğim tamamen yeni ve boş mesaj nesnesi

        new_msg.drive.speed = msg->drive.speed * 3.0;
        new_msg.drive.steering_angle = msg->drive.steering_angle * 3;
        //msg bir pointer olduğu için "->" işareti kullanılır

        publisher_->publish(new_msg);

        //Bir callback (geri çağırma) fonksiyonunun görevi sadece gelen veriyi alıp okumak ve işlem yapmaktır. Sınıfın altyapısını veya ayarlarını değiştirmemesi 
        //gerektiği için sonuna const koymak en güvenli (ve ROS 2'de en çok tavsiye edilen) C++ yazım kuralıdır.

    }

rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
//class'ın döngüsü boyunca bellekte tutlması gereken sub ve pub nesnelerinin tanımlandığı yer.

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv); //ros2 sistemini başlatır
    rclcpp::spin(std::make_shared<Relay>()); //Talker düğümünü ayağa kaldırır, sonsuz döngüye sokar. 
    //zamanlayıcı sürekli çalışmaya devam eder ve mesajlar publish edilir
    rclcpp::shutdown();//Düğüm durdurulduğunda (örneğin Ctrl+C'ye basıldığında) ROS 2 bağlantılarını ve temizlik işlemlerini güvenli bir şekilde kapatır.
    return 0;
}
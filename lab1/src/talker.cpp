#include "rclcpp/rclcpp.hpp"//ROS2 nin C++ için temel kütüphanesi
#include <chrono> //zaman ve süre ile ilgili işlemler için(ms cinsinden bekleme sğreleri gibi) 
#include <memory> //smart pointers(std:shared_ptr gibi) kullanmak için gerekli. Ros2 bellek yönetimini güvenli yapmak için akıllı işaretçileri yoğun bir şekilde kullanır
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // F1TENTH simülatörüne göndereceğimiz mesajın tipini tanımlayan başlık dosyası
using namespace std::chrono_literals; //zaman birimlerini uzun uzun değilde kısa yazmamamıza olanak sağlar. ms, s, h gibi
//namespacelerin içinde fonksiyonlar, operatörler, değişkenler gibi tanımlamalar bulunur. 
//using namespace ile bir namespace'i çalışmamıza dahil ederiz

class Talker : public rclcpp::Node {
//Ros2'nin temel rclcpp::Node sınıfından miras aldığını belirtir. Yani Talker Tam teşekkülü bir ROS2 node'udur

public:
    Talker() : Node("talker"){//Talker(): Sınıf başlatıldığında çalışan ilk fonksiyondur.
    //: Node("talker"): Miras aldığımız ana Node sınıfına, bu düğümün adının sistemde "talker" olarak görünmesi gerektiğini söylüyoruz
    //terminalde ros2 node list yazarsam bu ismi görürüm
        
        // PARAMETRELERİ TANIMLAMA
        this->declare_parameter("v", 2.0);
        this->declare_parameter("d", 1.0);
        //declare_parameter: ROS2 de parametreler, düğüm çalışırken dışarıdan (örneğin launch dosyasından) değiştirilebilen
        //değişkenlerdir. Burada v(hız) ve d(direksiyon açısı) adında iki parametre tanımlıyorum
        
        //PUBLİSHER OLUŞTURMA
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        //create_publisher: mesaj yayınlayacak nesneyi oluşturur.
        //ackermann_msgs::msg::AckermannDriveStamped: yayınlanacak messajın tipi
        //"/drive": mesajın yayınlanacağı topic'in adı
        //10: queue boyutu. eğer mesajlar işlenemeyecek kadar hızlı gelirse, son 10 mesaj bellekte tutulur, eskileri silinir

        //TIMER OLUŞTURMA
        timer_ = this->create_wall_timer(
            10ms, std::bind(&Talker::timer_callback, this)
        );
        //create_wall_timer: Belirli aralıklarla sürekli olarak bir fonksiyonu çalıştıran bir zamanlayıcı olşturur.
        //10ms: her 10 milisaniyede bir (yani saniyede 100 defa, 100Hz) calışacağını belirtir
        //F1TENTH'de aracı pürüzsüz sürmek için yüksek frekans gerekir
        //std::bind(...): zamanlayıcı her tetiklendiğinde timer_callback isimli fonksiyonun çağrılmasını sağlar
    }

private:
    void timer_callback(){
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        //auto msg = ...: İçi boş, yeni bir Ackermann mesaj nesnesi oluşturur. Verileri bu nesnenin içine dolduracağım

        msg.drive.speed = this->get_parameter("v").as_double();
        msg.drive.steering_angle=this->get_parameter("d").as_double();
        //this->get_parameter("...").as_double(): Daha önce tanımladığım v ve d parametrelerinin anlık değerlerini okur 
        //ve bunları double olarak alır.
        //Bu değerleri, oluşturduğum msg nesnesinin içindeki speed (hız) ve steering_angle (direksiyon açısı) alanlarına kaydeder.

        publisher_->publish(msg);
        //Doldurduğumuz bu mesajı publisher_ nesnesi aracılığıyla ROS 2 ağına (yani /drive topicine) gönderir.
    }

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

//Bunlar sınıfımızın private değişkenleridir. Yukarıda oluşturduğum zamanlayıcı (timer_) ve yayıncıyı (publisher_)
//bellekte tutmak için kullanılan akıllı işaretçilerdir (SharedPtr)


};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv); //ros2 sistemini başlatır
    rclcpp::spin(std::make_shared<Talker>()); //Talker düğümünü ayağa kaldırır, sonsuz döngüye sokar. 
    //zamanlayıcı sürekli çalışmaya devam eder ve mesajlar publish edilir
    rclcpp::shutdown();//Düğüm durdurulduğunda (örneğin Ctrl+C'ye basıldığında) ROS 2 bağlantılarını ve temizlik işlemlerini güvenli bir şekilde kapatır.
    return 0;
}
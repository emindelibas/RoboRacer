#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include <fstream>//CSVeklendi


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    // Waypoint'leri tutacağımız liste (x ve y koordinatları)
    std::vector<std::pair<double, double>> waypoints; 

    // Pure Pursuit Parametreleri
    double lookahead_distance = 1.0; // İleriye bakış mesafesi (l_d). Hızlandıkça bunu artırmak gerekebilir.
    double wheelbase = 0.35;         // Aracın dingil mesafesi (L). F1TENTH araçları için genelde 0.33 metredir.


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    
    
    //*************************
    //yeni eklendi
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
    //Tüm rotayı çizmek için yeni yayınlayıcı
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr route_viz_pub_;


    //CSV dosyasında waypoint okuma fonksiyonu

    void read_waypoint(const std::string& file_path){
        fstream file(file_path);// hem okuma hem yazma işlemlerinin yapılması için "file" oluşturdum
        
        /*
        C++'daki standart fstream kullanmak için normalde başına "std::" konulur ama en başta 
        "using namespace std;" olduğu için std:: yazmama gerek kalmadı 
        */
        
        string line, word;

        if (!file.is_open()){
            RCLCPP_ERROR(this->get_logger(), "CSV dosyasi acilamadi! Yol:%s", file_path.c_str());
            //c_str(): file_path içindekini string verir
            /*
            Bu fonksiyon, basic_string nesnesinin geçerli değerini temsil eden, null karakterle sonlandırılmış 
            bir karakter dizisi içeren bir diziye işaretçi döndürür. Bu dizi, basic_string nesnesinin değerini
            oluşturan karakter dizisinin aynısını ve sonunda ek bir sonlandırıcı null karakteri içerir.
            */
           return;
        }
        
        while (getline(file, line)){//dosyayı satır satır okuma. en baştan aşağıya doğru
            /*
            getline() fonksiyonu, dosyadan bir satırı sonuna (enter tuşuna basılan yere) kadar okur ve bunu line isimli metin 
            (string) değişkenine kaydeder. Dosyada okunacak satır kalmayana kadar bu while döngüsü dönmeye devam eder.
            */
            

            stringstream s(line);
            /*
            stringstream s(line): Az önce okuduğumuz bütün bir satırı (örneğin: "1.5, 2.3, 0.0") alır ve 
            onu içinden kelime kelime veri çekebileceğimiz bir "akışa" (stream) dönüştürür.
            */
            //getline() fonksiyonu sadece dosyalarda çalıştığı için stringstream kullandık. line'daki değeri bir nevi dosyaya dönüştürdük. 


            vector<double> row;
            //vector<double> row: O an okuduğumuz satırdaki sayıları geçici olarak tutacağımız boş bir liste (vektör) oluşturur.
            

            while (getline(s, word, ',')){//Bu içteki while döngüsü, az önce hazırladığımız satırı soldan sağa doğru virgül (,) görene kadar okur.
                row.push_back(stod(word));
                /*
                stod(word): Virgül görene kadar okuduğu metni (örneğin "1.5") alır ve 
                String TO Double komutuyla onu matematiksel bir ondalıklı sayıya (1.5) çevirir.
                */
                
                //push_back: Çevirdiği bu sayıyı geçici row (satır) listemize ekler. Satırdaki tüm virgüller bitene kadar bu işlem devam eder.

            }
            if (row.size()>=2){
                waypoints.push_back({row[0], row[1]});
            }
            /*
            Bir satırdaki tüm sayıları okuyup row listesine koyduk. Eğer bu listede en az 2 tane sayı varsa (çünkü bize X ve Y koordinatları lazım), 
            ilk sayıyı (row[0]) X koordinatı, ikinci sayıyı (row[1]) Y koordinatı olarak kabul eder.
            */
            //Bu ikiliyi, sınıfımızın en başında tanımladığımız ve arabanın rotasını belirleyen ana waypoints listesine kalıcı olarak ekler.
        }
        RCLCPP_INFO(this->get_logger(),"%zu waypoint basariyla yuklendi.", waypoints.size());



    }

    //*********************************** 
    //Marker publish etme fonksiyonu
    void publish_marker(double x, double y){//x ve y konumunda marker
        visualization_msgs::msg::Marker marker;
        //auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id="map"; //haritanın koordinatlarına göre çizdirmek için
        marker.header.stamp=this->now();//o anın zamanına ayarlandı


        marker.ns="target_waypoint";//namespace.

        marker.type=visualization_msgs::msg::Marker::SPHERE;//şekil küre oldu
        marker.action=visualization_msgs::msg::Marker::ADD;//ekrana ekle, varsa güncelle


        //dışardan gelen x ve y değerlerini haritada yerine oturtur
        marker.pose.position.x=x;
        marker.pose.position.y=y;
        marker.pose.position.z=0.0;//yerde

        //YÖNELİM (ORIENTATION)
        //marker'ın ne tarafa yönelceğini belrtmek için. çapının yönü
        marker.pose.orientation.x=0.0;
        marker.pose.orientation.y=0.0;
        marker.pose.orientation.z=0.0;
        marker.pose.orientation.w=1.0;//düz dur anlamına gelir
        /*
        Eğer bir nesnenin hiçbir yöne dönmesini istemiyorsan (yani Roll=0, Pitch=0, Yaw=0 ise),
        bunun kuaterniyon dilindeki karşılığı Identity Quaternion (Birim Kuaterniyon) olarak adlandırılır ve yukardaki değerleri alır
        sqrt{x^2 + y^2 + z^2 + w^2} = 1
        */

        //boyut
        marker.scale.x=0.3;
        marker.scale.y=0.3;
        marker.scale.z=0.3;

        //renk ve saydamlık
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; //saydamlık

        viz_pub_->publish(marker);
    }

    void publish_full_route() {
        visualization_msgs::msg::Marker route_marker;
        route_marker.header.frame_id = "map";
        route_marker.header.stamp = this->now();

        route_marker.ns = "full_route";//namespace
        //route_marker.id = 1; // Hedefte 0 kullanmıştık, bu 1 olmalı
        
        // KRİTİK NOKTA: Tek bir küre değil, küreler listesi (SPHERE_LIST)
        route_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; 
        route_marker.action = visualization_msgs::msg::Marker::ADD;

        // Noktaların boyutu
        route_marker.scale.x = 0.1;
        route_marker.scale.y = 0.1;
        route_marker.scale.z = 0.1;

        // Renk 
        route_marker.color.a = 1.0; 
        route_marker.color.r = 0.0;
        route_marker.color.g = 0.0;
        route_marker.color.b = 1.0;

        // orientation (w = 1.0 kuralı yine var)
        route_marker.pose.orientation.w = 1.0;

        // Bütün noktaları listeye ekleme döngüsü
        geometry_msgs::msg::Point p;
        for (const auto& wp : waypoints) {
            p.x = wp.first;
            p.y = wp.second;
            p.z = 0.0;
            route_marker.points.push_back(p); // Her noktayı mesaja paketleme
        }

        // Bütün rotayı RViz'e gönder
        route_viz_pub_->publish(route_marker);
        //viz_pub_->publish(route_marker);
    }



public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers





        drive_pub_ = this-> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",10
        );

        //***********************
        //visualization
        viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoint_markers", 10);
        route_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/full_route_markers", 10);

        std::string csv_path = "/home/emin/workspaces/f1tenth_ws/src/lab5/pure_pursuit/levine_rotasi.csv";//???
        read_waypoint(csv_path);

        pose_sub_ = this -> create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom",10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1)
        );//anlık konum ve yönelim bilgileri için PoseStamped kullanıldı
        
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
    {

        //Aracın anlık konumları
        double car_x = pose_msg->pose.pose.position.x;
        double car_y = pose_msg->pose.pose.position.y;


        /*
        tf2 (Transform Framework 2), ROS'un kalbinde yatan ve zaman içinde robotun farklı parçalarının 
        (tekerlekler, lazer sensörü, şasi) ve haritanın birbirine göre nerede olduğunu takip eden devasa bir kütüphanedir.
        */
        /*
        onun içindeki LinearMath (Doğrusal Matematik) araçlarını kullanıyoruz. 
        Çünkü tf2, 3 boyutlu uzay açılarını birbirine dönüştürmek için harika hazır fonksiyonlara sahiptir.
        */


        /*
        Neden Bu Dönüşüme İhtiyacımız Var? (Problemimiz Ne?)
        
        ROS 2, PoseStamped mesajında arabanın yönelimini (orientation) biz insanların anladığı dereceler (örneğin "Kuzeye doğru 45 derece") cinsinden göndermez. 
        Bunun yerine Quaternion (Kuaterniyon) adı verilen 4 boyutlu (x, y, z, w) karmaşık bir matematiksel format kullanır.

        Neden Quaternion? Çünkü 3 boyutlu hesaplamalarda bilgisayarlar için çok daha hızlıdır ve "Gimbal Lock" denilen, 
        eksenlerin üst üste binip kilitlenmesi sorununu çözer.

        Sorun Ne? Biz Pure Pursuit algoritmamızda X ve Y eksenli 2 boyutlu bir düzlemde çalışıyoruz. 
        Arabanın sadece sağa veya sola ne kadar döndüğünü bilmek istiyoruz. (x, y, z, w) sayıları bizim algoritmamız için hiçbir anlam ifade etmiyor.

        İşte bu kod, bilgisayarların anladığı o 4 boyutlu karmaşık yapıyı, bizim anladığımız Yaw 
        (Z ekseni etrafında dönüş / Arabanın burnunun baktığı yön) açısına çeviriyor.
        */


        tf2::Quaternion q(
            pose_msg->pose.pose.orientation.x,
            pose_msg->pose.pose.orientation.y,
            pose_msg->pose.pose.orientation.z,
            pose_msg->pose.pose.orientation.w
            //ROS mesajından gelen o 4 anlamsız sayıyı alıyoruz ve tf2 kütüphanesinin anlayabileceği resmi bir tf2::Quaternion objesi (q) haline getiriyoruz.
        );
        tf2::Matrix3x3 m(q);
        /*
        Oluşturduğumuz bu q kuaterniyonunu alıp, 3x3 boyutunda bir Dönüşüm Matrisine (m) çeviriyoruz. 
        Neden mi? Çünkü Matrix3x3 sınıfının içinde bizim için hayat kurtaran şu fonksiyon var: m.getRPY(roll, pitch, yaw).
        */  


        //yuvarlanma, şaha kalkması ve burnunun sağa sola dönmesi
        double roll, pitch, yaw;//boş değerler, aşşağıda doldurcam
        //önemli olan yaw'dır. diğerleri getRPY içine yazmamız gerektiği için var, onları kullanmayacağız
        
        m.getRPY(roll, pitch, yaw);
        /*
        Dönüşüm matrisimize (m) diyoruz ki: "İçindeki o karmaşık 3 boyutlu kuaterniyon matematiğini çöz, 
        Roll, Pitch ve Yaw açılarını hesapla ve gelip az önce yarattığım bu üç boş kutunun içine doldur."

        Bu satır çalıştıktan sonra artık elimizde arabanın yönünü belirten üç adet ondalıklı sayı (radyan cinsinden) oluyor.
        */

        double best_distance = numeric_limits<double>::infinity(); //değeri pozitif sonsuz değeri olarak döndürür. infinity() değeri numeric_limits'den gelir
        /*
        aradığımız lookahead mesafesine en yakın noktayı (yani mesafe farkı en küçük olan noktayı) bulmak istiyoruz. 
        Çıtayı en başta "sonsuz" kadar yüksek tutarız ki, döngünün içine girip denediğimiz ilk geçerli nokta anında 
        rekoru kırıp yeni "en iyi noktamız" olsun. Sonraki noktalar da bu yeni rekoru kırmaya çalışsın.
        */
        std::pair<double, double> best_local_wp;//En iyi noktayı tutmak için boş bir kutu oluşturdum. X ve Y değerlerini tutacak

        bool found_waypoint = false; //nokta bulunca true olacak. nokta bulamazsa best_locakl_wp kutusu anlamsıza sayılarla direksiyon açısı hesaplamaya çalışır
        // bu çökmeye sebep olabilir. bunun olmaması için bir önlem olarak koyarız. döngüde kullanacağım

        //***************
        // Hedef noktanın harita koordinatlarını tutmak için iki boş değişken oluştur. 
        double best_global_x = 0.0;
        double best_global_y = 0.0;


        // TODO: find the current waypoint to track using methods mentioned in lecture

        for ( const auto& wp : waypoints){
            //Arabanın hafızasına yüklediğimiz tüm noktaları (waypoints) sırayla tek tek ele alıyoruz. Her bir noktayı wp (waypoint) adında bir değişkene atıyoruz.
            double dx = wp.first - car_x;//nokta araçtan ne kadar uzaklıkta. x olarak. map'in koordinat sistemşne göre
            double dy = wp.second - car_y;//nokta araçtan ne kadar uzaklıkta. y olarak
            /*
            Sorun Ne? Bu mesafeler hala haritanın Kuzey-Güney (Y) ve Doğu-Batı (X) yönlerine göre hesaplı. 
            Arabanın burnu o an çapraz bakıyor olabilir! Bunu düzeltmemiz lazım.
            */




            // TODO: transform goal point to vehicle frame of reference
            //aracın koordinat sistemine göre hedef nokta nerede tanımlaması yapılır
            //map'in koordinat sisteminden aracın koordinat sistemine:geçiş
            //YAW: mapin koordinat sistemindeki x ekseni ile aracın kendi x ekseni arasındaki açı
            double local_x = std::cos(-yaw)*dx - sin(-yaw)*dy;//
            double local_y= std::sin(-yaw) * dx + std::cos(-yaw) * dy;

            if(local_x > 0.0){//eğer nokta aracın önünde ise
                double dist = std::sqrt(dx*dx + dy*dy);//mesafeyi belirle
                

                //Lookahead mesafesine en yakın olanı seç
                if (std::abs(dist - lookahead_distance) < std::abs(best_distance - lookahead_distance)){
                    //karşılaştırma: şu an incelediğimiz noktanın hata payı(çemberden ne kadar uzak), şampiyon hata payından daha küçük ise 
                    //demek ki çembere daha yakın yani daha iyi bir nokta bulduk

                    best_distance = dist;
                    best_local_wp = {local_x, local_y};//yeni noktanın araca göre konumu kaydedilir
                    

                    //************
                    best_global_x=wp.first;
                    best_global_y=wp.second;



                    found_waypoint = true;

                }  
            }
        }

        //************
        publish_marker(best_global_x,best_global_y);

        //****************************
        // Dosya okuması bitip liste dolduğunda hemen rotayı çiz!
        publish_full_route();



        // TODO: calculate curvature/steering angle
            
        // Formül: Curvature (Eğrilik) = 2 * y_local / L_d^2
        double curv = (2.0 * best_local_wp.second) / (best_distance*best_distance);

        double kp=0.3;
        double steering_angle = std::atan(wheelbase * curv);
        //double steering_angle = kp * curv;

        // TODO: publish drive message, don't forget to limit the steering angle.
        steering_angle = std::clamp(steering_angle, -0.36, 0.36);

        //viraja göre hız belirleme
        double velocity = 0.0;
        if(abs(steering_angle) < 0.17) velocity = 2.5;//düz yol veya hafif virej
        else velocity = 1.5;//keskin viraj

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();//boş bir mesaj paketi yarattım
        //STAMPED mesajların hepsinin bir header'ı olmak zorunda. bir etiket gibidir.


        //hız açı atamaları ve publish
        drive_msg.drive.steering_angle=steering_angle;
        drive_msg.drive.speed=velocity;
        drive_pub_->publish(drive_msg);


        // if(found_waypoint){
        //     // TODO: calculate curvature/steering angle
                
        //     // Formül: Curvature (Eğrilik) = 2 * y_local / L_d^2
        //     double curv = (2.0 * best_local_wp.second) / (best_distance*best_distance);

        //     double kp=0.3;
        //     //double steering_angle = std::atan(wheelbase * curv);
        //     double steering_angle = kp * curv;

        //     // TODO: publish drive message, don't forget to limit the steering angle.
        //     steering_angle = std::clamp(steering_angle, -0.36, 0.36);

        //     //viraja göre hız belirleme
        //     double velocity = 0.0;
        //     if(abs(steering_angle) < 0.17) velocity = 2.5;//düz yol veya hafif virej
        //     else velocity = 1.5;//keskin viraj

        //     auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();//boş bir mesaj paketi yarattım
        //     //STAMPED mesajların hepsinin bir header'ı olmak zorunda. bir etiket gibidir.


        //     //hız açı atamaları ve publish
        //     drive_msg.drive.steering_angle=steering_angle;
        //     drive_msg.drive.speed=velocity;
        //     drive_pub_->publish(drive_msg);

        // }

        
    }

    

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
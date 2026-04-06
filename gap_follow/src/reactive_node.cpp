#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers

        bubble_radius_ = 0.5; // Metre cinsinden güvenlik balonu yarıçapı
        max_lidar_range_ = 5.0; // Uzağı çok fazla önemsememek için sınır
        scan_sub_ = this -> create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10
        );

        RCLCPP_INFO(this->get_logger(), "Reactive Gap Follow Node Baslatildi.");

    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;


    float bubble_radius_;
    float max_lidar_range_;
    // Lidar dizisinin boyutunu tüm sınıf fonksiyonlarının görmesi için buraya ekliyoruz
    //Lidar verisi her geldiğinde bu değişkeni güncelleyip tüm fonksiyonların erişmesini sağlayacağız.
    int num_beams_;
    float angle_increment_;
    float angle_min_;



    //preprocess_lidar():sensörden gelen ham, gürültülü veya hatalı verileri, 
    //ana algoritmamızın (Follow the Gap) sorunsuz çalışabileceği temiz ve güvenilir bir hale getirmeye yarar.
    
    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        for (int i = 0; i < num_beams_; ++i) {
            if (std::isnan(ranges[i]) || std::isinf(ranges[i])) {
                ranges[i] = 0.0;
            } else if (ranges[i] > max_lidar_range_) {
                ranges[i] = max_lidar_range_;
            }

            // Aracın sadece önü (-75 ile +75 derece arası) 
            float angle = angle_min_ + i * angle_increment_;
            if (angle < -M_PI*5/ 12.0 || angle > M_PI*5 / 12.0) {
                ranges[i] = 0.0; 
            }
        }

    }

    // Güvenlik balonu 
    void create_safety_bubble(float* ranges) {
        // 1. Lidar verisindeki en yakın noktayı (engeli) bul
        float min_dist = 100.0; // Başlangıç için yeterince büyük bir değer
        int closest_idx = -1; //en yakın noktanın indexi. henüz engel olmadığı için -1

        for (int i = 0; i < num_beams_; ++i) {
            if (ranges[i] > 0.0 && ranges[i] < min_dist) { 
                min_dist = ranges[i];
                closest_idx = i;
            }
        }

        // Güvenlik Önlemi: Eğer tüm dizi 0 ise (geçerli okuma yoksa) çökmeyi önlemek için çık
        if (closest_idx == -1) {
            return; 
        }

        // 2. Sıfıra Bölme Koruması
        /*if (min_dist <= 0.0f) {
            min_dist = 0.01f;
        }*/

        // 3. Trigonometrik Balon Hesabı
        float angle = std::atan(bubble_radius_ / min_dist);
        int idx_radius = std::abs(angle / angle_increment_);//açı içindeki ışın sayısı

        // 4. Balonun içini sıfırlama
        for(int i = closest_idx - idx_radius; i <= closest_idx + idx_radius; ++i) {//en yakın engel merkez alınır

            if(i >= 0 && i < num_beams_) {
                ranges[i] = 0.0; 
            }
        }
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges



        int current_start = -1;// o an okuduğumuz boşluğun başladığı indeks
        //-1 de yazılabilir, bu br güvenlik önlemidir. şu anda bir indeks tutmuyo demektir
        int max_start=0;// Şimdiye kadar bulduğumuz EN BÜYÜK boşluğun başlangıcı


        int current_size= 0;// O an okuduğumuz boşluğun uzunluğu
        int max_size=0;// Şimdiye kadar bulduğumuz EN BÜYÜK boşluğun uzunluğu


        /*
        Diziyi baştan sona okuyoruz.

        Sıfırdan büyük sayılar gördükçe sayacı (current_length) artırıyoruz.

        Ne zaman bir 0.0 (tehlike) görsek, hemen o ana kadar saydığımız adım sayısına bakıyoruz. 
        Eğer bu adım sayısı, eski rekorumuzdan (max_length) daha fazlaysa, yeni REKOR bu aralık oluyor.

        Sonra sayacı sıfırlayıp bir sonraki 0.0 olmayan seriyi saymaya başlıyoruz.
        
        */



        for (int i=0; i<num_beams_; ++i)
        {
            // Eğer mesafe 0.0'dan büyükse (yani güvenli bir yolsa ve balonun içinde değilse)
            if ( ranges[i]>0.0){
                if(current_size == 0){
                    current_start=i;
                }
                
                current_size++;
            }
            else {
                if (current_size > max_size){
                    max_size = current_size;
                    max_start = current_start;
                }
                current_size=0;
                }
                
        }

        // Çok Önemli Bir Detay: Döngü bittiğinde, elimizde henüz sıfırlanmamış, 
        // dizinin en sonuna kadar devam eden bir seri kalmış olabilir. 
        // Bu seri de rekor kırmış olabilir, onu da kontrol etmeliyiz:
        if (current_size > max_size) 
        {
            max_size = current_size;
            max_start = current_start;

        }

        // Şablonun bizden beklediği int* indice formatına göre sonuçları kaydediyoruz:
        // indice[0] = Max gap başlangıç indeksi
        // indice[1] = Max gap bitiş indeksi

        // Sonuçları dışarı aktar
        indice[0] = max_start;
        
        // Bitiş indeksi: Başlangıç noktası + uzunluk - 1 (Çünkü indeksler 0'dan başlar)
        // Eğer uzunluk 0 ise (her yer 0 olmuşsa, ki bu bir hata durumudur), -1 dönmesini engellemek için küçük bir güvenlik önlemi:
        //max_length = 0 ve max_start=0 olacak, else bloğuna girecek.
        //indice[1] = 0 olur. "Bulabildiğim en büyük boşluk 0. indekste başlıyor ve 0. indekste bitiyor."
        if (max_size > 0) {
            indice[1] = max_start + max_size - 1; 
        } 
        else {
            indice[1] = max_start; 
        }

    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there


        //indice[0] boşluğun başlangıcı, indice[1] bitişi
        int start_i=indice[0];
        int end_i = indice[1];

        //int best_idx = (start_i + end_i) / 2;// hedef noktanın indeksi


        float max_depth=0.0f; //o noktaya olan uazklık


        // Sadece bulduğumuz güvenli boşluğun içindeki noktaları tarıyoruz
        for(int i=start_i;i<=end_i;++i){
            if(ranges[i]>max_depth){
                max_depth=ranges[i];
                //best_idx=i;

            }

        }
        
        
        // 2. Adım: Maksimum derinliğe eşit olan (veya çok yakın olan) 
        // ardışık ışınların başlangıç ve bitişini bulma



        int max_depth_start = -1;//henüz koridorun başlangıç ve bitişi olmadığı için -1
        int max_depth_end = -1;
        float threshold = max_depth - 0.2f;

        for (int i = start_i; i <= end_i; ++i) {
            // Float karşılaştırmasında hassasiyet (0.01) payı bırakıyoruz
            if (ranges[i]>=threshold) {
                if (max_depth_start == -1) {//Eğer bu koşul sağlanıyorsa, derin koridorun ilk ışınını bulduk demektir.
                    max_depth_start = i; // Serinin başı
                }//sadece bir kere, serinin en başında güncellenir

                max_depth_end = i;// Serinin sonu
                //Bu satır ise koşulu sağlayan her ışında sürekli güncellenir.
            }
        }

        //sonucu çağıran fonksiyona iletmek için indice dizisinin ilk elemanını güncellenir
        if (max_depth_start !=-1){
            indice[0] = (max_depth_start + max_depth_end) / 2;
        }
        else{
            indice[0] = (start_i + end_i) / 2;
        }

        // KÖŞE TUZAĞINI (Corner Trap) ÖNLEMEK İÇİN:
        // En derin noktayı (max_depth) aramak yerine, 
        // doğrudan bulduğumuz güvenli boşluğun "geometrik merkezini" hedef alıyoruz.
        
        
        

        
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {   
        // 1. Dizi boyutunu sınıfa kaydet
        num_beams_ = scan_msg->ranges.size();
        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;

        //2. Ros2 vektörünü kopyalayıp float* (raw pointer) elde etme
        std::vector<float> ranges_vec = scan_msg->ranges; //?
        //scan_msg->ranges çağırıldığında bir vektör döner.
        //scan_msg const olduğu için bir kopyasını alıyoruz(ranges_vec)
        
        float* ranges_ptr = ranges_vec.data();//bu kopyanın bellekteki raw pointer'ını alıp oluşturduğumuz fonksiyonlara göncderebiliyoruz
        //ranges_vec.data(): ROS2'nin verdiği ranges_vec vektöründen, içindeki verilerin bellekte başladığı adresi verir
        // bunu ranges pointerına atıyoruz ki fonksiyonar bunu kullanabilsin
        //.data() fonksiyonu çağırıldığında doğrudan Lidar sensörünün ölçtüğü mesafe (range) verilerine ulaşırız


        /*
        Burada veriyi kopyalamıyoruz, sadece verinin yerini gösteren bir adres alıyoruz. 
        Yani aynı eve giden iki farklı yol tarifimiz oluyor. Bu yüzden ranges pointer'ı üzerinden yapacağım her değişiklik
        (örneğin ranges[i] = 0.0f demek), aslında doğrudan ranges_vec vektörünün içindeki veriyi değiştirir. 
        Bu da tam olarak preprocess_lidar ve bubble adımlarında yapmak istediğim şey
        */


        // 1. Lidar verisini işle
        preprocess_lidar(ranges_ptr);
        // 2. Güvenlik balonunu oluştur (Engellerin etrafını sıfırla)
        create_safety_bubble(ranges_ptr);
        // 3. En büyük boşluğu bul (2 elemanlı bir dizi kullanıyoruz)
        int gap_indices[2] = {0, 0};
        find_max_gap(ranges_ptr, gap_indices);
        // 4. Hedef noktayı seç (gap_indices[0]'a sonucu yazacak)
        find_best_point(ranges_ptr, gap_indices);
        int target_idx = gap_indices[0];
        // 5. Direksiyon açısını hesapla ve ROS 2 üzerinden yayınlaangle
        float steering_angle = angle_min_ + target_idx * angle_increment_;//angle_min_,(başlangıç açısı) 0. indeksteki Lidar ışınının fiziksel olarak nereye baktığını söyler.
        //target_idx * angle_increment_: toplam ne kadarlık bir açısal mesafe kat ettiğimizi buluruz
        float velocity = 0.0;
        if (std::abs(steering_angle) < 0.17) velocity = 2.5; // Düz yol
        else if (std::abs(steering_angle) < 0.35) velocity = 2.0; // Hafif viraj
        else velocity = 1.0; // Keskin viraj

        


        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        drive_msg.drive.steering_angle=steering_angle;
        drive_msg.drive.speed=velocity;

        drive_pub_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
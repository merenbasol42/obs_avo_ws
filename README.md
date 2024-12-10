# Otonom Engel Tespiti ve Kaçınma Sistemi

## Proje Açıklaması
Bu ROS2 paketi, Lidar sensörü kullanarak otonom engel tespiti ve kaçınma sistemi sağlar. Robot, önündeki engelleri algılayarak çarpışmadan hareket edebilir.

## Gereksinimler
- ROS2 Humble
- Gazebo
- TurtleBot3 paketleri
- Python 3.10+

## Ön Hazırlık

### Gerekli Paketlerin Kurulumu
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-description
```

### Ortam Değişkenlerini Ayarlama
```bash
# TurtleBot3 modelini ayarla
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# ROS Domain ID'sini ayarla (çakışmaları önlemek için)
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc

# Gazebo kaynak dosyasını güncelle
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

source ~/.bashrc
```

## Kurulum
1. ROS2 workspace'inizde klonlayın
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Bağımlılıkları yükleyin
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Paketi derleyin
```bash
colcon build --packages-select obstacle_avoidance
source install/setup.bash
```

## Çalıştırma
### Gazebo Simülasyonunda Çalıştırma
```bash
# Gazebo simülasyonunu başlat
ros2 launch obstacle_avoidance gazebo_simulation_launch.py
```

### Manuel Düğüm Çalıştırma
```bash
# Ayrı ayrı düğümleri çalıştırma
ros2 run obstacle_avoidance lidar_node
ros2 run obstacle_avoidance obstacle_avoidance_node
```

## Sorun Giderme

### Yaygın Sorunlar ve Çözümleri

#### Lidar Verisi Alınamıyor
- ROS topic'lerini kontrol edin:
  ```bash
  ros2 topic list
  ros2 topic echo /scan
  ```

#### Gazebo Spawn Hatası
- URDF dosyasının doğru yolda olduğundan emin olun
- Gazebo ve ROS2 sürümlerinin uyumlu olup olmadığını kontrol edin

#### ROS Domain ID Çakışmaları
- Her bir makinede farklı ROS_DOMAIN_ID kullanın
- Ağ üzerinden iletişim sorunlarını çözün

### Log Dosyaları
- Detaylı log bilgileri için:
  ```bash
  ros2 run rqt_console rqt_console
  ```

## Düğümler
- `lidar_node`: Lidar verilerini işler
- `obstacle_avoidance_node`: Engel kaçınma algoritmasını uygular

## Konfigürasyon
Engel algılama parametreleri düğüm içinde ayarlanabilir:
- `min_obstacle_distance`: Minimum engel mesafesi (default: 0.5m)
- `obstacle_angle_range`: Engel tarama açısı (default: 30 derece)

## Lisans
MIT Lisansı

## Katkıda Bulunma
Hata bildirimleri ve pull request'ler memnuniyetle karşılanır.

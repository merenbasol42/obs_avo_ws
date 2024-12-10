# ROS2 Otonom Engel Kaçınma Sistemi (obs_avo_ws)

## Proje Açıklaması
ROS2 Humble ve TurtleBot3 kullanarak geliştirilen otonom engel tespiti ve kaçınma sistemi. Proje, Lidar sensörü ile gerçek zamanlı engel algılama ve otomatik yön değiştirme stratejisi içerir.

## Özellikler
- Lidar sensörü ile gerçek zamanlı engel tespiti
- Otomatik yön değiştirme algoritması
- Gazebo simülasyon desteği
- TurtleBot3 uyumlu

## Gereksinimler
- ROS2 Humble
- TurtleBot3 paketleri
- Gazebo
- Python 3.10+

## Kurulum

### 1. Gerekli Paketlerin Kurulumu
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-description
```

### 2. Ortam Değişkenlerini Ayarlama
```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

### 3. Projeyi Klonlama
```bash
git clone https://github.com/ustad/obs_avo_ws.git
cd obs_avo_ws
```

### 4. Bağımlılıkları Yükleme
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Projeyi Derleme
```bash
colcon build --packages-select obstacle_avoidance
source install/setup.bash
```

## Simülasyon Çalıştırma

### Gazebo ve TurtleBot3 Dünyasını Başlatma
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Engel Kaçınma Düğümlerini Çalıştırma
Ayrı bir terminalde:
```bash
ros2 launch obstacle_avoidance gazebo_simulation_launch.py
```

## Düğümler
- `lidar_node`: Lidar verilerini işler
- `obstacle_avoidance_node`: Engel kaçınma algoritmasını uygular

## Konfigürasyon
Engel algılama parametreleri düğüm içinde ayarlanabilir:
- `min_obstacle_distance`: Minimum engel mesafesi (default: 0.5m)
- `obstacle_angle_range`: Engel tarama açısı (default: 30 derece)

## Sorun Giderme
- Lidar verisi kontrolü:
  ```bash
  ros2 topic list
  ros2 topic echo /scan
  ```

## Katkıda Bulunma
Pull request'ler ve hata bildirimleri memnuniyetle karşılanır.

## Lisans
MIT Lisansı

## İletişim
Proje sahibi: [GitHub Profili](https://github.com/ustad)

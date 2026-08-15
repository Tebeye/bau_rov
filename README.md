# Autonomous Underwater Vehicle (ROV) Line Tracking System 📡

ROS 2 Humble & PX4 (Pixhawk) autonomous underwater vehicle (ROV) line tracking, depth stabilization PID, vision processing, SITL simulation, and fail-safe system.

---

## 📡 1. Communication Architecture & ROS 2 Topic Map

Communication between Raspberry Pi (ROS 2 Humble) and Pixhawk (PX4) is handled via MAVROS (MAVLink) at **921600 baud rate**:

```
[Pixhawk]  ─── (/mavros/local_position/odom) ───> [control_node] (Depth Z feedback)
[Camera]   ─── (/camera/image_raw)           ───> [vision_node]  (Raw BGR Image)
[vision]   ─── (/vision/line_error)          ───> [control_node] (Centroid Error - Hata Yaw)
[control]  ─── (/mavros/setpoint_raw/attitude) ─> [Pixhawk]      (Thrust / Attitude Rates)
```

---

## 📋 2. Task List (FAZ 1 - FAZ 5 Status)

### FAZ 1: İletişim ve Donanım Altyapısı
- [x] **T-1.1**: Ubuntu Server 22.04 LTS ve ROS 2 Humble ortam gereksinimleri hazırdır. Script: `scripts/setup_pi_environment.sh`
- [x] **T-1.2**: Pixhawk TELEM2 portu ile Pi UART/USB fiziksel bağlantı ayarları (Baudrate: 921600) tanımlandı.
- [x] **T-1.3**: MAVROS paketleri kuruldu ve launch dosyasına entegre edildi.
- [x] **T-1.4**: Pixhawk `/mavros/local_position/odom` ROS 2 konu dinleme aboneliği `control_node.py` içinde tanımlandı.

### FAZ 2: Gazebo Classic 11 SITL Simülasyonu
- [x] **T-2.1**: Gazebo Classic 11 ve PX4 SITL entegrasyon yapısı hazırlandı.
- [x] **T-2.2**: Kamera ve Derinlik sensör eklentili Sualtı aracı SDF modeli oluşturuldu: `models/rov_model/model.sdf`
- [x] **T-2.3**: Sualtı ortamı ve sarı hat takip haritası oluşturuldu: `worlds/tema1_hat_takibi.world`

### FAZ 3: OpenCV Görüntü İşleme Düğümü (`vision_node`)
- [x] **T-3.1**: `/camera/image_raw` konusunu dinleyen `cv_bridge` tabanlı Python ROS 2 düğümü yazıldı: `rov_line_tracking/vision_node.py`
- [x] **T-3.2**: BGR -> Grayscale dönüşümü ve `cv2.adaptiveThreshold` (Gaussian) filtresi uygulandı.
- [x] **T-3.3**: `cv2.findContours` ile şerit konturları tespiti ve `cv2.moments` ile Ağırlık Merkezi ($C_x, C_y$) hesabı kodlandı.
- [x] **T-3.4**: Merkez sapma hatası ($\text{Hata\_Yaw} = X_{merkez} - C_x$) hesaplanıp `/vision/line_error` konusunda yayınlandı.

### FAZ 4: Kapalı Döngü PID ve Otonom Görev Düğümü (`control_node`)
- [x] **T-4.1**: 10 Hz frekansta çalışan asenkron ana görev döngüsü (`create_timer`) kurgulandı: `rov_line_tracking/control_node.py`
- [x] **T-4.2**: Derinlik Sabitleme (Heave) PID kontrolcüsü yazıldı (Hedef: 2.0m, Hassasiyet: ±5 cm, Max Kuvvet: 7 N).
- [x] **T-4.3**: Yöneltim (Yaw) PID kontrolcüsü yazıldı.
- [x] **T-4.4**: Surge (İleri) ve Sway (Yan) eksenlerinde sabitleme limitleri (Max Kuvvet: 10 N) tanımlandı.
- [x] **T-4.5**: Hesaplanan thrust ve rate verileri MAVROS setpoint başlıklarına aktarıldı.

### FAZ 5: Fail-Safe Mekanizması ve Havuz Testleri
- [x] **T-5.1**: Şerit kaybedildiğinde `LostCounter` ve Dead Reckoning arama modu geliştirildi: `rov_line_tracking/fail_safe.py`
- [x] **T-5.2**: Tolerans süresi aşıldığında motorları durdurup pozitif Heave ile acil yüzeye çıkış prosedürü eklendi.
- [x] **T-5.3**: SITL parametre optimizasyonu için `config/params.yaml` dosyası yapılandırıldı.
- [x] **T-5.4**: Fiziksel araç ve havuz testleri için hazır paket yapısı tamamlandı.

---

## 📅 3. Proje Takvimi (Schedule)

| Hafta | Aşama / Odak Noktası | Çıktı / Milestone |
| :--- | :--- | :--- |
| **Hafta 1** | Kurulum ve İletişim Köprüsü | Pi ↔ Pixhawk arası MAVROS bağlantısının kurulması ve sensör verilerinin ROS 2'de görülmesi. |
| **Hafta 2** | SITL ve Görüntü İşleme (`vision_node`) | Gazebo simülasyonunda adaptif threshold ile şerit merkezinin ($C_x, C_y$) hatasız tespiti. |
| **Hafta 3** | Derinlik ve Yaw PID Kontrolcüsü | Simülasyonda ±5 cm derinlik hassasiyeti ve 2sn yerleşme süresine sahip PID modülü. |
| **Hafta 4** | 10 Hz Görev Döngüsü & Fail-Safe | Şerit arama (Dead Reckoning) ve acil durum yüzeye çıkış mantıklarının doğrulanması. |
| **Hafta 5** | SITL Entegrasyonu & Havuz Testi | Tüm sistemin Gazebo'da tam otonom tur atması ve ardından fiziksel araç havuz testleri. |

---

## 🚀 Çalıştırma Talimatları

### 1. Raspberry Pi Ortam Kurulumu
```bash
bash scripts/setup_pi_environment.sh
```

### 2. ROS 2 Paketi Derleme ve Başlatma
```bash
cd ros2_ws
colcon build --packages-select rov_line_tracking
source install/setup.bash
ros2 launch rov_line_tracking line_tracking.launch.py
```

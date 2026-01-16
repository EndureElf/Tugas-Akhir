# QUICK START - Cara Cepat Akses Kamera MIPI

## 📍 LOKASI FILE

Semua file ada di:
```
/home/vboxuser/Documents/yahboomcar_ws/src/mipi_camera_simple/
```

## 🚀 CARA TERCEPAT (3 LANGKAH)

### 1. Build Package
```bash
cd /home/vboxuser/Documents/yahboomcar_ws
colcon build --packages-select mipi_camera_simple
source install/setup.bash
```

### 2. Jalankan Kamera (Pilih salah satu)

**OPSI A - Langsung test kamera (publisher + viewer):**
```bash
ros2 launch mipi_camera_simple camera_full.launch.py
```

**OPSI B - Publisher saja (untuk dipakai package lain):**
```bash
ros2 run mipi_camera_simple camera_publisher
```

**OPSI C - Viewer saja (untuk lihat topic yang sudah ada):**
```bash
ros2 run mipi_camera_simple camera_viewer
```

### 3. Cek Topic Kamera
```bash
ros2 topic list | grep mipi
ros2 topic hz /mipi_camera/image_raw/compressed
```

## 📂 STRUKTUR FILE

```
mipi_camera_simple/
├── mipi_camera_simple/
│   ├── camera_publisher.py    👈 Script untuk akses kamera
│   └── camera_viewer.py        👈 Script untuk lihat kamera
├── launch/
│   ├── camera_publisher.launch.py
│   ├── camera_viewer.launch.py
│   └── camera_full.launch.py  👈 PALING MUDAH, pakai ini
├── README.md                   👈 Dokumentasi lengkap
└── QUICKSTART.md              👈 File ini
```

## 🔧 UBAH RESOLUSI

```bash
# 1280x720
ros2 launch mipi_camera_simple camera_full.launch.py width:=1280 height:=720

# 320x240 (lebih cepat)
ros2 launch mipi_camera_simple camera_full.launch.py width:=320 height:=240
```

## ❌ TROUBLESHOOTING

### Error "hobot_vio not found"
```bash
source /opt/tros/humble/setup.bash
```

### Package tidak ditemukan
```bash
cd /home/vboxuser/Documents/yahboomcar_ws
colcon build --packages-select mipi_camera_simple
source install/setup.bash
```

### Kamera tidak muncul
1. Cek koneksi kabel kamera MIPI
2. Restart robot
3. Pastikan tidak ada program lain yang pakai kamera

## 🎯 CONTOH PENGGUNAAN

### Tes Kamera Cepat
```bash
cd /home/vboxuser/Documents/yahboomcar_ws
source install/setup.bash
ros2 launch mipi_camera_simple camera_full.launch.py
```
Tekan **Q** untuk keluar, **S** untuk screenshot.

### Pakai dengan Package Lain
```bash
# Terminal 1 - Jalankan kamera
ros2 run mipi_camera_simple camera_publisher

# Terminal 2 - Jalankan body tracking (contoh)
ros2 launch body_tracking hobot_body_tracking.launch.py
```

### Subscribe di Python Anda Sendiri
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.sub = self.create_subscription(
            CompressedImage,
            '/mipi_camera/image_raw/compressed',
            self.callback,
            10)

    def callback(self, msg):
        print(f"Dapat gambar: {len(msg.data)} bytes")

rclpy.init()
node = MyNode()
rclpy.spin(node)
```

## 📝 CATATAN PENTING

1. **Selalu source setup.bash** sebelum run:
   ```bash
   source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
   ```

2. **Topic yang dipublikasikan:**
   - `/mipi_camera/image_raw/compressed` (JPEG compressed)

3. **Resolusi default:** 640x480 @ 30fps

4. **Kamera yang didukung:**
   - GC4663 (MIPI camera di RDK X3)
   - Kamera MIPI lainnya (mungkin perlu adjustment)

## 🔄 UPDATE PACKAGE

Kalau mau edit script:
```bash
cd /home/vboxuser/Documents/yahboomcar_ws/src/mipi_camera_simple/mipi_camera_simple
nano camera_publisher.py  # atau camera_viewer.py

# Setelah edit, build ulang
cd /home/vboxuser/Documents/yahboomcar_ws
colcon build --packages-select mipi_camera_simple
source install/setup.bash
```

## 📞 BANTUAN

Lihat README.md untuk dokumentasi lengkap:
```bash
cat /home/vboxuser/Documents/yahboomcar_ws/src/mipi_camera_simple/README.md
```

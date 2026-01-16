# MIPI Camera Simple - RDK Robot Camera Package

Package ROS2 untuk mengakses kamera MIPI pada robot RDK (Horizon Robotics Development Kit).

## Struktur Package

```
mipi_camera_simple/
├── mipi_camera_simple/
│   ├── camera_publisher.py   # Publisher untuk kamera MIPI
│   └── camera_viewer.py       # Viewer untuk melihat gambar
├── launch/
│   ├── camera_publisher.launch.py  # Launch publisher saja
│   ├── camera_viewer.launch.py     # Launch viewer saja
│   └── camera_full.launch.py       # Launch publisher + viewer
└── README.md
```

## Lokasi File

Semua file berada di:
```
/home/vboxuser/Documents/yahboomcar_ws/src/mipi_camera_simple/
```

## Cara Build Package

1. **Build package:**
   ```bash
   cd /home/vboxuser/Documents/yahboomcar_ws
   colcon build --packages-select mipi_camera_simple
   source install/setup.bash
   ```

2. **Atau build semua package:**
   ```bash
   cd /home/vboxuser/Documents/yahboomcar_ws
   colcon build
   source install/setup.bash
   ```

## Cara Menjalankan

### Metode 1: Menggunakan ros2 run (Manual)

**A. Jalankan Publisher (untuk mempublikasikan gambar dari kamera):**
```bash
source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
ros2 run mipi_camera_simple camera_publisher
```

**B. Jalankan Viewer (untuk melihat gambar, buka terminal baru):**
```bash
source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
ros2 run mipi_camera_simple camera_viewer
```

**C. Dengan parameter custom:**
```bash
# Publisher dengan resolusi custom
ros2 run mipi_camera_simple camera_publisher --ros-args \
  -p width:=1280 \
  -p height:=720 \
  -p fps:=30

# Viewer dengan topic custom
ros2 run mipi_camera_simple camera_viewer --ros-args \
  -p topic:=/csi/image_raw/compressed
```

### Metode 2: Menggunakan Launch File (LEBIH MUDAH)

**A. Launch publisher saja:**
```bash
source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
ros2 launch mipi_camera_simple camera_publisher.launch.py
```

**B. Launch viewer saja:**
```bash
source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
ros2 launch mipi_camera_simple camera_viewer.launch.py
```

**C. Launch publisher + viewer sekaligus (PALING MUDAH):**
```bash
source /home/vboxuser/Documents/yahboomcar_ws/install/setup.bash
ros2 launch mipi_camera_simple camera_full.launch.py
```

**D. Launch dengan parameter custom:**
```bash
# Dengan resolusi custom
ros2 launch mipi_camera_simple camera_full.launch.py width:=1280 height:=720 fps:=20
```

## Topic yang Dipublikasikan

- **Publisher topic:** `/mipi_camera/image_raw/compressed`
  - Type: `sensor_msgs/CompressedImage`
  - Format: JPEG compressed

## Cara Melihat Topic

```bash
# List semua topic
ros2 topic list

# Lihat info topic
ros2 topic info /mipi_camera/image_raw/compressed

# Monitor frekuensi publish
ros2 topic hz /mipi_camera/image_raw/compressed

# Echo data (hati-hati, data besar!)
ros2 topic echo /mipi_camera/image_raw/compressed
```

## Cara Test dengan RViz2

```bash
rviz2
```

Lalu:
1. Add -> By Topic -> `/mipi_camera/image_raw/compressed` -> CompressedImage
2. Atau gunakan Image viewer

## Parameter yang Bisa Diubah

### Camera Publisher:
- `width`: Lebar resolusi (default: 640)
- `height`: Tinggi resolusi (default: 480)
- `fps`: Frame per second (default: 30)
- `debug`: Debug mode (default: true)

### Camera Viewer:
- `topic`: Topic yang di-subscribe (default: `/mipi_camera/image_raw/compressed`)
- `show_window`: Tampilkan window (default: true)
- `save_images`: Auto save images (default: false)
- `save_path`: Path untuk save images (default: `/tmp/`)

## Troubleshooting

### Error: "hobot_vio not found"

**Solusi:**
1. Pastikan TROS sudah terinstall:
   ```bash
   ls /opt/tros/
   ```

2. Source TROS environment:
   ```bash
   source /opt/tros/humble/setup.bash
   ```

3. Install TROS jika belum ada (lihat dokumentasi Horizon Robotics)

### Error: "Package 'mipi_camera_simple' not found"

**Solusi:**
```bash
cd /home/vboxuser/Documents/yahboomcar_ws
colcon build --packages-select mipi_camera_simple
source install/setup.bash
```

### Camera tidak terbuka

**Solusi:**
1. Cek koneksi kamera MIPI
2. Pastikan tidak ada program lain yang menggunakan kamera
3. Restart robot

### Viewer tidak muncul

**Solusi:**
1. Pastikan publisher sudah jalan terlebih dahulu
2. Cek apakah topic sudah dipublikasikan: `ros2 topic list`
3. Pastikan X11 forwarding aktif jika remote

## Kontrol Keyboard (di Viewer)

- **q**: Quit/keluar
- **s**: Save screenshot ke /tmp/

## Contoh Penggunaan

### 1. Publikasikan dan lihat kamera:
```bash
ros2 launch mipi_camera_simple camera_full.launch.py
```

### 2. Gunakan dengan resolusi tinggi:
```bash
ros2 launch mipi_camera_simple camera_full.launch.py width:=1280 height:=720
```

### 3. Subscribe dari topic lain:
```bash
ros2 run mipi_camera_simple camera_viewer --ros-args \
  -p topic:=/csi/image_raw/compressed
```

## Integrasi dengan Package Lain

Package ini bisa digunakan bersama package lain seperti:
- `body_tracking` (deteksi tubuh)
- `gesture_control` (kontrol gesture)
- `line_follower_perception` (line following)

Contoh:
```bash
# Terminal 1: Jalankan camera publisher
ros2 run mipi_camera_simple camera_publisher

# Terminal 2: Jalankan body tracking
ros2 launch body_tracking hobot_body_tracking.launch.py
```

## Lisensi

Apache-2.0

## Author

vboxuser

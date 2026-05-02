Hal - hal yang harus di lakukan:
1. tulis di terminal: git clone https://github.com/eDarrrrr/ojt2026f1tenth_ws.git
2. colcon build dulu workspace
3. Download packages packages nya
   ```bash
     sudo apt install -y \
      ros-humble-ackermann-steering-controller \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-controller-manager \
      ros-humble-ign-ros2-control \
      ros-humble-ros-gz \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-interfaces
   
6. bisa di coba ros2 launch Tutotialf1tenth gazebo.launch.py

(note: jika di wsl jalankan command
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```
terlebih dahulu)

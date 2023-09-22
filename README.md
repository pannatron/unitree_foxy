# Unitree Foxy Project

โปรเจกต์นี้มีจุดประสงค์เพื่อทำการทดสอบและพัฒนาฟังก์ชันต่าง ๆ บน Unitree Robot โดยใช้ ROS 2.

## ขั้นตอนการติดตั้ง

1. โคลน repository จาก GitHub:
```bash
git clone https://github.com/pannatron/unitree_foxy.git
```

เข้าไปในโฟลเดอร์ของโปรเจกต์ และสร้างโฟลเดอร์ src:
```bash
cd unitree_foxy
mkdir src
```

คัดลอกหรือย้ายโฟลเดอร์ gscam2 และ ros2_shared ไปยังโฟลเดอร์ src:
```bash
cp -r path_to_gscam2 path_to_ros2_shared src/
```
Source ไฟล์ setup.bash เพื่ออัพเดท environment ของ ROS 2:
```bash
colcon build ;source install/setup.bash
```
# การทดสอบ
เพื่อทำการทดสอบโปรแกรม, ให้ใช้คำสั่งต่อไปนี้:
```bash
ros2 launch gscam2 node_param_launch.py
```
ตอนนี้, คุณควรจะสามารถดูฟีดวิดีโอจากกล้องและรับข้อมูลจาก Robot ได้.

#Unitree_control_ros2

```bash
https://gist.github.com/dbaldwin/feb0d279c67e0bcb191d2b366f867a84
```

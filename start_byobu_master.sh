#!/bin/bash

echo "Starting the byobu setup..."

# สร้าง session แรกชื่อ "video_source" และตั้งชื่อ window แรกเป็น "front_cam"
echo "Setting up 'video_source' session..."
byobu new-session -d -s video_source -n 'udp_front_cam'
byobu send-keys -t video_source:0 'cd .. ;./et_config.sh ' C-m
sleep 0.5
byobu send-keys -t video_source:0 '123321123' C-m
sleep 1
byobu send-keys -t video_source:0 'ssh unitree@192.168.123.13' C-m
sleep 2
byobu send-keys -t video_source:0 '123' C-m
sleep 1
byobu send-keys -t video_source:0 'cd UnitreecameraSDK/;./bins/example_putImagetrans' C-m
sleep 2

# สร้าง session "image_processing" และตั้งชื่อ windows ด้านภายใน
echo "Setting up 'image_processing' session..."
byobu new-session -d -s image_processing -n 'gscam2_front'
byobu send-keys -t image_processing:0 'ros2 launch gscam2 node_param_launch.py' C-m
sleep 0.5

byobu new-window -t image_processing:1 -n 'hand_detection'
byobu send-keys -t image_processing:1 'ros2 run hand_detection hand_detection_node.py' C-m
sleep 0.5

byobu new-window -t image_processing:2 -n 'aruco_distance'
byobu send-keys -t image_processing:2 'ros2 run hand_detection  aruco_distance_estimator.py' C-m
sleep 0.5

# สร้าง session "controller_manager" และตั้งชื่อ windows ด้านภายใน
echo "Setting up 'controller_manager' session..."
byobu new-session -d -s controller_manager -n 'ros2_udp_highlevel'
#byobu send-keys -t controller_manager:0 'ros2 run unitree_legged_real ros2_udp highlevel' C-m
sleep 0.5

byobu new-window -t controller_manager:1 -n 'ros2_walk'
byobu send-keys -t controller_manager:1 'ros2 run unitree_legged_real ros2_walk_example' C-m
sleep 0.5

echo "Byobu setup completed. You can now attach to any session."


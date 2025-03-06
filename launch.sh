#!/bin/bash

# 启动 urg_node_drive（使用 xterm）
xterm -title "urg_node_drive" -e "bash -c 'source install/setup.sh; ros2 run urg_node urg_node_driver --ros-args --params-file /home/autodrive/laser_ws/src/urg_node-ros2-devel/launch/urg_node_ethernet.yaml; exec bash'" &

sleep 1
# 启动 pointcloud_2d_to_3d
xterm -title "pointcloud_2d_to_3d" -e "bash -c 'source install/setup.sh; ros2 run urg_node pointcloud_2d_to_3d; exec bash'" &

sleep 1
# 启动 imu
xterm -title "imu" -e "bash -c 'source install/setup.sh; ros2 launch gnss_imu_sim imu_driver_launch.py; exec bash'" &

sleep 1
# 启动 RViz2
xterm -title "Starting RViz2..." -e "bash -c 'rviz2; exec bash'" &

# 当前终端启动 keyboard_serial_node
sleep 1
echo "===== 启动键盘控制节点（当前终端） ====="
. install/setup.sh
# ros2 run keyboard_serial keyboard_serial_node
ros2 run point_cloud_subscriber point_cloud_subscriber
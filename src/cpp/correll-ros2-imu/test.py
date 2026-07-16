
import sys
sys.path.append("build")

import ros2_imu_node_py

ros2_imu_node_py.spin_rclcpp(2)

imu_publisher = ros2_imu_node_py.ros2_imu_node("imu_test_topic")

imu_publisher.push_data_to_deque([0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0,0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
import time

while True:
    time.sleep(1)
    imu_publisher.push_data_to_deque([0, 0, 0, 1], [9, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0,0], [0, 0, 0, 0, 0, 0, 0, 0, 0])


import sys
sys.path.append("build")

import ros2_imu_node_py

ros2_imu_node_py.spin_rclcpp()

imu_publisher = ros2_imu_node_py.ros2_imu_node("test_topic")

imu_publisher.test_print_data([0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0,0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
import time

while True:
    time.sleep(1)
    imu_publisher.test_print_data([0, 0, 0, 1], [9, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0,0], [0, 0, 0, 0, 0, 0, 0, 0, 0])

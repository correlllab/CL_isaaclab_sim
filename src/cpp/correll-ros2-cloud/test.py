import sys
sys.path.append("build")

import ros2_point_cloud_node_py

ros2_point_cloud_node_py.spin_rclcpp()
node = ros2_point_cloud_node_py.ros2_point_cloud_node("test")
import torch
tensor = torch.rand((6000,), dtype=torch.float, device="cpu")
print(tensor)
print(tensor.flatten().shape)
import time

while True:
    node.push_data_to_deque(tensor.data_ptr(), tensor.shape[0])
    time.sleep(1)



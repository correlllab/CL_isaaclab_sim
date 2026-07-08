import sys
sys.path.append("build")

import ros2_point_cloud_node_py

ros2_point_cloud_node_py.spin_rclcpp()
node = ros2_point_cloud_node_py.ros2_point_cloud_node(["test"])
import torch
tensor = torch.rand((6000,), dtype=torch.float, device="cuda")
print(tensor)
print(tensor.flatten().shape)
import time

while True:
    print(tensor[0])
    print(tensor[1])
    print(tensor[2])
    print(tensor[3])
    node.push_data_to_deque(tensor.data_ptr(), tensor.shape[0], "test")
    time.sleep(1)



import torch
import sys
from PIL import Image
sys.path.append("build")
import time
#import turbo_encoder_py
#import turbo_encoder_py
#encoder = turbo_encoder_py.turbo_encoder("test")

import ros2_camera_node_py

ros2_camera_node_py.spin_rclcpp()
node = ros2_camera_node_py.ros2_camera_node(["/realsense/left_hand/color/image_raw/compressed", "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed"])

#turbo_encoder_py.turbo_encoder()
import torchvision.transforms as transforms
image = Image.open('iceland.avif')
image_rgb = image.convert("RGB")
image_depth = image.convert("L")
transform = transforms.Compose([transforms.PILToTensor()])
#
rgb_img_tensor = transform(image_rgb)
depth_img_tensor = transform(image_depth)
rgb_img_tensor = rgb_img_tensor

breakpoint()

while True:
    time.sleep(1)
    node.push_data_to_deque(rgb_img_tensor.data_ptr(), rgb_img_tensor.shape[2], rgb_img_tensor.shape[1], "/realsense/left_hand/color/image_raw/compressed", "INTERLEAVED")
    node.push_data_to_deque(depth_img_tensor.data_ptr(), depth_img_tensor.shape[2], depth_img_tensor.shape[1], "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "INTERLEAVED")

#encoder.encode(depth_img_tensor.data_ptr(), depth_img_tensor.shape[2], depth_img_tensor.shape[1])
#depth_img_tensor = depth_img_tensor.reshape(3000, 2000, 1).repeat(1, 1, 3).to("cuda")
#breakpoint()
##import nvjpeg_encoder_py
##nvjpeg_encoder_py.nvjpeg_encoder()
#import ros2_camera_node_py
#
#ros2_camera_node_py.spin_rclcpp()
#node = ros2_camera_node_py.ros2_camera_node(["/realsense/left_hand/color/image_raw/compressed", "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed"])
#
#import time
#
#while True:
#    node.push_data_to_deque(rgb_img_tensor.data_ptr(), rgb_img_tensor.shape[0], rgb_img_tensor.shape[1], "/realsense/left_hand/color/image_raw/compressed", "PLANAR")
#    node.push_data_to_deque(depth_img_tensor.data_ptr(), depth_img_tensor.shape[0], depth_img_tensor.shape[1], "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "PLANAR")
#
#    time.sleep(1)
#    pass
#
##import ros2_nvjpeg_compressed_image_publisher_py
##tensor = torch.randint(256, (1280, 720, 3), device='cuda')
##tensor2 = torch.randint(256, (1280, 720, 3), device='cuda')
##tensor3 = torch.randint(256, (1280, 720, 3), device='cuda')
##tensor4 = torch.randint(256, (1280, 720, 3), device='cuda')
##
##gray_scale_tensor = torch.randint(256, (60, 60), device='cuda')
##import numpy as np
##nvjpegInterface = nvjpeg_encoder_py.nvjpeg_encoder(["color", "color1"])
##image = Image.fromarray(gray_scale_tensor.cpu().numpy().astype(np.uint8))
##image.save("test_python.jpeg")
##test = []
##breakpoint()
##gray_scale_tensor_3_channel = torch.stack([gray_scale_tensor, gray_scale_tensor, gray_scale_tensor], dim=-1)
##gray_ptr = gray_scale_tensor_3_channel.data_ptr()
##height = gray_scale_tensor_3_channel.shape[0]
##width = gray_scale_tensor_3_channel.shape[1]
##
##image2 = Image.fromarray(gray_scale_tensor_3_channel.cpu().numpy().astype(np.uint8))
##image2.save("test_python2.jpeg")
##breakpoint()
##nvjpegInterface.encode_image_buffer(gray_scale_tensor.data_ptr(), gray_scale_tensor.shape[1], gray_scale_tensor.shape[0], "color", [])
###nvjpegInterface.encode_image_buffer(gray_ptr, width, height, "color", test)
##breakpoint()
##ptr = tensor.data_ptr()
##height = tensor.shape[0]
##width = tensor.shape[1]
##print(f"{ptr=}")
##
##ptr2 = tensor2.data_ptr()
##height2 = tensor2.shape[0]
##width2 = tensor2.shape[1]
##
##ptr3 = tensor3.data_ptr()
##height3 = tensor3.shape[0]
##width3 = tensor3.shape[1]
##ptr4 = tensor4.data_ptr()
##height4 = tensor4.shape[0]
##width4 = tensor4.shape[1]
##test = []
##nvjpegInterface.encode_image_buffer(ptr, width, height, "/realsense/left_hand/color/image_raw/compressed")
##ros2_nvjpeg_compressed_image_publisher_py.spin_rclcpp()
##ros2_nvjpeg_interface = ros2_nvjpeg_compressed_image_publisher_py.ros2_nvjpeg_compressed_image_publisher()
##import time
###try:
##while True:
##
##
##    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/left_hand/color/image_raw/compressed", ptr, width, height)
##    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/right_hand/color/image_raw/compressed", ptr2, width2, height2)
##    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", ptr3, width3, height3)
##
##    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/right_hand/aligned_depth_to_color/image_raw/compressed", ptr4, width4, height4)
##    time.sleep(0.01)
##
#
##except Exception as e:
##    import traceback
##    traceback.print_exc()
##    print(e)
##
##
##print("test1")
#
##try:
##    ros2_nvjpeg_interface.test_writer_thread("/realsense/left_hand/color/image_raw/compressed", ptr, width, height)
##
##except Exception as e:
##    print(e)
##    import traceback
##    traceback.print_exc()
##    print("test")
##breakpoint(
##ros2_nvjpeg_interface.test_reader_thread()
##nvjpegInterface.start_reader_thread(ptr, width, height, ptr2, width2, height2, ptr3, width3, height3, ptr4, width4, height4):W:
##
##for i in range(10):
##    nvjpegInterface.write_on_thread()
##
#

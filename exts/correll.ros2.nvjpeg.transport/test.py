import torch
import sys
sys.path.append("../build")

#import nvjpeg_encoder_py
import ros2_nvjpeg_compressed_image_publisher_py
tensor = torch.randint(256, (1280, 720, 3), device='cuda')
tensor2 = torch.randint(256, (1280, 720, 3), device='cuda')
tensor3 = torch.randint(256, (1280, 720, 3), device='cuda')
tensor4 = torch.randint(256, (1280, 720, 3), device='cuda')

#nvjpegInterface = nvjpeg_encoder_py.nvjpeg_encoder() 
ptr = tensor.data_ptr()
height = tensor.shape[0]
width = tensor.shape[1]
print(f"{ptr=}")

ptr2 = tensor2.data_ptr()
height2 = tensor2.shape[0]
width2 = tensor2.shape[1]

ptr3 = tensor3.data_ptr()
height3 = tensor3.shape[0]
width3 = tensor3.shape[1]
ptr4 = tensor4.data_ptr()
height4 = tensor4.shape[0]
width4 = tensor4.shape[1]
test = []
#nvjpegInterface.encode_image_buffer(ptr, width, height, "/realsense/left_hand/color/image_raw/compressed")
ros2_nvjpeg_compressed_image_publisher_py.spin_rclcpp()
ros2_nvjpeg_interface = ros2_nvjpeg_compressed_image_publisher_py.ros2_nvjpeg_compressed_image_publisher()
import time
#try:
while True:


    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/left_hand/color/image_raw/compressed", ptr, width, height)
    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/right_hand/color/image_raw/compressed", ptr2, width2, height2)
    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", ptr3, width3, height3)

    ros2_nvjpeg_interface.write_to_publish_thread_safe("/realsense/right_hand/aligned_depth_to_color/image_raw/compressed", ptr4, width4, height4)
    time.sleep(0.01)
#

#except Exception as e:
#    import traceback
#    traceback.print_exc()
#    print(e)
#
#
#print("test1")

#try:
#    ros2_nvjpeg_interface.test_writer_thread("/realsense/left_hand/color/image_raw/compressed", ptr, width, height)
#
#except Exception as e:
#    print(e)
#    import traceback
#    traceback.print_exc()
#    print("test")
#breakpoint(
#ros2_nvjpeg_interface.test_reader_thread()
#nvjpegInterface.start_reader_thread(ptr, width, height, ptr2, width2, height2, ptr3, width3, height3, ptr4, width4, height4):W:
#
#for i in range(10):
#    nvjpegInterface.write_on_thread()
#



#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-camera/Ros2CameraNode.hpp>

Ros2CameraNode::Ros2CameraNode(std::vector<std::string> topics) :rclcpp::Node("correllRos2CameraNode") {

  for (auto& topic : topics) {

    mDequeMap.try_emplace(topic, std::make_unique<ImageDeque>());
    if (topic.find("depth") != std::string::npos) {
      mEncoderMap.try_emplace(topic, std::make_unique<NvjpegEncoder>(DataType::DEPTH));
    
    } else {
      mEncoderMap.try_emplace(topic, std::make_unique<NvjpegEncoder>(DataType::RGB));
    
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 10);

    mPublisherMap.try_emplace(topic, pub);
    mPool.enqueue(std::bind(&Ros2CameraNode::ReaderThread, this, topic));
  }

}

void Ros2CameraNode::PushDataToDeque(long dataPtr, int width, int height, std::string topic, std::string structure) {
  ImageData data{dataPtr, width, height, Structure::PLANAR};
  if (structure == "INTERLEAVED") {
    data.structure = Structure::INTERLEAVED;
  
  }

  mDequeMap[topic]->EmplaceFront(data);
  mDequeMap[topic]->SemaphoreRelease();

}

void Ros2CameraNode::ReaderThread(std::string topic) {
  while (true) {

    ImageData latest = mDequeMap[topic]->PopBack();
    std::vector<unsigned char> buffer = mEncoderMap[topic]->Encode(latest);
    std_msgs::msg::Header header;
    sensor_msgs::msg::CompressedImage msg;

    header.stamp = this->now();
    header.frame_id = "test";

    msg.header = header;
    msg.format = "jpeg";
    msg.data = buffer;

    mPublisherMap[topic]->publish(msg);

    mDequeMap[topic]->SemaphoreAcquire(); 

  }

}


Ros2CameraNode::~Ros2CameraNode() {}

void spin_rclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_camera_node_py, m) {
  pybind11::class_<Ros2CameraNode, std::shared_ptr<Ros2CameraNode>>(m, "ros2_camera_node").def(pybind11::init<std::vector<std::string>>()).def("push_data_to_deque", &Ros2CameraNode::PushDataToDeque);

  m.def("spin_rclcpp", &spin_rclcpp);
};

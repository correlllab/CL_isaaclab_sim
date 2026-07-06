#include <pybind11/pybind11.h>
#include <correll-ros2-nvjpeg-transport/Ros2NvjpegTransport.hpp>



Ros2NvjpegTransportNode::Ros2NvjpegTransportNode(std::vector<std::string> topics) : rclcpp::Node("Ros2NvjpegCompressedImageTransportNode"), mThreadPool(4) {

  std::shared_ptr<NvjpegEncoder> rgbNvjpegEncoder = std::make_shared<nvjpegEncoder>();
  for (auto& topic : topics) {
  
  }

}


ros2NvjpegCompressedImageTransportNode::ros2NvjpegCompressedImageTransportNode(std::vector<std::string> topicNames) : rclcpp::Node("correllIsaacLabRos2NvjpegCompressedImageTransportNode"), threadPool(4) {

  topics = topicNames;
  std::vector<std::string> colorTopics;
  std::vector<std::string> depthTopics;

  for (auto& topic : topics) {
    if (topic.find("aligned_depth") != std::string::npos) {
      depthTopics.push_back(topic);
      std::cout << "depth topic: " << topic << "\n";
    } else if (topic.find("/color") != std::string::npos) {
      colorTopics.push_back(topic);
      std::cout << "color topic: " << topic << "\n";
    } else {
      std::cout << "no match: " << topic.c_str() << "\n";
    }

  }

  m_nvjpegColorEncoder = std::make_shared<nvjpegEncoder>(colorTopics);
  m_nvjpegGrayEncoder = std::make_shared<nvjpegEncoder>(depthTopics);
  for (auto& topic : topics) {
    std::cout << topic.c_str() << "\n";
    std::shared_ptr<imageData> data = std::make_shared<imageData>(0, 0, 0);

    map[topic] = data;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    threadPool.enqueue(std::bind(&ros2NvjpegCompressedImageTransportNode::initializeCompressionPublicationThreadThreadSafe, this, topic));
  }

}

ros2NvjpegCompressedImageTransportNode::~ros2NvjpegCompressedImageTransportNode() {

  ;

}

//void ros2NvjpegCompressedImageTransportNode::initializeCompressionPublicationThreadThreadSafe(std::string topicKey) {
//  std::cout <<"starting thread sleeping" << "\n";
//  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//  std::cout <<"initializing thread publisher" << "\n";
//  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(topicKey, 10);
//  std::cout <<"initializing dataObj sharedptr" << "\n";
//  std::shared_ptr<imageData> dataObj = map[topicKey];
//  std::cout <<"initialziing nvImage object" << "\n";
//  nvjpegImage_t nvImage;
//  std::cout <<"imnitializng jpegBuffer vector" << "\n";
//  std::vector<unsigned char> jpegBuffer;
//  while (true) {
//    std::cout <<"initializng unique lock" << "\n";
//    std::unique_lock<std::mutex> guard(dataObj->mutex);
//    std::cout <<"initializing cv.wait readyToWrite" << "\n";
//    dataObj->cv.wait(guard, [dataObj]() {return !dataObj->readyToWrite;});
//
//    std::cout <<"if topic find aligned depth" << "\n";
//    if (topicKey.find("aligned_depth") != std::string::npos) {
//
//      m_nvjpegGrayEncoder->encodeImageBuffer(dataObj->dataPtr, dataObj->width, dataObj->height, topicKey, jpegBuffer);
//      std::cout <<"aligned depth topic found" << "\n";
//    } else if (topicKey.find("/color") != std::string::npos) {
//
//      m_nvjpegColorEncoder->encodeImageBuffer(dataObj->dataPtr, dataObj->width, dataObj->height, topicKey, jpegBuffer);
//      std::cout <<"color topic found" << "\n";
//    } else {
//      std::cout << "no encoder match" << "\n";
//
//    }
//    std::cout << "jpeg size after: " << jpegBuffer.size() << "\n";
//
//
//    std::cout << "initailzing ros2 msggg" << "\n";
//    dataObj->compMsgHeader.stamp = this->now();
//    dataObj->compMsgHeader.frame_id = topicKey;
//    dataObj->compImageMsg.header = dataObj->compMsgHeader;
//    dataObj->compImageMsg.format = "jpeg";
//    dataObj->compImageMsg.data = jpegBuffer;
//
//    std::cout << "publishing ros2 msg" << "\n";
//    publisher->publish(dataObj->compImageMsg);
//
//    std::cout << "doing guard.unlock now" << "\n";
//    guard.unlock();
//    std::cout << "updating readyToWrite" << "\n";
//    dataObj->readyToWrite=true;
//    std::cout << "notifiyng condition variable" << "\n";
//    dataObj->cv.notify_all();
//  }
//
//}
//
//void ros2NvjpegCompressedImageTransportNode::writeToPublishThreadSafe(std::string topicKey, long dataPtr, int width, int height) {
//  std::cout << "acquring shared ptr interface" << "\n";
//  std::shared_ptr<imageData> dataObj = map[topicKey];
//  std::cout << "doing unique lock for writer" << "\n";
//  std::unique_lock<std::mutex> guard(dataObj->mutex);
//  std::cout << "doing cv.wait for writer" << "\n";
//  dataObj->cv.wait(guard, [dataObj]() {return dataObj->readyToWrite;});
//  std::cout << "updating dataptr in writer to: " << dataPtr << "\n";
//  dataObj->dataPtr = dataPtr;
//  std::cout << "updating width in writer to: " << width << "\n";
//  dataObj->width = width;
//  std::cout << "updating height in writer to: " << height << "\n";
//  dataObj->height = height;
//  std::cout << "guard.unlcok in writer" << "\n";
//  guard.unlock();
//  std::cout << "readytoWrite false in writer" << "\n";
//  dataObj->readyToWrite = false;
//  std::cout << "condition variable notifyy all in writr" << "\n";
//  dataObj->cv.notify_all();
//
//}


void spinRclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_nvjpeg_compressed_image_publisher_py, m) {

  pybind11::class_<ros2NvjpegCompressedImageTransportNode, std::shared_ptr<ros2NvjpegCompressedImageTransportNode>>(m, "ros2_nvjpeg_compressed_image_publisher").def(pybind11::init<std::vector<std::string>>()).def("initialize_compression_publication_thread_thread_safe", &ros2NvjpegCompressedImageTransportNode::initializeCompressionPublicationThreadThreadSafe).def("write_to_publish_thread_safe", &ros2NvjpegCompressedImageTransportNode::writeToPublishThreadSafe);

  m.def("spin_rclcpp", &spinRclcpp);
};

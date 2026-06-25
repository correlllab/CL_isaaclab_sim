
#include <pybind11/pybind11.h>
#include <ros2_nvjpeg_compressed_image_publisher.hpp>


ros2NvjpegCompressedImagePublisher::ros2NvjpegCompressedImagePublisher() : rclcpp::Node("correllIsaacLabRos2NvjpegCompressedImageTransportNode"), threadPool(10), topics{"/realsense/left_hand/color/image_raw/compressed", "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "/realsense/right_hand/color/image_raw/compressed", "/realsense/right_hand/aligned_depth_to_color/image_raw/compressed"} {

  //nvjpegEncoder nvjpeg_encoder;

  //nvjpegEncoderState_t nvEncState;
  //stringPublisher = this->create_publisher<std_msgs::msg::String>("test", 10);
  //threadPool.enqueue(std::bind(&ros2NvjpegCompressedImagePublisher::test_reader_thread, this, "test"));

  m_nvjpegEncoder = std::make_shared<nvjpegEncoder>();
  for (auto& topic : topics) {
    std::cout << topic.c_str() << "\n";
    std::shared_ptr<imageData> data = std::make_shared<imageData>(0, 0, 0);

    map[topic] = data;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    threadPool.enqueue(std::bind(&ros2NvjpegCompressedImagePublisher::initializeCompressionPublicationThreadThreadSafe, this, topic));
  }


//  nvjpegCreateSimple(&m_nvHandle);
//  nvjpegEncoderParamsCreate(m_nvHandle, &m_nvEncParams, m_cudaStream);
//  nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);
//
//  for (auto& topic : topics) {
//    simData data{topic};
//    nvjpegImageStream nvjpeg {topic};
//
//    m_simDatas.push_back(data);
//    m_nvjpegImages.push_back(nvjpeg);
//  
//  }
}

ros2NvjpegCompressedImagePublisher::~ros2NvjpegCompressedImagePublisher() {

//  nvjpegEncoderParamsDestroy(m_nvEncParams);
//  nvjpegDestroy(m_nvHandle);

}

void ros2NvjpegCompressedImagePublisher::initializeCompressionPublicationThreadThreadSafe(std::string topicKey) {
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(topicKey, 10);
  std::cout << "starting test_reader_thread execution" << "\n";
  std::cout << "tid: " << std::this_thread::get_id() << "\n";
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::shared_ptr<imageData> dataObj = map[topicKey];
  //nvjpegEncoderState_t nvEncState;
  //nvjpegEncoderStateCreate(m_nvjpegEncoder->m_nvHandle, &nvEncState, m_nvjpegEncoder->m_cudaStream);
  nvjpegImage_t nvImage;
  std::vector<unsigned char> jpegBuffer;
  while (true) {
    std::unique_lock<std::mutex> guard(dataObj->mutex);
    std::cout << "waiting!" << "\n";
    dataObj->cv.wait(guard, [dataObj]() {return !dataObj->readyToWrite;});

    m_nvjpegEncoder->encodeImageBuffer(dataObj->dataPtr, dataObj->width, dataObj->height, topicKey, jpegBuffer);

    std::cout << "jpeg size after: " << jpegBuffer.size() << "\n";
    dataObj->compMsgHeader.stamp = this->now();
    dataObj->compMsgHeader.frame_id = topicKey;
    dataObj->compImageMsg.header = dataObj->compMsgHeader;
    dataObj->compImageMsg.format = "jpeg";
    dataObj->compImageMsg.data = jpegBuffer;

    publisher->publish(dataObj->compImageMsg);
    //std::cout << "just encoded image" << "\n";
    //dataObj->stringMsg.data = std::to_string(dataObj->dataPtr);
    //std::cout << "jpegBuffer size after compression : " << jpegBuffer.size() << "\n"; 
    //testPublisher->publish(dataObj->stringMsg);

    guard.unlock();
    dataObj->readyToWrite=true;
    dataObj->cv.notify_all();
  }

   //std::this_thread::sleep_for()
    //std::cout << "in while true loop" << "\n";
    //std::unique_lock<std::mutex> guard(imageData.mutex);
    //std::cout << "initialized guard" << "\n";
    //imageData.cv.wait(guard, []() {return !readyToWrite;});
    //std::cout << "waiting for cv" << "\n";

    //std::cout << "tid : " << std::this_thread::get_id() << "; dataPtr: " << imageData.dataPtr << "\n";
    //std::cout << "tid : " << std::this_thread::get_id() << "; width: " << imageData.width << "\n";
    //std::cout << "tid : " << std::this_thread::get_id() << "; height: " << imageData.height << "\n";

    //guard.unlock();

    //std::cout << "guard unlocked" << "\n";
    //readyToWrite = true;
    //std::cout << "readyToWrite true" << "\n";
    //imageData.cv.notify_all();
    //std::cout << "notificaiton" << "\n";

}

long inc = 0;
void ros2NvjpegCompressedImagePublisher::writeToPublishThreadSafe(std::string topicKey, long dataPtr, int width, int height) {
  std::shared_ptr<imageData> dataObj = map[topicKey];
  std::unique_lock<std::mutex> guard(dataObj->mutex);
  dataObj->cv.wait(guard, [dataObj]() {return dataObj->readyToWrite;});
  std::cout << "dataPtr should now be: " << dataPtr << "\n";
  dataObj->dataPtr = dataPtr;
  std::cout << "width should now be: " << width << "\n";
  dataObj->width = width;
  std::cout << "height should now be: " << height << "\n";
  dataObj->height = height;
  guard.unlock();
  dataObj->readyToWrite = false;
  dataObj->cv.notify_all();
  inc++;

}


void spinRclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_nvjpeg_compressed_image_publisher_py, m) {

  pybind11::class_<ros2NvjpegCompressedImagePublisher, std::shared_ptr<ros2NvjpegCompressedImagePublisher>>(m, "ros2_nvjpeg_compressed_image_publisher").def(pybind11::init<>()).def("initialize_compression_publication_thread_thread_safe", &ros2NvjpegCompressedImagePublisher::initializeCompressionPublicationThreadThreadSafe).def("write_to_publish_thread_safe", &ros2NvjpegCompressedImagePublisher::writeToPublishThreadSafe);

  m.def("spin_rclcpp", &spinRclcpp);
};


#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <correll-common/ThreadPool.hpp>
#include <correll-ros2-nvjpeg-transport/ImageQueue.hpp>
#include <correll-ros2-nvjpeg-transport/NvjpegEncoder.hpp>

class Ros2NvjpegTransportNode : public rclcpp::Node {
  Ros2NvjpegTransportNode(std::vector<std::string> topics);
  ~Ros2NvjpegTransportNode();

  std::unordered_map<std::string, std::shared_ptr<NvjpegEncoder>> mEncoders;
  std::unordered_map<std::string, std::unique_ptr<ImageQueue>> mDeques;
  std::unique_ptr<ThreadPool> mThreadPool;

  void PushImageToQueue(std::string topic, Image img);
  void StartTransportThread(std::string topic);



}


//struct imageData {
//  long dataPtr;
//  int width, height;
//  std::mutex mutex;
//  std::condition_variable cv;
//  bool readyToWrite;
//
//  sensor_msgs::msg::CompressedImage compImageMsg;
//  std_msgs::msg::Header compMsgHeader;
//  std_msgs::msg::String stringMsg;
//
//  imageData(long ptr, int w, int h) : dataPtr(ptr), width(w), height(h), compImageMsg(sensor_msgs::msg::CompressedImage()), readyToWrite(true), stringMsg(std_msgs::msg::String()) {};
//
//} ;
//
//
//
//class ros2NvjpegCompressedImageTransportNode : public rclcpp::Node {
//
//  public:
//
//    std::unordered_map<std::string, std::shared_ptr<imageData>> map;
//    ros2NvjpegCompressedImageTransportNode(std::vector<std::string> topicNames);
//    ~ros2NvjpegCompressedImageTransportNode();
//
//    std::shared_ptr<nvjpegEncoder> m_nvjpegColorEncoder;
//    std::shared_ptr<nvjpegEncoder> m_nvjpegGrayEncoder;
//
//    ThreadPool threadPool;
//    std::vector<std::string> topics;
//    void writeToPublishThreadSafe(std::string topicKey, long dataPtr, int width, int height);
//
//    void initializeCompressionPublicationThreadThreadSafe(std::string topicKey);
//
//};

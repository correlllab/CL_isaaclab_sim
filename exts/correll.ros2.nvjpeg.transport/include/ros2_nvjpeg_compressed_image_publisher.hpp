
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <unordered_map>
#include <nvjpeg_encoder.hpp>
struct imageData {
  long dataPtr;
  int width, height;
  std::mutex mutex;
  std::condition_variable cv;
  bool readyToWrite;

  //nvjpegEncoderState_t nvEncState;
  //nvjpegImage_t nvImage;

  sensor_msgs::msg::CompressedImage compImageMsg;
  std_msgs::msg::Header compMsgHeader;
  std_msgs::msg::String stringMsg;

  imageData(long ptr, int w, int h) : dataPtr(ptr), width(w), height(h), compImageMsg(sensor_msgs::msg::CompressedImage()), readyToWrite(true), stringMsg(std_msgs::msg::String()) {};

} ;



class ros2NvjpegCompressedImagePublisher : public rclcpp::Node {

  public:

    struct {
      std::string test;
      std::mutex mutex;
      std::condition_variable cv;
    
    } testIntObject;

    std::unordered_map<std::string, std::shared_ptr<imageData>> map;
    ros2NvjpegCompressedImagePublisher();
    ~ros2NvjpegCompressedImagePublisher();

    std::shared_ptr<nvjpegEncoder> m_nvjpegEncoder;

    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stringPublisher;

    ThreadPool threadPool;
    std::vector<std::string> topics;
    void writeToPublishThreadSafe(std::string topicKey, long dataPtr, int width, int height);

    void initializeCompressionPublicationThreadThreadSafe(std::string topicKey);


    //void initializeCompressionPublicationThreads(std::vector<std::string> topics);  
    //void runCompressPubThread(uint8_t topicIndexInTopics);

};



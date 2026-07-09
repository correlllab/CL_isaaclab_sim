
#include <correll-common/ThreadPool.hpp>
#include <correll-common/ThreadSafeDeque.hpp>

#include <correll-ros2-camera/NvjpegEncoder.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

struct ImageDeque {
  private:
    TQueueConcurrent<ImageData> mInternalDeque;
    std::counting_semaphore<> mInternalSemaphore{0};
  public:

    ImageDeque() {};

    void EmplaceFront(ImageData data) {
      mInternalDeque.emplace_front(data);
    
    }

    ImageData PopBack() {
      ImageData elem = mInternalDeque.pop_back();
      return elem;
    
    }

    void SemaphoreRelease() {
      mInternalSemaphore.release();
    }

    void SemaphoreAcquire() {
      mInternalSemaphore.acquire();
    }

};



class Ros2CameraNode : public rclcpp::Node {
  public:
    Ros2CameraNode(std::vector<std::string> topics);
    ~Ros2CameraNode();

    void PushDataToDeque(long dataPtr, int width, int height, std::string topic, std::string structure);


  private:

    void ReaderThread(std::string topic);


    std::unordered_map<std::string, std::unique_ptr<ImageDeque>> mDequeMap;
    std::unordered_map<std::string, std::unique_ptr<NvjpegEncoder>> mEncoderMap;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> mPublisherMap;
    //ImageDeque mDeque;
    ThreadPool mPool{2};
    //rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mCompressedImagePublisher;
    //NvjpegEncoder mEncoder{DataType::RGB};



};

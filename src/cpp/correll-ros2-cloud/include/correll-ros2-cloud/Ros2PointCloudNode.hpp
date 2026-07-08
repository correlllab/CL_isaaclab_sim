#include <semaphore>

#include <correll-common/ThreadSafeDeque.hpp>
#include <correll-common/ThreadPool.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

struct PointCloudData {
  long dataPtr, numElements;
};

struct PointCloudDeque {
  private:
    TQueueConcurrent<PointCloudData> mInternalDeque;
    std::counting_semaphore<> mInternalSemaphore{0};
  public:

  void EmplaceFront(PointCloudData data) {
    mInternalDeque.emplace_front(data);
  
  }

  PointCloudData PopBack() {
    return mInternalDeque.pop_back();
  
  }

  void SemaphoreRelease() {
    mInternalSemaphore.release();
  }

  void SemaphoreAcquire() {
    mInternalSemaphore.acquire();
  }

};

class Ros2PointCloudNode : public rclcpp::Node {

  public:

    Ros2PointCloudNode(std::string topic);

    uint8_t PushDataToDeque(long dataPtr, long numElements);
  private:

    void ReaderThread();
    PointCloudDeque mDeque;
    ThreadPool mPool{2};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPublisher;

};

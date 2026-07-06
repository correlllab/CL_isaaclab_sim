#include <semaphore>

#include <correll-common/ThreadSafeDeque.hpp>
#include <correll-common/ThreadPool.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

struct ImuData {
  std::array<double, 4> quat_orient;
  std::array<double, 9> orient_covar, ang_vel_covar, lin_acc_covar;
  std::array<double, 3> ang_vel, lin_acc;

};

struct ImuDeque {
  private:
    TQueueConcurrent<ImuData> mInternalDeque;
    std::counting_semaphore<> mInternalSemaphore{0};
  public:

  void EmplaceFront(ImuData data) {
    mInternalDeque.emplace_front(data);
  
  }

  ImuData PopBack() {
    ImuData elem = mInternalDeque.pop_back();
    return elem;
  
  }

  void SemaphoreRelease() {
    mInternalSemaphore.release();
  }

  void SemaphoreAcquire() {
    mInternalSemaphore.acquire();
  }

};

class Ros2ImuNode : public rclcpp::Node {

  public:

    Ros2ImuNode(std::string topic);

    uint8_t PushDataToDeque(std::array<double, 4> quat_orient, std::array<double, 9> orient_covar, std::array<double, 3> ang_vel, std::array<double, 9> ang_vel_covar, std::array<double, 3> lin_acc, std::array<double, 9> lin_acc_covar);
  private:

    void ReaderThread();
    ImuDeque mDeque;
    ThreadPool mPool{2};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPublisher;

};

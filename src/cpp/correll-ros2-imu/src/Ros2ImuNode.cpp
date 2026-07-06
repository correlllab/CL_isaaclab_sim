
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-imu/Ros2ImuNode.hpp>

Ros2ImuNode::Ros2ImuNode(std::string topic) : rclcpp::Node("correllRos2ImuNode") {

  mImuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);

  mPool.enqueue(std::bind(&Ros2ImuNode::ReaderThread, this));

}

uint8_t Ros2ImuNode::PushDataToDeque(std::array<double, 4> quat_orient, std::array<double, 9> orient_covar, std::array<double, 3> ang_vel, std::array<double, 9> ang_vel_covar, std::array<double, 3> lin_acc, std::array<double, 9> lin_acc_covar) {

  ImuData data{quat_orient, orient_covar, ang_vel_covar, lin_acc_covar, ang_vel, lin_acc};
  mDeque.EmplaceFront(data);
  mDeque.SemaphoreRelease();

  return 0;

}

void Ros2ImuNode::ReaderThread() {
  while (true) {
    ImuData latest = mDeque.PopBack();
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "imu_frame";

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = latest.quat_orient[0];
    orientation.y = latest.quat_orient[1];
    orientation.z = latest.quat_orient[2];
    orientation.w = latest.quat_orient[3];

    geometry_msgs::msg::Vector3 angular_velocity, linear_acceleration;

    angular_velocity.x = latest.ang_vel[0];
    angular_velocity.y = latest.ang_vel[1];
    angular_velocity.z = latest.ang_vel[2];

    linear_acceleration.x = latest.lin_acc[0];
    linear_acceleration.y = latest.lin_acc[1];
    linear_acceleration.z = latest.lin_acc[2];

    sensor_msgs::msg::Imu msg;
    msg.header = header;
    msg.orientation = orientation;
    msg.orientation_covariance = latest.orient_covar;
    msg.angular_velocity = angular_velocity;
    msg.angular_velocity_covariance = latest.ang_vel_covar;
    msg.linear_acceleration = linear_acceleration;
    msg.linear_acceleration_covariance = latest.lin_acc_covar;
    mImuPublisher->publish(msg);
    mDeque.SemaphoreAcquire();
  }

}


void spinRclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_imu_node_py, m) {

  pybind11::class_<Ros2ImuNode, std::shared_ptr<Ros2ImuNode>>(m, "ros2_imu_node").def(pybind11::init<std::string>()).def("push_data_to_deque", &Ros2ImuNode::PushDataToDeque);

  m.def("spin_rclcpp", &spinRclcpp);
};

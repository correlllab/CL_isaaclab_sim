
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-cloud/Ros2PointCloudNode.hpp>

Ros2PointCloudNode::Ros2PointCloudNode(std::vector<std::string> topics) : rclcpp::Node("correllRos2PointCloudNode") {

  for (auto& topic : topics) {

    mDequeMap.try_emplace(topic, std::make_unique<PointCloudDeque>());

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

    mPublisherMap.try_emplace(topic, pub);
    mPool.enqueue(std::bind(&Ros2PointCloudNode::ReaderThread, this, topic));
  }

  //mPointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

  //mPool.enqueue(std::bind(&Ros2PointCloudNode::ReaderThread, this));

}

uint8_t Ros2PointCloudNode::PushDataToDeque(long dataPtr, long numElements, std::string topic) {
  PointCloudData data{dataPtr, numElements};

  mDequeMap[topic]->EmplaceFront(data);
  mDequeMap[topic]->SemaphoreRelease();

  return 0;

}

void Ros2PointCloudNode::ReaderThread(std::string topic) {
  while (true) {
    PointCloudData latest = mDequeMap[topic]->PopBack();
    std_msgs::msg::Header header;
    sensor_msgs::msg::PointCloud2 msg;
    sensor_msgs::msg::PointField x;
    sensor_msgs::msg::PointField y;
    sensor_msgs::msg::PointField z;


    x.name = "x";
    x.offset = 0;
    x.datatype = 7;
    x.count = 1;

    y.name = "y";
    y.offset = 4;
    y.datatype = 7;
    y.count = 1; 

    z.name = "z";
    z.offset = 8;
    z.datatype = 7;
    z.count = 1;

    msg.height = 1;
    msg.width = latest.numElements / 3;
    //total number of points, not vlaues

    sensor_msgs::msg::PointField fields[3] = {x, y, z};

    header.stamp = this->now();
    header.frame_id = "lidar_link";

    msg.header = header;
    msg.is_dense = true;
    msg.point_step = 12;
    msg.row_step = (latest.numElements / 3) * 12;

    msg.fields = {x, y, z};

    std::vector<unsigned char> buffer(4 * latest.numElements);
    std::memcpy(buffer.data(), (void*)latest.dataPtr, (4 * latest.numElements));
    msg.data = std::move(buffer);

    mPublisherMap[topic]->publish(msg);
    
    mDequeMap[topic]->SemaphoreAcquire();
  }

}


void spinRclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_point_cloud_node_py, m) {

  pybind11::class_<Ros2PointCloudNode, std::shared_ptr<Ros2PointCloudNode>>(m, "ros2_point_cloud_node").def(pybind11::init<std::vector<std::string>>()).def("push_data_to_deque", &Ros2PointCloudNode::PushDataToDeque);

  m.def("spin_rclcpp", &spinRclcpp);
};

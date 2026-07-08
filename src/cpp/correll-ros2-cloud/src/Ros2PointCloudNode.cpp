
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-cloud/Ros2PointCloudNode.hpp>

Ros2PointCloudNode::Ros2PointCloudNode(std::string topic) : rclcpp::Node("correllRos2PointCloudNode") {

  mPointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

  mPool.enqueue(std::bind(&Ros2PointCloudNode::ReaderThread, this));

}

uint8_t Ros2PointCloudNode::PushDataToDeque(long dataPtr, long numElements) {
  std::cout << "dataPtr: " << dataPtr << "\n";
  std::cout << "size: " << numElements << "\n";
  

  PointCloudData data{dataPtr, numElements};
  mDeque.EmplaceFront(data);
  mDeque.SemaphoreRelease();

  return 0;

}

void Ros2PointCloudNode::ReaderThread() {
  while (true) {
    PointCloudData latest = mDeque.PopBack();
    std_msgs::msg::Header header;
    sensor_msgs::msg::PointCloud2 msg;
    sensor_msgs::msg::PointField x;
    sensor_msgs::msg::PointField y;
    sensor_msgs::msg::PointField z;


    x.name = "x";
    x.offset = 0;
    x.datatype = 7;
    x.count = latest.numElements / 3;

    y.name = "y";
    y.offset = 4;
    y.datatype = 7;
    y.count = latest.numElements / 3;

    z.name = "z";
    z.offset = 8;
    z.datatype = 7;
    z.count = latest.numElements / 3;

    msg.height = 1;
    msg.width = latest.numElements * 4;

    sensor_msgs::msg::PointField fields[3] = {x, y, z};
    std::cout << "typeid fields: " << typeid(fields).name() << "\n";

    header.stamp = this->now();
    header.frame_id = "lidar_link";
    std::cout << "test0" << "\n";

    msg.header = header;

    std::cout << "test1" << "\n";
    msg.fields = {x, y, z};

    std::cout << "test2" << "\n";
    const float* addie = reinterpret_cast<const float*>(latest.dataPtr);
    const float* addie2 = addie + (4*latest.numElements);

    std::vector<float> vec1(latest.numElements);
    std::vector<unsigned char> vec2(4 * latest.numElements);



    std::memcpy((void*)(vec1.data()), (void*)addie, 4*latest.numElements);
    std::memcpy((void*)(vec2.data()), (void*)(vec1.data()), 4*latest.numElements);

    std::cout << "elem0: " << vec2[0] << "\n";
    std::cout << "elem1: " << vec2[1] << "\n";
    std::cout << "elem2: " << vec2[2] << "\n";

    msg.data = vec2;
    std::cout << "vec1 size: " << vec1.size() << "\n";

    //std::cout << "test3" << "\n";
    //std::cout << "test4" << "\n";

    mPointCloudPublisher->publish(msg);
    
    mDeque.SemaphoreAcquire();
  }

}


void spinRclcpp() {
  rclcpp::init(0, nullptr);

}

PYBIND11_MODULE(ros2_point_cloud_node_py, m) {

  pybind11::class_<Ros2PointCloudNode, std::shared_ptr<Ros2PointCloudNode>>(m, "ros2_point_cloud_node").def(pybind11::init<std::string>()).def("push_data_to_deque", &Ros2PointCloudNode::PushDataToDeque);

  m.def("spin_rclcpp", &spinRclcpp);
};

#include <OgnClRos2CppLivoxLidarNodeDatabase.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <chrono>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>
#include <string>
#include <memory>
#include <isaacsim/ros2/bridge/IRos2Bridge.h>
#include <isaacsim/ros2/bridge/Ros2Factory.h>
#include <isaacsim/ros2/bridge/Ros2Types.h>
#include <isaacsim/ros2/bridge/Ros2QoS.h>

namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace livox {
        namespace lidar {

          class OgnClRos2CppLivoxLidarNode  {

            isaacsim::sensors::physx::LidarSensorInterface* mLidarInterface = nullptr;
            std::string mLidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";
            isaacsim::ros2::bridge::Ros2Bridge* mRos2Bridge;
            isaacsim::ros2::bridge::Ros2Factory* mRos2Factory;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle>* mRos2ContextHandlePtr;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2NodeHandle> mRos2NodeHandle;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2PointCloudMessage> mRos2PointCloudMsg;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2Publisher> mRos2PointCloudPublisher;
            carb::tasking::ITasking* mCarbTaskingInterface;
            carb::tasking::TaskGroup mCarbTaskGroup;
            std::chrono::steady_clock mClock;
            double mStartTime;
            public:

              OgnClRos2CppLivoxLidarNode() {}
              ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {

                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);
                state.initInternalState();
                //state.mCarbTaskingInterface->sleepNs(10000000000);

                //state.mCarbTaskingInterface->addTask(carb::tasking::Priority::eHigh, state.mCarbTaskGroup, test); 
              }

              void initInternalState() {
                mLidarInterface = carb::getCachedInterface<isaacsim::sensors::physx::LidarSensorInterface>();

                if (mLidarInterface) {
                  std::cout << "carb::getCachedInterface failed on lidar interface query" << "\n";
                }
                else {
                  std::cout << "lidar interface succesfully retrieved from carb::getCachedInterface" << "\n";
                }
                //std::string lidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";

                if (mLidarInterface->isLidarSensor(mLidarPath.c_str())) {
                  std::cout << "lidar sensor succesuflly detected at: " << mLidarPath.c_str() << "\n";
                }
                else {
                  std::cout << "we should debug dump here" << "\n";
                }
                mRos2Bridge = carb::getCachedInterface<isaacsim::ros2::bridge::Ros2Bridge>();
                mRos2Factory = mRos2Bridge->getFactory();
                mRos2ContextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle>*>(mRos2Bridge->getDefaultContextHandleAddr());
                mRos2NodeHandle = mRos2Factory->createNodeHandle("test", "test", mRos2ContextHandlePtr->get());
                mRos2PointCloudMsg = mRos2Factory->createPointCloudMessage();
                isaacsim::ros2::bridge::Ros2QoSProfile qos;
                const std::string qosProfile = "on-demand";
                isaacsim::ros2::bridge::jsonToRos2QoSProfile(qos, qosProfile);
                mRos2PointCloudPublisher = mRos2Factory->createPublisher(mRos2NodeHandle.get(), "/test/test", mRos2PointCloudMsg->getTypeSupportHandle(), qos);

                mCarbTaskingInterface = carb::getCachedInterface<carb::tasking::ITasking>();
                mCarbTaskingInterface->addTask(carb::tasking::Priority::eHigh, mCarbTaskGroup, std::bind(&OgnClRos2CppLivoxLidarNode::test, this));
              }

                //tasking->sleepNs(10000000000);
                //tasking->addTask(carb::tasking::Priority::eHigh, tasks, test); 
              //static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId);
              //static void release(NodeObj const& node);

              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {


                OgnClRos2CppLivoxLidarNode& state = db.perInstanceState<OgnClRos2CppLivoxLidarNode>();


                //isaacsim::ros2::bridge::Ros2PointCloudMessage pointCloud = state.mFactory.generatePointCloudMessage();

                carb::Float3* carbPointCloud = state.mLidarInterface->getPointCloud(state.mLidarPath.c_str());
                int rows = state.mLidarInterface->getNumRows(state.mLidarPath.c_str());
                int cols = state.mLidarInterface->getNumCols(state.mLidarPath.c_str());
                std::vector<uint8_t> rawPointCloud;

                for (int i = 0; i < (rows * cols); i++) {
                  rawPointCloud.push_back(carbPointCloud[i].x);
                  rawPointCloud.push_back(carbPointCloud[i].y);
                  rawPointCloud.push_back(carbPointCloud[i].z);
                }

                std::cout << rawPointCloud.size() << "\n";
                auto now = state.mClock.now();
                //pointCloudMessage->generateBuffer(std::chrono::duration_cast<std::chrono::seconds>(now - state.mStartTime).count(), "test_frame_id", cols, rows, 12)
                //state.mRos2PointCloudMsg.data = rawPointCloud.data();
                return true;
              }
              void test() {
                while (true) {
                  int size = sizeof(mRos2PointCloudMsg.get()) / sizeof(mRos2PointCloudMsg.get()[0]);
                  std::cout << "size of point cloud to be published: " << size << "\n";
                }
              }
            };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

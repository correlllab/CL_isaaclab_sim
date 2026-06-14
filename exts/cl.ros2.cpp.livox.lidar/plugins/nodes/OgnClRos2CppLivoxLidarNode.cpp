#include <chrono>
#include <string>
#include <memory>

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <isaacsim/ros2/bridge/IRos2Bridge.h>
#include <isaacsim/ros2/bridge/Ros2Factory.h>
#include <isaacsim/ros2/bridge/Ros2Types.h>
#include <isaacsim/ros2/bridge/Ros2QoS.h>

#include <OgnClRos2CppLivoxLidarNodeDatabase.h>

namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace livox {
        namespace lidar {

          class OgnClRos2CppLivoxLidarNode  {

            isaacsim::sensors::physx::LidarSensorInterface* mLidarInterface;
            isaacsim::ros2::bridge::Ros2Bridge* mRos2Bridge;
            isaacsim::ros2::bridge::Ros2Factory* mRos2Factory;
            isaacsim::ros2::bridge::Ros2QoSProfile mQoS;

            std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle>* mRos2ContextHandlePtr;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2NodeHandle> mRos2NodeHandle;
            std::shared_ptr<isaacsim::ros2::bridge::Ros2Publisher> mRos2PointCloudPublisher;

            std::shared_ptr<isaacsim::ros2::bridge::Ros2PointCloudMessage> mRos2PointCloudMsg;

            carb::tasking::ITasking* mCarbTaskingInterface;
            carb::tasking::TaskGroup mCarbTaskGroup;
            std::vector<carb::tasking::Future<>> mCarbTasks;


            const std::string mLidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";
            const std::string mQoSProfile = R"({
              "history": "keepLast",
              "depth": 10,
              "reliability": "bestEffort",
              "durability": "volatile",
              "deadline": 5.0,
              "lifespan": 10.0,
              "liveliness": "automatic",
              "leaseDuration": 9.0
            })";

            const std::string mLidarFrameId = "test_frame_id";
            
            std::chrono::steady_clock mClock;
            std::chrono::steady_clock::time_point mStartTime = mClock.now();

            bool running = true;

            public:

              OgnClRos2CppLivoxLidarNode() {

                mLidarInterface = carb::getCachedInterface<isaacsim::sensors::physx::LidarSensorInterface>();
                mRos2Bridge = carb::getCachedInterface<isaacsim::ros2::bridge::Ros2Bridge>();
                mRos2Factory = mRos2Bridge->getFactory();
                mRos2ContextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle>*>(mRos2Bridge->getDefaultContextHandleAddr());
                mRos2NodeHandle = mRos2Factory->createNodeHandle("test", "test", mRos2ContextHandlePtr->get());

                mRos2PointCloudMsg = mRos2Factory->createPointCloudMessage();

                isaacsim::ros2::bridge::Ros2QoSProfile qos;
                isaacsim::ros2::bridge::jsonToRos2QoSProfile(qos, mQoSProfile);

                mRos2PointCloudPublisher = mRos2Factory->createPublisher(mRos2NodeHandle.get(), "/test/test", mRos2PointCloudMsg->getTypeSupportHandle(), qos);
                mCarbTaskingInterface = carb::getCachedInterface<carb::tasking::ITasking>();
             
              }
              ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {

                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);
                state.initInternalState();
              }

              void initInternalState() {
                if (mLidarInterface) {
                  std::cout << "carb::getCachedInterface failed on lidar interface query" << "\n";
                }
                else {
                  std::cout << "lidar interface succesfully retrieved from carb::getCachedInterface" << "\n";
                }
                if (mLidarInterface->isLidarSensor(mLidarPath.c_str())) {
                  std::cout << "lidar sensor succesuflly detected at: " << mLidarPath.c_str() << "\n";
                }
                else {
                  std::cout << "we should debug dump here" << "\n";
                }

                mCarbTasks.push_back(mCarbTaskingInterface->addTaskIn(std::chrono::seconds(3), carb::tasking::Priority::eDefault, {}, &OgnClRos2CppLivoxLidarNode::test, this));

              }


              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {

                OgnClRos2CppLivoxLidarNode& state = db.perInstanceState<OgnClRos2CppLivoxLidarNode>();

                std::shared_ptr<isaacsim::ros2::bridge::Ros2PointCloudMessage> tmpMsg = state.mRos2Factory->createPointCloudMessage();

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
                int pointStep = sizeof(carb::Float3);
                int width = rows * cols;
                int height = 1;
                int totalBytes = pointStep * width;
                auto now = state.mClock.now();
                auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now - state.mStartTime).count();

                tmpMsg->generateBuffer(timestamp, state.mLidarFrameId, width, height, pointStep);
                memcpy(tmpMsg->getBufferPtr(), reinterpret_cast<void*>(rawPointCloud.data()), totalBytes);
                std::cout << "_____Writer Thread__________________" << "\n";
                std::cout << "Rows: " << rows << "\n";
                std::cout << "Cols: " << cols << "\n";
                std::cout << "width: " << width << "\n";
                std::cout << "totalBytes: " << totalBytes << "\n";
                std::cout << "timestamp: " << timestamp << "\n";
                std::cout << "_______________________" << "\n";

                state.mRos2PointCloudMsg = tmpMsg;
                return true;
              }

              static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId) {
                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);

                state.running = false;
                for (auto& task : state.mCarbTasks) {
                  if (!state.mCarbTaskingInterface->tryCancelTask(*task.task_if())) {
                    task.wait();
                  }
                
                }
                state.mCarbTasks.clear();
              }

              void test() {
                while (running) {
                  mRos2PointCloudPublisher.get()->publish(mRos2PointCloudMsg->getPtr());
                }
              }

            };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

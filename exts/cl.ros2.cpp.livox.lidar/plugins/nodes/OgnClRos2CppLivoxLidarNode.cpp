#include <string>

#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <isaacsim/ros2/bridge/IRos2Bridge.h>
#include <isaacsim/ros2/bridge/Ros2Factory.h>
#include <isaacsim/ros2/bridge/Ros2Types.h>
#include <isaacsim/ros2/bridge/Ros2QoS.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>

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

            isaacsim::core::simulation_manager::ISimulationManager* mIsaacSimInterface;

            std::shared_ptr<isaacsim::ros2::bridge::Ros2Publisher> mRos2StringPublisher;

            std::shared_ptr<isaacsim::ros2::bridge::Ros2SemanticLabelMessage> mRos2StringMsg;

            const std::string mLidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";
            const std::string mQoSProfile = R"({
              "history": "keepLast",
              "depth": 10,
              "reliability": "reliable",
              "durability": "volatile",
              "deadline": 5.0,
              "lifespan": 10.0,
              "liveliness": "automatic",
              "leaseDuration": 9.0
            })";
            //needs to be reliable for rviz2, seems to need to be bestEffort for ros2 topic hz?

            const std::string mLidarFrameId = "test_frame_id";
            //bool running = true;

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

                mRos2PointCloudPublisher = mRos2Factory->createPublisher(mRos2NodeHandle.get(), "/test/pc", mRos2PointCloudMsg->getTypeSupportHandle(), qos);

                mIsaacSimInterface = carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();

              }
              ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {

                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);

              }

              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {

                OgnClRos2CppLivoxLidarNode& state = db.perInstanceState<OgnClRos2CppLivoxLidarNode>();

                carb::Float3* carbPointCloud = state.mLidarInterface->getPointCloud(state.mLidarPath.c_str());
                int rows = state.mLidarInterface->getNumRows(state.mLidarPath.c_str());
                int cols = state.mLidarInterface->getNumCols(state.mLidarPath.c_str());
                int width = rows * cols;
                int totalBytes = width * sizeof(carb::Float3);

                int pointStep = sizeof(carb::Float3);
                int height = 1;
                double timestamp = state.mIsaacSimInterface->getSimulationTimeMonotonic();
                //not sure if ros2 requires it to be monotonic
                state.mRos2PointCloudMsg->generateBuffer(timestamp, state.mLidarFrameId, width, 1, sizeof(carb::Float3));
                memcpy(state.mRos2PointCloudMsg->getBufferPtr(), reinterpret_cast<void*>(carbPointCloud), totalBytes);
                state.mRos2PointCloudPublisher.get()->publish(state.mRos2PointCloudMsg->getPtr());
                return true;
              }

              static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId) {
                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);
              }
            };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

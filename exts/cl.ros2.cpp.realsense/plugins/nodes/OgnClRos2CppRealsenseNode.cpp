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

#include <OgnClRos2CppRealsenseNodeDatabase.h>

namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace realsense {

        class OgnClRos2CppRealsenseNode {

          private:

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

            isaacsim::core::simulation_manager::ISimulationManager mIsaacSimInterface;
            const std::string mQoSProfile = R"({
                "history": "keepLast",
                "depth": 10,
                "reliability": "bestEffort",
                "durability": "volatile",
                "deadline": 5.0,
                "lifespan": 10.0,
                "liveliness": "automatic",
                "leaseDuration": 9.0
              })"

          public:
            OgnClRos2CppRealsenseNode() {

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

              mIsaacSimInterface = carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();

            }

            ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {

                OgnClRos2CppRealsenseNode& state = OgnClRos2CppRealsenseNodeDatabase::sPerInstanceState<OgnClRos2CppRealsenseNode>(node, instanceId);
              }

              static bool compute(OgnClRos2CppRealsenseNodeDatabase& db) {

                OgnClRos2CppRealsenseNode& state = db.perInstanceState<OgnClRos2CppRealsenseNode>();
                return true;
              }

              static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId) {
                OgnClRos2CppRealsenseNode& state = OgnClRos2CppRealsenseNodeDatabase::sPerInstanceState<OgnClRos2CppRealsenseNode>(node, instanceId);

              }
        };

        REGISTER_OGN_NODE()

      }
    }
  }
}


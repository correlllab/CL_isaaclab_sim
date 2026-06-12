#include <OgnClRos2CppLivoxLidarNodeDatabase.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>



namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace livox {
        namespace lidar {
          class OgnClRos2CppLivoxLidarNode  {

            std::unique_ptr<isaacsim::sensors::physx::LidarSensorInterface> mLidarInterface;

            public:

              OgnClRos2CppLivoxLidarNode() {}
              ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {
                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);

                state.mLidarInterface = std::make_unique<isaacsim::sensors::physx::LidarSensorInterface>();
              
              }
              //static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId);
              //static void release(NodeObj const& node);

              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {
                OgnClRos2CppLivoxLidarNode& state = db.perInstanceState<OgnClRos2CppLivoxLidarNode>();
                std::cout << "hi" << "\n";
                return true;
              }
              
            };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

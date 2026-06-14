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

          struct pointCloud {
            carb::Float3* pc;
            int numRows;
            int numCols;
          };

          class OgnClRos2CppLivoxLidarNode  {

            isaacsim::sensors::physx::LidarSensorInterface* mLidarInterface = nullptr;
            std::string mLidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";

            std::unique_ptr<pointCloud> latestPc;


            public:

              OgnClRos2CppLivoxLidarNode() {}
              ~OgnClRos2CppLivoxLidarNode() {}

              static void initInstance(NodeObj const& node, GraphInstanceID instanceId) {
                std::cout << "calling initInstance" << "\n";
                OgnClRos2CppLivoxLidarNode& state = OgnClRos2CppLivoxLidarNodeDatabase::sPerInstanceState<OgnClRos2CppLivoxLidarNode>(node, instanceId);
                std::cout << "just defined state" << "\n";

                std::cout << "about to define lidar interface as unique ptr" << "\n";
                state.mLidarInterface = carb::getCachedInterface<isaacsim::sensors::physx::LidarSensorInterface>();

                if (!state.mLidarInterface) {
                  std::cout << "carb::getCachedInterface failed on lidar interface query" << "\n";
                }
                else {
                  std::cout << "lidar interface succesfully retrieved from carb::getCachedInterface" << "\n";
                }
                //std::string lidarPath = "/World/envs/env_0/Robot/lidar_link/livox_lidar";

                bool ret = state.mLidarInterface->isLidarSensor(state.mLidarPath.c_str());
                if (ret) {
                  std::cout << "lidar sensor succesuflly detected at: " << state.mLidarPath.c_str() << "\n";
                }
                else {
                  std::cout << "we should debug dump here" << "\n";
                }
                
                isaacsim::ros2::bridge::Ros2Bridge* ros2Bridge = carb::getCachedInterface<isaacsim::ros2::bridge::Ros2Bridge>();

                isaacsim::ros2::bridge::Ros2Factory* factory = ros2Bridge->getFactory();

                auto* contextHandlePtr = reinterpret_cast<std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle>*>(ros2Bridge->getDefaultContextHandleAddr());

                std::shared_ptr<isaacsim::ros2::bridge::Ros2NodeHandle> nodeHandle = factory->createNodeHandle("test", "test", contextHandlePtr->get());

                std::shared_ptr<isaacsim::ros2::bridge::Ros2PointCloudMessage> msg = factory->createPointCloudMessage();

                isaacsim::ros2::bridge::Ros2QoSProfile qos;
                const std::string& qosProfile = "10";
                isaacsim::ros2::bridge::jsonToRos2QoSProfile(qos, qosProfile);

                std::shared_ptr<isaacsim::ros2::bridge::Ros2Publisher> publisher = factory->createPublisher(nodeHandle.get(), "/test/test", msg->getTypeSupportHandle(), qos);


                carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
                carb::tasking::TaskGroup tasks;
                tasking->sleepNs(10000000000);
                tasking->addTask(carb::tasking::Priority::eHigh, tasks, test); 

               
              }

              //static void releaseInstance(NodeObj const& node, GraphInstanceID instanceId);
              //static void release(NodeObj const& node);

              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {
                OgnClRos2CppLivoxLidarNode& state = db.perInstanceState<OgnClRos2CppLivoxLidarNode>();
                
                auto start = std::chrono::high_resolution_clock::now();
                carb::Float3* pc = state.mLidarInterface->getPointCloud(state.mLidarPath.c_str());
                

                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                if (!pc) {
                  std::cout << "no pc!" << "\n";
                }
                else {
                  std::cout << "pc recieved" << "\n";
                  std::cout << "pc recieved in: " << duration.count() << "\n";
                  int rows = state.mLidarInterface->getNumRows(state.mLidarPath.c_str());
                  int cols = state.mLidarInterface->getNumCols(state.mLidarPath.c_str());
                  state.latestPc = std::make_unique<pointCloud>();
                  state.latestPc->pc = pc;
                  state.latestPc->numRows = rows;
                  state.latestPc->numCols = cols;
                  std::cout << "rows: " << rows << "\n";
                  std::cout << "cols: " << cols << "\n";

                }
                return true;
              }

              
              static void test() {
                int i = 100;
                while (true) {
                  std::cout << "this is worker thread!" << "\n";
                }
              }
              


              
            };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

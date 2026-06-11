#include <OgnClRos2CppLivoxLidarNodeDatabase.h>


namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace livox {
        namespace lidar {
          class OgnClRos2CppLivoxLidarNode {
            public:
              static bool compute(OgnClRos2CppLivoxLidarNodeDatabase& db) {
                return true;
              }
          };

          REGISTER_OGN_NODE()
        }
      }
    }
  }
}

#define CARB_EXPORTS
#include <carb/PluginUtils.h>
#include <omni/ext/IExt.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>

const struct carb::PluginImplDesc pluginImplDesc = {
"cl.ros2.cpp.livox.lidar",
"test", "mateo @ correll lab", carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry, omni::fabric::IPath, omni::fabric::IToken);

  DECLARE_OGN_NODES();

namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace livox {
        namespace lidar {

          class ClRos2CppLivoxLidarExtension : public omni::ext::IExt {
            public:
              void onStartup(const char* extId) override {
                //maybe some of the usd initialization code here??
                printf("ClRos2CppLivoxLidarExtension starting up (ext_id: %s). \n", extId);
              }

              void onShutdown() override {
                printf("ClRos2CppLivoxLidarExtension shutting down\n");
                RELEASE_OGN_NODES()
              }
            private:
          };
        }
      }
    }
  }
}

CARB_PLUGIN_IMPL(pluginImplDesc, cl::ros2::cpp::livox::lidar::ClRos2CppLivoxLidarExtension)


void fillInterface(cl::ros2::cpp::livox::lidar::ClRos2CppLivoxLidarExtension& iface) {}


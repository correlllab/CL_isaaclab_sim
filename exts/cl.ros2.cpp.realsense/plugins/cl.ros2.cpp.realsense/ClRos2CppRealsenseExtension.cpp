#define CARB_EXPORTS
#include <carb/PluginUtils.h>
#include <omni/ext/IExt.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>
//#include <carb>
#include <omni/kit/commands/ICommandBridge.h>
#include <carb/dictionary/IDictionary.h>

#include <pxr/usd/usdUtils/stageCache.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <chrono>

const struct carb::PluginImplDesc pluginImplDesc = {
"cl.ros2.cpp.realsense",
"test", "mateo @ correll lab", carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry, omni::fabric::IPath, omni::fabric::IToken);

DECLARE_OGN_NODES();

namespace cl {
  namespace ros2 {
    namespace cpp {
      namespace realsense {
        class ClRos2CppRealsenseExtension : public omni::ext::IExt {
          public:
            void onStartup(const char* extId) override {
              INITIALIZE_OGN_NODES();
            }
            void onShutdown() override {
              RELEASE_OGN_NODES();
            }
        };
      }
    }
  }
}

CARB_PLUGIN_IMPL(pluginImplDesc, cl::ros2::cpp::realsense::ClRos2CppRealsenseExtension)
void fillInterface(cl::ros2::cpp::realsense::ClRos2CppRealsenseExtension& iface) {}

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

                const std::vector<PXR_NS::UsdStageRefPtr> allStages = PXR_NS::UsdUtilsStageCache::Get().GetAllStages();

                PXR_NS::UsdStageRefPtr activeStage = allStages[0];

                static const PXR_NS::SdfPath livoxLidarPath("/World/envs/env_0/Robot/lidar_link/livox_lidar");

                if (!activeStage->GetPrimAtPath(livoxLidarPath)) {
                  auto bigStart = std::chrono::high_resolution_clock::now();

                  auto commandBridge = carb::getCachedInterface<omni::kit::commands::ICommandBridge>();

                  auto iDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
                  carb::dictionary::Item* kwargs =  iDictionary->createItem(nullptr, "", carb::dictionary::ItemType::eDictionary);
                  iDictionary->makeStringAtPath(kwargs, "path", "/livox_lidar");

                  iDictionary->makeStringAtPath(kwargs, "parent", "/World/envs/env_0/Robot/lidar_link");

                  iDictionary->makeFloatAtPath(kwargs, "min_range", 0.4);

                  iDictionary->makeFloatAtPath(kwargs, "max_range", 100.0);

                  iDictionary->makeBoolAtPath(kwargs, "draw_points", false);

                  iDictionary->makeBoolAtPath(kwargs, "draw_lines", true);

                  iDictionary->makeFloatAtPath(kwargs, "horizontal_fov", 360.0);

                  iDictionary->makeFloatAtPath(kwargs, "vertical_fov", 60.0);

                  iDictionary->makeFloatAtPath(kwargs, "horizontal_resolution", 0.4);

                  iDictionary->makeFloatAtPath(kwargs, "vertical_resolution", 0.4);

                  iDictionary->makeFloatAtPath(kwargs, "rotation_rate", 0.0);

                  iDictionary->makeBoolAtPath(kwargs, "high_lod", true);

                  iDictionary->makeFloatAtPath(kwargs, "yaw_offset", 0.0);

                  iDictionary->makeBoolAtPath(kwargs, "enable_semantics", false);

                  auto smallBegin = std::chrono::high_resolution_clock::now();

                  //auto f = std::async(
                  //  static_cast<bool (omni::kit::commands::ICommandBridge::*)(const char*, const carb::dictionary::Item*) const>(&omni::kit::commands::ICommandBridge::executeCommand), commandBridge, "RangeSensorCreateLidar", kwargs);
//
                  //calling command execution async blocks, i think it must be done on main extension thread
                  commandBridge->executeCommand("RangeSensorCreateLidar", kwargs);
                  
                  auto smallEnd = std::chrono::high_resolution_clock::now();
                  auto bigEnd = std::chrono::high_resolution_clock::now();

                  auto bigDuration = std::chrono::duration_cast<std::chrono::microseconds>(bigEnd - bigStart);
                  auto smallDuration = std::chrono::duration_cast<std::chrono::microseconds>(smallEnd - smallBegin);


                  std::cout << "whole command preparation + execution takes: " << bigDuration.count() << " microseconds" << "\n"; 

                  std::cout << "command execution takes: " << smallDuration.count() << " microseconds" << "\n"; 
                  std::cout << "called RangeSensorCreateLidar" << "\n";
                }

                INITIALIZE_OGN_NODES()
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


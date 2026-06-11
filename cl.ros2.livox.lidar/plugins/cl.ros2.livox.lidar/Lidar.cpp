#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>

#include <omni/graph/core/ogn/Registration.h>

#include <cl/ros2/livox/lidar/Lidar.h>

const struct carb::PluginImplDesc pluginImplDesc = { "cl.ros2.livox.lidar.plugin",
                                                     "Helpful text describing the plugin", "Author",
                                                     carb::PluginHotReload::eEnabled, "dev" };

DECLARE_OGN_NODES();

namespace cl
{
namespace ros2
{
namespace livox
{
namespace lidar
{

void setDefaultStatus(const char* status)
{
    CARB_LOG_INFO("setDefaultStatus %s", status);
}

class LidarImpl : public ILidar
{
public:
    bool registerObject(uint32_t id) override
    {
        CARB_LOG_INFO("registerObject %d", id);
        mId = id;
        return mId ? true : false;
    }

private:
    uint32_t mId = 0;
};

/**
 * The Extension class
 */
class Extension : public omni::ext::IExt
{
public:
    /**
     * Method called when the extension is loaded/enabled
     */
    void onStartup(const char* extId) override
    {
        CARB_LOG_INFO("onStartup %s", extId);
        INITIALIZE_OGN_NODES();
    }

    /**
     * Method called when the extension is disabled
     */
    void onShutdown() override
    {
        CARB_LOG_INFO("onShutdown");
        RELEASE_OGN_NODES();
    }
};

} // namespace cl
} // namespace ros2
} // namespace livox
} // namespace lidar

/**
 * Optional function (called the first time an interface is acquired from the plugin library)
 */
CARB_EXPORT void carbOnPluginStartup()
{
    CARB_LOG_INFO("carbOnPluginStartup");
}

/**
 * Optional function (called right before the OS release the plugin library)
 */
CARB_EXPORT void carbOnPluginShutdown()
{
    CARB_LOG_INFO("carbOnPluginShutdown");
}

CARB_PLUGIN_IMPL(pluginImplDesc, cl::ros2::livox::lidar::LidarImpl, cl::ros2::livox::lidar::Extension)

void fillInterface(cl::ros2::livox::lidar::LidarImpl& iface)
{
}

void fillInterface(cl::ros2::livox::lidar::Extension& iface)
{
}

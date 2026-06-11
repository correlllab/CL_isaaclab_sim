#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>

#include <omni/graph/core/ogn/Registration.h>

#include <cl/ros2/livox/imu/Imu.h>

const struct carb::PluginImplDesc pluginImplDesc = { "cl.ros2.livox.imu.plugin",
                                                     "Helpful text describing the plugin", "Author",
                                                     carb::PluginHotReload::eEnabled, "dev" };

DECLARE_OGN_NODES();

namespace cl
{
namespace ros2
{
namespace livox
{
namespace imu
{

void setDefaultStatus(const char* status)
{
    CARB_LOG_INFO("setDefaultStatus %s", status);
}

class ImuImpl : public IImu
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
} // namespace imu

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

CARB_PLUGIN_IMPL(pluginImplDesc, cl::ros2::livox::imu::ImuImpl, cl::ros2::livox::imu::Extension)

void fillInterface(cl::ros2::livox::imu::ImuImpl& iface)
{
}

void fillInterface(cl::ros2::livox::imu::Extension& iface)
{
}

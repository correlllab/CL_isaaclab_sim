/*
Carbonite SDK API:
  https://docs.omniverse.nvidia.com/kit/docs/carbonite/latest/api/carbonite_api.html
*/

#pragma once

#define CARB_EXPORTS

#include <carb/Defines.h>
#include <carb/Interface.h>

#include <cstdint>

namespace cl
{
namespace ros2
{
namespace livox
{
namespace imu
{

// ------------------
// custom API declaration. E.g.:
CARB_EXPORT void setDefaultStatus(const char* status);
// ------------------

/**
 * Carbonite interface
 */
struct IImu
{
    CARB_PLUGIN_INTERFACE("cl::ros2::livox::imu::IImu", 1, 0);

    // ------------------
    // custom API declaration. E.g.:
    virtual bool registerObject(uint32_t id) = 0;
    // ------------------
};

} // namespace cl
} // namespace ros2
} // namespace livox
} // namespace imu

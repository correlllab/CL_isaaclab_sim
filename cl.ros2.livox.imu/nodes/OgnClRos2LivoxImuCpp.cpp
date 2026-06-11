/*
OmniGraph core C++ API:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.core/latest/Overview.html

OmniGraph attribute data types:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/attribute_types.html

Collection of OmniGraph code examples in C++:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/ogn_code_samples_cpp.html

Collection of OmniGraph tutorials:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.tutorials/latest/Overview.html
*/

#include <cstdint>
#include <string>

#include <OgnClRos2LivoxImuCppDatabase.h>

namespace cl
{
namespace ros2
{
namespace livox
{
namespace imu
{

class OgnClRos2LivoxImuCpp
{
public:
    /**
     * Method called by the OmniGraph framework whenever an instance is added to the graph
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnClRos2LivoxImuCppDatabase::sPerInstanceState<OgnClRos2LivoxImuCpp>(nodeObj, instanceId);
    }

    /**
     * Compute the output based on inputs and internal state
     */
    static bool compute(OgnClRos2LivoxImuCppDatabase& db)
    {
        // node internal state
        auto& state = db.perInstanceState<OgnClRos2LivoxImuCpp>();
        // direct access to the ABI objects
        const auto& nodeObj = db.abi_node();
        const auto& contextObj = db.abi_context();

        try
        {
            // ------------------
            // read input values
            const int32_t input1 = db.inputs.inputAttribute1();
            const std::string input2 = db.tokenToString(db.inputs.inputAttribute2());
            // do custom computation
            state.status = true;
            // ...
            // write output values
            db.outputs.outputAttribute1() = 0.0;
            // ------------------
        }
        catch (const std::exception& e)
        {
            db.logError("Computation error: %s", e.what());
            return false;
        }
        return true;
    }

    /**
     * Method called by the OmniGraph framework whenever an instance is removed from the graph
     */
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnClRos2LivoxImuCppDatabase::sPerInstanceState<OgnClRos2LivoxImuCpp>(nodeObj, instanceId);
    }

private:
    bool status = false;
};

REGISTER_OGN_NODE()

} // namespace cl
} // namespace ros2
} // namespace livox
} // namespace imu

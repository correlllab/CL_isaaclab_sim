// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#include <ROS2ExampleNodeDatabase.h>
#include <string>

// In this example, we will publish a string message with an OmniGraph Node
#include "std_msgs/msg/string.h"
#include "rosidl_runtime_c/string_functions.h"

// ROS includes for creating nodes, publishers etc.
#include "rcl/rcl.h"

// Helpers to explicit shorten names you know you will use
using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace omni {
namespace example {
namespace cpp {
namespace omnigraph_node_ros {

class ROS2ExampleNode
{
public:
    static bool compute(ROS2ExampleNodeDatabase& db)
    {
        auto& state = db.internalState<ROS2ExampleNode>();

        if(!state.pub_created)
        {
            state.context = rcl_get_zero_initialized_context();
            state.init_options = rcl_get_zero_initialized_init_options();
            state.allocator = rcl_get_default_allocator();
            rcl_ret_t rc;

            // create init_options
            rc = rcl_init_options_init(&state.init_options, state.allocator);
            if (rc != RCL_RET_OK)
            {
                printf("Error rcl_init_options_init.\n");
                return false;
            }

            // create context
            rc = rcl_init(0, nullptr, &state.init_options, &state.context);
            if (rc != RCL_RET_OK)
            {
                printf("Error in rcl_init.\n");
                return false;
            }

            // create rcl_node
            state.my_node = rcl_get_zero_initialized_node();
            state.node_ops = rcl_node_get_default_options();
            rc = rcl_node_init(&state.my_node, "node_0", "custom_node", &state.context, &state.node_ops);
            if (rc != RCL_RET_OK)
            {
                printf("Error in rcl_node_init\n");
                return false;
            }

            const char * topic_name = "my_string";
            const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);


            state.pub_options = rcl_publisher_get_default_options();

            // Initialize Publisher
            rc = rcl_publisher_init(
                &state.my_pub,
                &state.my_node,
                my_type_support,
                topic_name,
                &state.pub_options);
            if (RCL_RET_OK != rc)
            {
                printf("Error in rcl_publisher_init %s.\n", topic_name);
                return false;
            }
            // Node, publisher was successfully created
            state.pub_created = true;

            return true;
        }

        std_msgs__msg__String* ros_msg = std_msgs__msg__String__create();

        // Get the string to be published
        std::string msg = db.inputs.publishString();

        // Assign our ros message the data from this string
        rosidl_runtime_c__String__assign(&ros_msg->data, msg.c_str());

        rcl_ret_t rc;
        rc = rcl_publish(&state.my_pub, ros_msg, NULL);
        if (rc != RCL_RET_OK)
        {
            // RCL_RET_PUBLISHER_INVALID is returned initially and then the message gets published
            return false;
        }

        // Destroy the ROS message published to release the memory it used
        std_msgs__msg__String__destroy(ros_msg);

        // Returning true tells Omnigraph that the compute was successful and the output value is now valid.
        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = ROS2ExampleNodeDatabase::sPerInstanceState<ROS2ExampleNode>(nodeObj, instanceId);


        // Remove Publisher
        rcl_ret_t rc = rcl_publisher_fini(&state.my_pub, &state.my_node);
        if (rc != RCL_RET_OK) {
            printf("Failed to finalize publisher: %d\n", rc);
        }

        // Remove Node
        rc = rcl_node_fini(&state.my_node);
        if (rc != RCL_RET_OK) {
            printf("Failed to finalize node: %d\n", rc);
        }

        state.pub_created = false;
    }

private:
    rcl_publisher_t my_pub;
    rcl_node_t my_node;
    rcl_context_t context;
    rcl_node_options_t node_ops;
    rcl_init_options_t init_options;
    rcl_allocator_t allocator;
    rcl_publisher_options_t pub_options;
    bool pub_created {false};
};

// This macro provides the information necessary to OmniGraph that lets it automatically register and deregister
// your node type definition.
REGISTER_OGN_NODE()

} // omnigraph_node_ros
} // cpp
} // omni
} // example

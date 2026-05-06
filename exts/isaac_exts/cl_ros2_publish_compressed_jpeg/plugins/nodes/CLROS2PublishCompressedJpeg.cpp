
#include <iostream>
#include <CLROS2PublishCompressedJpegDatabase.h>
#include <string>
#include "std_msgs/msg/string.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rcl/rcl.h"

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace cl_ros2_publish_compressed_jpeg {
  class CLROS2PublishCompressedJpeg {
    public:
      static bool compute(CLROS2PublishCompressedJpegDatabase& db) {
        auto& state = db.internalState<CLROS2PublishCompressedJpeg>();
        if (!state.pub_created) {
          state.context = rcl_get_zero_initialized_context();
          state.init_options = rcl_get_zero_initialized_init_options();
          state.allocator = rcl_get_default_allocator();
          rcl_ret_t rc;

          rc = rcl_init_options_init(&state.init_options, state.allocator);
          if (rc != RCL_RET_OK) {
            printf("ERROR rcl init options init \n");
            return false;
          }

          rc = rcl_init(0, nullptr, &state.init_options, &state.context);

          if (rc != RCL_RET_OK) {
            printf("Error in rcl_init. \n");
            return false;
          }

          state.my_node = rcl_get_zero_initialized_node();
          state.node_ops = rcl_node_get_default_options();
          rc = rcl_node_init(&state.my_node, "node_0", "custom_node", &state.context, &state.node_ops);
          if (rc != RCL_RET_OK) {
            printf("Error in rcl_node_init \n");
            return false;
          }

          const char * topic_name = "testRCLTopic";
          const rosidl_message_type_support_t* my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

          state.pub_options = rcl_publisher_get_default_options();

          rc = rcl_publisher_init(
            &state.my_pub,
            &state.my_node,
            my_type_support,
            topic_name,
            &state.pub_options
          );

          if (RCL_RET_OK != rc) {
            printf("error in rcl_publisher_init \n");
            return false;
          }

          state.pub_created = true;
          return true;
        }

        std_msgs__msg__String* ros_msg = std_msgs__msg__String__create();
        std::string msg = db.inputs.publishString();
        rosidl_runtime_c__String__assign(&ros_msg->data, msg.c_str());

        rcl_ret_t rc;

        rc = rcl_publish(&state.my_pub, ros_msg, NULL);
        if (rc != RCL_RET_OK) {
          return false;
        }

        std_msgs__msg__String__destroy(ros_msg);

        std::cout << "this means the node worked!!!!!" << "\n";
        return true;

      }

      static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId) {
        auto& state = CLROS2PublishCompressedJpegDatabase::sPerInstanceState<CLROS2PublishCompressedJpeg>(nodeObj, instanceId);

        rcl_ret_t rc = rcl_publisher_fini(&state.my_pub, &state.my_node);
        if(rc != RCL_RET_OK) {
          printf("Failed to finalize publisher \n");
        }

        rc = rcl_node_fini(&state.my_node);
        if (rc != RCL_RET_OK) {
          printf("failed to finailize node \n");
        
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
  REGISTER_OGN_NODE()
}

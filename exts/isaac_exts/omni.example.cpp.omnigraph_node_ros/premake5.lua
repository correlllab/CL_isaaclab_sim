-- Setup the basic extension information.
local ext = get_current_extension_info()
project_ext(ext)


-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, "omni/example/cpp/omnigraph_node_ros")


-- --------------------------------------------------------------------------------------------------------------
-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
}


-- --------------------------------------------------------------------------------------------------------------
-- Copy the __init__.py to allow building of a non-linked ogn/ import directory.
-- In a mixed extension this would be part of a separate Python-based project but since here it is just the one
-- file it can be copied directly with no build dependencies.
repo_build.prebuild_copy {
    { "omni/example/cpp/omnigraph_node_ros/__init__.py", ogn.python_target_path }
}


-- --------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn( ext, ogn )


-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)
    -- It is important that you add all subdirectories containing C++ code to this project
    add_files("source", "plugins/"..ogn.module)
    add_files("nodes", "plugins/nodes")

    -- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
    add_ogn_dependencies(ogn)

    includedirs {
        -- System level ROS includes
        "%{target_deps}/system_ros/include/std_msgs",

        "%{target_deps}/system_ros/include/geometry_msgs",

        "%{target_deps}/system_ros/include/rosidl_runtime_c",

        "%{target_deps}/system_ros/include/rosidl_typesupport_interface",

        "%{target_deps}/system_ros/include/rcl",

        "%{target_deps}/system_ros/include/rcutils",

        "%{target_deps}/system_ros/include/rmw",

        "%{target_deps}/system_ros/include/rcl_yaml_param_parser",

        -- Additional sourced ROS workspace includes
        "%{target_deps}/additional_ros/include/tutorial_interfaces",
    }

    libdirs {
        -- System level ROS libraries
        "%{target_deps}/system_ros/lib",

        -- Additional sourced ROS workspace libraries
        "%{target_deps}/additional_ros/lib",
    }

    links{
        --  Minimal ROS 2 C API libs needed for your nodes to work
        "rosidl_runtime_c", "rcutils", "rcl", "rmw",

        -- For the simple string message, add the deps
        "std_msgs__rosidl_typesupport_c", "std_msgs__rosidl_generator_c",

        -- Add dependencies of the custom message with its libs
        "geometry_msgs__rosidl_typesupport_c", "geometry_msgs__rosidl_typesupport_c",
        "tutorial_interfaces__rosidl_typesupport_c", "tutorial_interfaces__rosidl_generator_c",
    }

    filter { "system:linux" }
        linkoptions { "-Wl,--export-dynamic" }

    cppdialect "C++17"

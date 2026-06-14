-- Setup the basic extension information.
local ext = get_current_extension_info()
project_ext(ext)


-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, "cl/ros2/cpp/livox/lidar")


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
    { "cl/ros2/cpp/livox/lidar/__init__.py", ogn.python_target_path }
}


-- --------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn( ext, ogn )


-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)

    add_files("source", "plugins/"..ogn.module)
    add_files("nodes", "plugins/nodes")

    includedirs {
        "/opt/conda/envs/unitree_sim_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.physx/include",
        "/opt/conda/envs/unitree_sim_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/include",
        "/opt/conda/envs/unitree_sim_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.includes/include",
        "/opt/conda/include"
    }

    add_ogn_dependencies(ogn)

    cppdialect "C++17"

--project_ext_plugin(ext, ogn.plugin_project)
--    -- It is important that you add all subdirectories containing C++ code to this project
--    add_files("source", "plugins/"..ogn.module)
--    add_files("nodes", "plugins/nodes")
--    includedirs {
--      "/opt/conda/envs/unitree_sim_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.physx/include",
--      "/opt/ros/humble/include"
--
--    }
--    libdirs {
--      "/opt/ros/humble/lib"
--    }
--
--    links {
--      "rosidl_runtime_c", "rcutils", "rcl", "rmw", "std_msgs__rosidl_typesupport_c", "std_msgs__rosidl_generator_c"
--    }
--
--    -- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
--    add_ogn_dependencies(ogn)
--
--    cppdialect "C++17"

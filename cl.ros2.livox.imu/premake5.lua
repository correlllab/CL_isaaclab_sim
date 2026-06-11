-- Setup the basic extension variables
local ext = get_current_extension_info()
local ogn = get_ogn_project_information(ext, "cl/ros2/livox/imu")
-- Set up the basic shared project information
project_ext (ext)

-- -------------------------------------
-- Breaking this out as a separate project ensures the .ogn files
-- are processed before their results are needed
project_ext_ogn(ext, ogn)

-- -------------------------------------
-- Build the C++ plugin that will be loaded by the extension
project_ext_plugin(ext, "cl.ros2.livox.imu.plugin")
    cppdialect "C++17"

    add_files("include", "include/cl/ros2/livox/imu")
    add_files("source", "plugins/cl.ros2.livox.imu")
    add_files("nodes", ogn.nodes_path)
    includedirs {
        "include",
        "plugins/cl.ros2.livox.imu",
        "%{target_deps}/nv_usd/release/include",
    }
    -- OGN standard dependencies: includes, libraries, compiler flags
    add_ogn_dependencies(ogn)

    filter { "configurations:debug" }
        defines { "_DEBUG" }
    filter { "configurations:release" }
        defines { "NDEBUG" }
    filter {}

-- -------------------------------------
-- Link/copy folders and files to be packaged with the extension
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "include", ext.target_dir.."/include" },
}

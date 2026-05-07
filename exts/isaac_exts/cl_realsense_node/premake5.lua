-- Setup the basic extension variables
local ext = get_current_extension_info()
local ogn = get_ogn_project_information(ext, "cl_realsense_node")
-- Set up the basic shared project information
project_ext (ext)

-- -------------------------------------
-- Breaking this out as a separate project ensures the .ogn files
-- are processed before their results are needed
project_ext_ogn(ext, ogn)

add_ogn_dependencies(ogn, {"python/nodes"})

-- -------------------------------------
-- Link/copy folders and files to be packaged with the extension
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "python/impl", ext.target_dir.."/cl_realsense_node/impl" },
    { "python/tests", ext.target_dir.."/cl_realsense_node/tests" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir.."/cl_realsense_node" },
}

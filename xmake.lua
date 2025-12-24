set_project("concord")
set_version("2.5.0")
set_xmakever("2.7.0")

-- Set C++ standard
set_languages("c++20")

-- Add build options
add_rules("mode.debug", "mode.release")

-- Compiler warnings and flags
add_cxxflags("-Wall", "-Wextra", "-Wpedantic")
add_cxxflags("-Wno-reorder", "-Wno-narrowing", "-Wno-array-bounds")
add_cxxflags("-Wno-unused-variable", "-Wno-unused-parameter", "-Wno-stringop-overflow", "-Wno-unused-but-set-variable")

-- Add global search paths for packages in ~/.local
local home = os.getenv("HOME")
if home then
    add_includedirs(path.join(home, ".local/include"))
    add_linkdirs(path.join(home, ".local/lib"))
end

-- Add devbox/nix paths for system packages
local cmake_prefix = os.getenv("CMAKE_PREFIX_PATH")
if cmake_prefix then
    add_includedirs(path.join(cmake_prefix, "include"))
    add_linkdirs(path.join(cmake_prefix, "lib"))
end

local pkg_config = os.getenv("PKG_CONFIG_PATH")
if pkg_config then
    -- Split PKG_CONFIG_PATH by ':' and process each path
    for _, pkgconfig_path in ipairs(pkg_config:split(':')) do
        if os.isdir(pkgconfig_path) then
            -- PKG_CONFIG_PATH typically points to .../lib/pkgconfig
            -- We want to get the prefix (two levels up) to find include and lib
            local lib_dir = path.directory(pkgconfig_path)  -- .../lib
            local prefix_dir = path.directory(lib_dir)      -- .../
            local include_dir = path.join(prefix_dir, "include")

            if os.isdir(lib_dir) then
                add_linkdirs(lib_dir)
            end
            if os.isdir(include_dir) then
                add_includedirs(include_dir)
            end
        end
    end
end

-- Options
option("concord_examples")
    set_default(false)
    set_showmenu(true)
    set_description("Build examples")
option_end()

option("concord_tests")
    set_default(false)
    set_showmenu(true)
    set_description("Enable tests")
option_end()

-- Note: rerun_sdk integration removed from xmake to keep builds sandbox-friendly.

-- Tests use doctest headers; prefer a local checkout (no network).
local doctest_dir = nil
if has_config("concord_tests") then
    local candidates = os.dirs(path.join(os.projectdir(), "..", "*", "build", "_deps", "doctest-src"))
    for _, cand in ipairs(candidates) do
        if os.isfile(path.join(cand, "doctest", "doctest.h")) then
            doctest_dir = cand
            break
        end
    end
end

-- Mandatory datapod dependency (like ../optinum). Prefer local checkout at ../datapod, otherwise fetch from git.
local dp_dir = path.join(os.projectdir(), "..", "datapod")
local dp_source_dir = os.isdir(dp_dir) and dp_dir or path.join(os.projectdir(), "build/_deps/datapod-src")

package("datapod")
    set_sourcedir(dp_source_dir)

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if sourcedir == dp_dir then
            return
        end
        if not os.isdir(sourcedir) then
            print("Fetching datapod from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "0.0.6",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/datapod.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs, {cmake_generator = "Unix Makefiles"})
    end)
package_end()

add_requires("datapod")

-- Main library target
target("concord")
    set_kind("headeronly")

    -- Add header files
    add_headerfiles("include/(concord/**.hpp)")
    add_includedirs("include", {public = true})

    add_packages("datapod", {public = true})
    add_defines("SHORT_NAMESPACE", {public = true})

    -- Headers-only install
    add_installfiles("include/(concord/**.hpp)")
target_end()

-- Examples (only build when concord is the main project)
if has_config("concord_examples") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("examples/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("concord")

            add_includedirs("include")
        target_end()
    end
end

-- Tests (only build when concord is the main project)
if has_config("concord_tests") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("test/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("concord")
            add_includedirs("include")
            if doctest_dir then
                add_includedirs(doctest_dir)
            else
                raise("doctest not found (expected ../*/build/_deps/doctest-src); build any sibling that vendors doctest or set it up locally")
            end
            add_defines("DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN")

            -- Add as test
            add_tests("default", {rundir = os.projectdir()})
        target_end()
    end
end

-- Task to generate CMakeLists.txt
task("cmake")
    on_run(function ()
        import("core.project.config")

        -- Load configuration
        config.load()

        -- Generate CMakeLists.txt
        os.exec("xmake project -k cmakelists")

        print("CMakeLists.txt generated successfully!")
    end)

    set_menu {
        usage = "xmake cmake",
        description = "Generate CMakeLists.txt from xmake.lua",
        options = {}
    }
task_end()

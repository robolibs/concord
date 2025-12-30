-- Project configuration
-- NOTE: Due to xmake description domain limitations, PROJECT_NAME must be hardcoded
--       and kept in sync with the NAME file. The VERSION is read dynamically.
local PROJECT_NAME = "concord"
local PROJECT_VERSION = "$(shell cat VERSION)"

-- Dependencies formats:
--   Git:    {"name", "https://github.com/org/repo.git", "tag"}
--   Local:  {"name", "../path/to/local"}  (optional: uses git if not found)
--   System: "pkgconfig::libname" or {system = "boost"}
local LIB_DEPS = {
    {"datapod", "https://github.com/robolibs/datapod.git", "0.0.15"},
    {"optinum", "https://github.com/robolibs/optinum.git", "0.0.12"},
    {"graphix", "https://github.com/robolibs/graphix.git", "0.0.2"},
}
local EXAMPLE_DEPS = {
    {system = "rerun_sdk"},
}
local TEST_DEPS = {
    {"doctest", "https://github.com/doctest/doctest.git", "v2.4.11"},
}

set_project(PROJECT_NAME)
set_version(PROJECT_VERSION)
set_xmakever("2.7.0")

set_languages("c++20")
add_rules("mode.debug", "mode.release")

-- Compiler flags
add_cxxflags("-Wall", "-Wextra", "-Wpedantic",
             "-Wno-reorder", "-Wno-narrowing", "-Wno-array-bounds",
             "-Wno-unused-variable", "-Wno-unused-parameter",
             "-Wno-stringop-overflow", "-Wno-unused-but-set-variable",
             "-Wno-gnu-line-marker", "-Wno-comment")

-- Architecture-specific SIMD flags
if is_arch("x86_64", "x64", "i386", "x86") then
    add_cxxflags("-mavx", "-mavx2", "-mfma")
elseif is_arch("arm64", "arm64-v8a", "aarch64") then
    -- ARM64: NEON is enabled by default
elseif is_arch("arm", "armv7", "armv7-a") then
    add_cxxflags("-mfpu=neon", "-mfloat-abi=hard")
end

-- Environment paths (HOME, CMAKE_PREFIX_PATH, PKG_CONFIG_PATH)
local function add_env_paths()
    local home = os.getenv("HOME")
    if home then
        add_includedirs(path.join(home, ".local/include"))
        add_linkdirs(path.join(home, ".local/lib"))
    end

    local cmake_prefix = os.getenv("CMAKE_PREFIX_PATH")
    if cmake_prefix then
        add_includedirs(path.join(cmake_prefix, "include"))
        add_linkdirs(path.join(cmake_prefix, "lib"))
    end

    local pkg_config = os.getenv("PKG_CONFIG_PATH")
    if pkg_config then
        for _, p in ipairs(pkg_config:split(':')) do
            if os.isdir(p) then
                local lib_dir = path.directory(p)
                local prefix = path.directory(lib_dir)
                if prefix ~= "/usr" and prefix ~= "/usr/local" then
                    if os.isdir(lib_dir) then add_linkdirs(lib_dir) end
                    if os.isdir(path.join(prefix, "include")) then
                        add_includedirs(path.join(prefix, "include"))
                    end
                end
            end
        end
    end
end
add_env_paths()

-- Options
option("examples", {default = false, showmenu = true, description = "Build examples"})
option("tests",    {default = false, showmenu = true, description = "Enable tests"})
option("short_namespace", {default = false, showmenu = true, description = "Enable short namespace alias"})
option("expose_all", {default = false, showmenu = true, description = "Expose all submodule functions in optinum:: namespace"})

-- Helper: process dependency
local function process_dep(dep)
    -- String: xmake/system package (e.g., "pkgconfig::libzmq" or "boost")
    if type(dep) == "string" then
        add_requires(dep)
        return dep
    end

    -- Table with "system" key: system package with alias
    if dep.system then
        add_requires(dep.system, {system = true})
        return dep.system
    end

    -- Table: {name, source, tag}
    local name, source, tag = dep[1], dep[2], dep[3]

    -- Local path (no tag, source is a path)
    if not tag and source and not source:match("^https?://") then
        local local_dir = path.join(os.projectdir(), source)
        if os.isdir(local_dir) then
            package(name)
                set_sourcedir(local_dir)
                on_install(function (pkg)
                    local configs = {"-DCMAKE_BUILD_TYPE=" .. (pkg:is_debug() and "Debug" or "Release")}
                    import("package.tools.cmake").install(pkg, configs, {cmake_generator = "Unix Makefiles"})
                end)
            package_end()
            add_requires(name)
            return name
        end
    end

    -- Git dependency
    local sourcedir = path.join(os.projectdir(), "build/_deps/" .. name .. "-src")
    package(name)
        set_sourcedir(sourcedir)
        on_fetch(function (pkg)
            if not os.isdir(pkg:sourcedir()) then
                print("Fetching " .. name .. " from git...")
                os.mkdir(path.directory(pkg:sourcedir()))
                os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", tag,
                                "-c", "advice.detachedHead=false", source, pkg:sourcedir()})
            end
        end)
        on_install(function (pkg)
            local configs = {"-DCMAKE_BUILD_TYPE=" .. (pkg:is_debug() and "Debug" or "Release")}
            import("package.tools.cmake").install(pkg, configs, {cmake_generator = "Unix Makefiles"})
        end)
    package_end()
    add_requires(name)
    return name
end

-- Process all dependencies and collect names
local LIB_DEP_NAMES = {}
for _, dep in ipairs(LIB_DEPS) do
    table.insert(LIB_DEP_NAMES, process_dep(dep))
end

local EXAMPLE_DEP_NAMES = {unpack(LIB_DEP_NAMES)}
for _, dep in ipairs(EXAMPLE_DEPS) do
    table.insert(EXAMPLE_DEP_NAMES, process_dep(dep))
end

local TEST_DEP_NAMES = {unpack(EXAMPLE_DEP_NAMES)}
if has_config("tests") then
    for _, dep in ipairs(TEST_DEPS) do
        table.insert(TEST_DEP_NAMES, process_dep(dep))
    end
end

-- Main library
target(PROJECT_NAME)
    set_kind("static")
    add_files("src/" .. PROJECT_NAME .. "/**.cpp")
    add_headerfiles("include/(" .. PROJECT_NAME .. "/**.hpp)")
    add_includedirs("include", {public = true})
    add_installfiles("include/(" .. PROJECT_NAME .. "/**.hpp)")

    for _, dep in ipairs(LIB_DEP_NAMES) do add_packages(dep) end

    if has_config("short_namespace") then
        add_defines("SHORT_NAMESPACE", {public = true})
    end
    if has_config("expose_all") then
        add_defines("OPTINUM_EXPOSE_ALL", {public = true})
    end

    on_install(function (target)
        os.cp(target:targetfile(), path.join(target:installdir(), "lib", path.filename(target:targetfile())))
    end)
target_end()

-- Helper: create binary targets from glob pattern
local function add_binaries(pattern, opts)
    opts = opts or {}
    for _, filepath in ipairs(os.files(pattern)) do
        target(path.basename(filepath))
            set_kind("binary")
            add_files(filepath)
            add_deps(PROJECT_NAME)
            add_includedirs("include")
            add_defines("SHORT_NAMESPACE")

            if opts.packages then
                for _, pkg in ipairs(opts.packages) do add_packages(pkg) end
            end
            if opts.defines then
                for _, def in ipairs(opts.defines) do add_defines(def) end
            end
            if opts.syslinks then
                for _, link in ipairs(opts.syslinks) do add_syslinks(link) end
            end
            if opts.is_test then
                add_tests("default", {rundir = os.projectdir()})
            end
        target_end()
    end
end

-- Examples & Tests (only when this is the main project)
if os.projectdir() == os.curdir() then
    if has_config("examples") then
        add_binaries("examples/*.cpp", {
            packages = EXAMPLE_DEP_NAMES
        })
    end

    if has_config("tests") then
        add_binaries("test/**.cpp", {
            packages = TEST_DEP_NAMES,
            defines = {"DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN"},
            syslinks = {"pthread"},
            is_test = true
        })
    end
end

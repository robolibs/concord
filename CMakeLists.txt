cmake_minimum_required(VERSION 3.15)

# ==================================================================================================
# Project configuration
# ==================================================================================================
set(PROJECT_NAME concord)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" PROJECT_VERSION)
string(STRIP "${PROJECT_VERSION}" PROJECT_VERSION)

# Dependencies formats:
#   Git:    set(LIB_DEPS "name|https://github.com/org/repo.git|tag" ...)
#   Find:   set(LIB_DEPS "name" ...)  # Uses find_package
#   Pkg:    set(LIB_DEPS "pkg::name" ...)  # Uses pkg-config
set(LIB_DEPS
    "datapod|https://github.com/robolibs/datapod.git|0.0.15"
    "optinum|https://github.com/robolibs/optinum.git|0.0.12"
    "graphix|https://github.com/robolibs/graphix.git|0.0.2"
)
set(EXAMPLE_DEPS
    "pkg::rerun_sdk"
)
set(TEST_DEPS
    "doctest|https://github.com/doctest/doctest.git|v2.4.12"
)

# ==================================================================================================
# Project setup
# ==================================================================================================
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)

add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-reorder -Wno-narrowing -Wno-array-bounds
    -Wno-unused-variable -Wno-unused-parameter
    -Wno-stringop-overflow -Wno-unused-but-set-variable
    -Wno-gnu-line-marker -Wno-comment
)

# Architecture-specific SIMD flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[3-6]86")
    add_compile_options(-mavx -mavx2 -mfma)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # ARM64: NEON is enabled by default
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    add_compile_options(-mfpu=neon -mfloat-abi=hard)
endif()

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
option(${PROJECT_NAME_UPPER}_BUILD_EXAMPLES "Build examples" OFF)
option(${PROJECT_NAME_UPPER}_ENABLE_TESTS "Enable tests" OFF)
option(SHORT_NAMESPACE "Enable short namespace alias" ON)
option(EXPOSE_ALL "Expose all submodule functions in namespace" OFF)

include(FetchContent)

# ==================================================================================================
# Helper function: process dependencies
# ==================================================================================================
function(process_deps dep_list out_targets)
    set(targets)
    foreach(dep IN LISTS ${dep_list})
        # Check if it's a git dependency (contains |)
        string(FIND "${dep}" "|" has_pipe)
        # Check if it's a pkg-config dependency (starts with pkg::)
        string(FIND "${dep}" "pkg::" is_pkg)

        if(has_pipe GREATER -1)
            # Parse: name|url|tag
            string(REPLACE "|" ";" dep_parts "${dep}")
            list(GET dep_parts 0 dep_name)
            list(GET dep_parts 1 dep_url)
            list(GET dep_parts 2 dep_tag)

            FetchContent_Declare(${dep_name}
                GIT_REPOSITORY "${dep_url}"
                GIT_TAG "${dep_tag}"
            )
            FetchContent_MakeAvailable(${dep_name})
            list(APPEND targets ${dep_name})
        elseif(is_pkg EQUAL 0)
            # pkg-config dependency: pkg::name
            string(SUBSTRING "${dep}" 5 -1 pkg_name)
            find_package(PkgConfig REQUIRED)
            pkg_check_modules(${pkg_name} REQUIRED IMPORTED_TARGET ${pkg_name})
            list(APPEND targets PkgConfig::${pkg_name})
        else()
            # Simple dependency - use find_package
            find_package(${dep} REQUIRED)
            list(APPEND targets ${dep}::${dep})
        endif()
    endforeach()
    set(${out_targets} ${targets} PARENT_SCOPE)
endfunction()

# ==================================================================================================
# Process library dependencies
# ==================================================================================================
process_deps(LIB_DEPS LIB_DEP_TARGETS)

# ==================================================================================================
# Main library
# ==================================================================================================
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "src/${PROJECT_NAME}/*.cpp")

if(LIB_SOURCES)
    add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_compile_definitions(${PROJECT_NAME} PUBLIC
        $<$<BOOL:${SHORT_NAMESPACE}>:SHORT_NAMESPACE>
        $<$<BOOL:${EXPOSE_ALL}>:OPTINUM_EXPOSE_ALL>
    )
else()
    add_library(${PROJECT_NAME} INTERFACE)
    target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    if(SHORT_NAMESPACE)
        target_compile_definitions(${PROJECT_NAME} INTERFACE SHORT_NAMESPACE)
    endif()
    if(EXPOSE_ALL)
        target_compile_definitions(${PROJECT_NAME} INTERFACE OPTINUM_EXPOSE_ALL)
    endif()
endif()

if(LIB_DEP_TARGETS)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LIB_DEP_TARGETS})
endif()

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# ==================================================================================================
# Installation
# ==================================================================================================
include(GNUInstallDirs)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
export(EXPORT ${PROJECT_NAME}Targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_NAME}::
)

# ==================================================================================================
# Examples
# ==================================================================================================
if(${PROJECT_NAME_UPPER}_BUILD_EXAMPLES)
    process_deps(EXAMPLE_DEPS EXAMPLE_DEP_TARGETS)
    set(ALL_EXAMPLE_DEPS ${LIB_DEP_TARGETS} ${EXAMPLE_DEP_TARGETS})

    file(GLOB example_sources CONFIGURE_DEPENDS examples/*.cpp)
    foreach(src_file IN LISTS example_sources)
        get_filename_component(exec_name "${src_file}" NAME_WE)
        add_executable(${exec_name} "${src_file}")
        target_compile_definitions(${exec_name} PRIVATE SHORT_NAMESPACE)
        target_link_libraries(${exec_name} ${PROJECT_NAME}::${PROJECT_NAME} ${ALL_EXAMPLE_DEPS})
        install(TARGETS ${exec_name} DESTINATION bin)
    endforeach()
endif()

# ==================================================================================================
# Tests
# ==================================================================================================
if(${PROJECT_NAME_UPPER}_ENABLE_TESTS)
    enable_testing()
    process_deps(TEST_DEPS TEST_DEP_TARGETS)
    set(ALL_TEST_DEPS ${LIB_DEP_TARGETS} ${TEST_DEP_TARGETS})

    file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS test/*.cpp)
    foreach(src_file IN LISTS test_sources)
        get_filename_component(test_name "${src_file}" NAME_WE)
        add_executable(${test_name} "${src_file}")
        target_compile_definitions(${test_name} PRIVATE SHORT_NAMESPACE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
        target_link_libraries(${test_name} ${PROJECT_NAME}::${PROJECT_NAME} ${ALL_TEST_DEPS})
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

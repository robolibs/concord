cmake_minimum_required(VERSION 3.15)

set(project_name concord)
project(${project_name} VERSION 2.5.0 LANGUAGES CXX)

add_compile_options(-Wall -Wextra -Wpedantic)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(params
    -Wno-reorder
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-but-set-variable
)

string(TOUPPER ${project_name} project_name_upper)
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)
include(FetchContent)

# --------------------------------------------------------------------------------------------------
set(ext_deps)
set(lib_deps)

# --------------------------------------------------------------------------------------------------
# Mandatory datapod dependency (like ../optinum). Prefer local checkout at ../datapod.
set(CONCORD_DATAPOD_SOURCE_DIR "" CACHE PATH "Path to a local datapod checkout (overrides ../datapod)")
if(CONCORD_DATAPOD_SOURCE_DIR AND EXISTS "${CONCORD_DATAPOD_SOURCE_DIR}/CMakeLists.txt")
  FetchContent_Declare(datapod SOURCE_DIR "${CONCORD_DATAPOD_SOURCE_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../datapod/CMakeLists.txt")
  FetchContent_Declare(datapod SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../datapod")
else()
  FetchContent_Declare(datapod GIT_REPOSITORY https://github.com/robolibs/datapod.git GIT_TAG 0.0.13)
endif()
FetchContent_MakeAvailable(datapod)
list(APPEND lib_deps datapod::datapod)


# --------------------------------------------------------------------------------------------------
# Collect all source files from src directory
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

# Create library - STATIC if we have sources, INTERFACE if header-only
if(LIB_SOURCES)
    add_library(${project_name} STATIC ${LIB_SOURCES})
    target_include_directories(${project_name} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${project_name} PUBLIC ${lib_deps})
    # Always enable datapod short namespace alias: dp::
    target_compile_definitions(${project_name} PUBLIC SHORT_NAMESPACE)
    target_compile_options(${project_name} PRIVATE ${params})
else()
    add_library(${project_name} INTERFACE)
    target_include_directories(${project_name} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${project_name} INTERFACE ${lib_deps})
    target_compile_definitions(${project_name} INTERFACE SHORT_NAMESPACE)
endif()

# Allow users to link via `${project_name}::${project_name}`
add_library(${project_name}::${project_name} ALIAS ${project_name})

install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

#Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#Install and export the INTERFACE target
install(TARGETS ${project_name} EXPORT ${project_name}Targets)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_BUILD_EXAMPLES)
  find_package(rerun_sdk QUIET)
  if(TARGET rerun_sdk OR TARGET rerun_sdk::rerun_sdk)
    list(APPEND ext_deps rerun_sdk)
  endif()

  file(GLOB exe CONFIGURE_DEPENDS examples/*.cpp)
  # ----------------------------------------------
  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
    if(TARGET rerun_sdk OR TARGET rerun_sdk::rerun_sdk)
      target_compile_definitions(${exec_name} PRIVATE HAS_RERUN)
    endif()
    target_compile_options(${exec_name} PRIVATE ${params})
    target_compile_definitions(${exec_name} PRIVATE SHORT_NAMESPACE)
    target_link_libraries(${exec_name} ${project_name}::${project_name} ${ext_deps})
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
  endforeach()
  # ----------------------------------------------
  foreach(exec IN LISTS exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    get_filename_component(exec_name "${exec}" NAME)
    file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
  endforeach()
  # ----------------------------------------------
  install(TARGETS
    ${exec_names}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  include(FetchContent)
  # Avoid network access during the "update" step (useful in restricted/offline environments).
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
  set(CONCORD_DOCTEST_SOURCE_DIR "" CACHE PATH "Path to a local doctest source tree (contains doctest/doctest.h)")
  if(NOT CONCORD_DOCTEST_SOURCE_DIR)
    file(GLOB _concord_doctest_candidates "${CMAKE_CURRENT_LIST_DIR}/../*/build/_deps/doctest-src")
    foreach(_cand IN LISTS _concord_doctest_candidates)
      if(EXISTS "${_cand}/doctest/doctest.h")
        set(CONCORD_DOCTEST_SOURCE_DIR "${_cand}" CACHE PATH "Path to a local doctest source tree (contains doctest/doctest.h)" FORCE)
        break()
      endif()
    endforeach()
  endif()

  if(CONCORD_DOCTEST_SOURCE_DIR AND EXISTS "${CONCORD_DOCTEST_SOURCE_DIR}/doctest/doctest.h")
    FetchContent_Declare(DocTest SOURCE_DIR "${CONCORD_DOCTEST_SOURCE_DIR}")
  else()
    FetchContent_Declare(
      DocTest
      GIT_REPOSITORY "https://github.com/onqtam/doctest"
      GIT_TAG        v2.4.11
    )
  endif()
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src CONFIGURE_DEPENDS test/*.cpp)

  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
    target_compile_options(${test_name} PRIVATE ${params})
    target_compile_definitions(${test_name} PRIVATE SHORT_NAMESPACE)
    target_link_libraries(${test_name} ${project_name}::${project_name} doctest_with_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

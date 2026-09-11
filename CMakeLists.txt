cmake_minimum_required(VERSION 3.15...3.27)
project(wow_navmesh LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT SKBUILD)
  message(WARNING "This CMakeLists.txt is meant to be driven by scikit-build-core (pip install).")
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# wow-navmesh only targets 64-bit Windows (matching TrinityCore/AzerothCore's own
# mmap tooling and the only platform prebuilt wheels are published for).
if (NOT WIN32)
  message(FATAL_ERROR
    "wow-navmesh only supports Windows. Building on '${CMAKE_SYSTEM_NAME}' is not supported.")
endif()

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR
    "wow-navmesh only supports 64-bit Python interpreters. "
    "A 32-bit build was detected (CMAKE_SIZEOF_VOID_P=4); reconfigure with a 64-bit toolchain/interpreter.")
endif()

set(DETOUR_SOURCE_DIR
  "${PROJECT_SOURCE_DIR}/extern/recastnavigation/Detour"
  CACHE PATH "Path to the Detour library source tree (Include/ and Source/)")

if (NOT EXISTS "${DETOUR_SOURCE_DIR}/Include/DetourNavMesh.h")
  message(FATAL_ERROR
    "Detour sources not found at '${DETOUR_SOURCE_DIR}'. "
    "The recastnavigation submodule looks uninitialized. Run:\n"
    "    git submodule update --init --recursive\n"
    "or point DETOUR_SOURCE_DIR at an existing Detour/ checkout (Include/ and Source/).")
endif()

# TrinityCore/AzerothCore mmap tiles are generated with DT_POLYREF64 enabled (a wider
# dtPolyRef changes the on-disk size of dtLink, and therefore every tile's byte layout).
# Keep this ON to read those tiles; turning it OFF targets a plain Detour navmesh built
# without that define instead. See src/bindings.cpp for the layout mismatch this guards.
option(WOW_NAVMESH_POLYREF64
  "Build against the DT_POLYREF64 tile layout used by TrinityCore/AzerothCore mmaps" ON)

if(NOT DEFINED Python_ROOT_DIR)
  execute_process(
    COMMAND python -c "import sys; print(sys.base_prefix)"
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE Python_ROOT_DIR)
endif()

find_package(Python 3.10
  REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(
  _wow_navmesh
  src/bindings.cpp
  "${DETOUR_SOURCE_DIR}/Source/DetourAlloc.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourAssert.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourCommon.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourNavMesh.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourNavMeshBuilder.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourNavMeshQuery.cpp"
  "${DETOUR_SOURCE_DIR}/Source/DetourNode.cpp"
)

target_include_directories(_wow_navmesh PRIVATE "${DETOUR_SOURCE_DIR}/Include")

if (WOW_NAVMESH_POLYREF64)
  target_compile_definitions(_wow_navmesh PRIVATE DT_POLYREF64 WOW_NAVMESH_POLYREF64=1)
else()
  target_compile_definitions(_wow_navmesh PRIVATE WOW_NAVMESH_POLYREF64=0)
endif()

install(TARGETS _wow_navmesh LIBRARY DESTINATION wow_navmesh)

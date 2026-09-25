cmake_minimum_required(VERSION 3.15)
project(ethercat_sdk_py LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- Python + nanobind ---
find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

# --- Third-party native deps ---
find_package(soem REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

# --- Locate ethercat_sdk + common sources ---
#
# The wheel is built directly from the project tree so the nanobind extension
# links in ethercat_sdk's implementation statically. This removes the need for
# a prior colcon build and produces a single self-contained .so.
get_filename_component(_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(ETHERCAT_SDK_DIR "${_repo_root}/hardware/ethercat_sdk"
    CACHE PATH "Path to ethercat_sdk package")
set(COMMON_DIR "${_repo_root}/common"
    CACHE PATH "Path to common package")

if(NOT EXISTS "${ETHERCAT_SDK_DIR}/src/master.cpp")
  message(FATAL_ERROR
    "ethercat_sdk sources not found at ${ETHERCAT_SDK_DIR}. "
    "Build from within the robo_control repo, or pass "
    "-DETHERCAT_SDK_DIR=/path/to/ethercat_sdk.")
endif()
if(NOT EXISTS "${COMMON_DIR}/include/common")
  message(FATAL_ERROR
    "common headers not found at ${COMMON_DIR}. "
    "Pass -DCOMMON_DIR=/path/to/common when building outside the repo.")
endif()

message(STATUS "ethercat_sdk sources: ${ETHERCAT_SDK_DIR}")
message(STATUS "common headers:       ${COMMON_DIR}")

# --- nanobind extension with ethercat_sdk sources baked in ---
nanobind_add_module(ethercat_sdk_ext
  STABLE_ABI
  NB_STATIC
  src/bindings.cpp
  "${ETHERCAT_SDK_DIR}/src/master.cpp"
)

set_target_properties(ethercat_sdk_ext PROPERTIES OUTPUT_NAME "_core")

target_include_directories(ethercat_sdk_ext PRIVATE
  "${ETHERCAT_SDK_DIR}/include"
  "${COMMON_DIR}/include"
)

target_link_libraries(ethercat_sdk_ext PRIVATE
  soem
  spdlog::spdlog_header_only
  Threads::Threads
)

# The package is laid out as `ethercat_sdk/_core.abi3.so`, so the auditwheel
# bundle dir resolves at `$ORIGIN/../ethercat_sdk.libs`.
set_target_properties(ethercat_sdk_ext PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../ethercat_sdk.libs"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS ethercat_sdk_ext LIBRARY DESTINATION ethercat_sdk)

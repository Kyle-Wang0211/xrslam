cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-test-test LANGUAGES CXX)

# [pw] 删除了 include_directories(test_feature_track PRIVATE ${CMAKE_SOURCE_DIR}/xrslam/src)。
#      三个错:include_directories 的签名是 (dir...),不吃 target 也不吃 PRIVATE ⇒
#      "test_feature_track" 和 "PRIVATE" 被当成两个相对路径目录塞进了全目录 include path;
#      而且 target 在本文件末尾才 add_executable,此处尚不存在;CMAKE_SOURCE_DIR 嵌入时指错。
#      正确写法挪到文件末尾(见 target_include_directories(test_feature_track ...))。
include(SuperBuildDepends)
superbuild_depend(googletest)

function(xr_add_test target)
    set(options)
    set(multiValueArgs LINKS SRCS)
    cmake_parse_arguments(_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${target} ${_TEST_SRCS})
    target_link_libraries(${target} ${_TEST_LINKS} depends::googletest)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endfunction()

xr_add_test(test_version SRCS src/test_main.cpp src/test_version.cpp)
target_link_libraries(test_version xrslam)

xr_add_test(test_se3_cost_function SRCS src/test_main.cpp src/test_se3_cost_function.cpp)
target_link_libraries(test_se3_cost_function xrslam)

xr_add_test(test_pnp SRCS src/test_main.cpp src/test_pnp.cpp)
target_link_libraries(test_pnp xrslam)

xr_add_test(test_feature_track SRCS src/test_main.cpp src/test_feature_track.cpp)
target_link_libraries(test_feature_track xrslam-extra-opencv-image xrslam-extra-yaml-config)
# [pw] 取代文件开头那条写错签名的 include_directories:必须在 target 创建之后,
#      而且用 XRSLAM_ROOT_DIR 而不是 CMAKE_SOURCE_DIR。
target_include_directories(test_feature_track PRIVATE ${XRSLAM_ROOT_DIR}/xrslam/src)

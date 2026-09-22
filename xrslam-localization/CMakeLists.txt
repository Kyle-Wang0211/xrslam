cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-localization)

include(SuperBuildDepends)
# [pw] CMAKE_SOURCE_DIR -> XRSLAM_ROOT_DIR(根 CMakeLists 里定义)。前者在本仓被
#      add_subdirectory/FetchContent 嵌入时指向外层工程根。这里也不能用 PROJECT_SOURCE_DIR:
#      本文件第 3 行自己调了 project(xrslam-localization),它等于本目录。
include_directories(${XRSLAM_ROOT_DIR}/xrslam-interface/include)

set(XRSLAM_LOCALIZATION_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_LOCALIZATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(xrslam-localization
  ${XRSLAM_LOCALIZATION_INCLUDE_DIR}/XRGlobalLocalizer.h
  ${XRSLAM_LOCALIZATION_SOURCE_DIR}/XRGlobalLocalizerInternal.cpp
  ${XRSLAM_LOCALIZATION_SOURCE_DIR}/XRGlobalLocalizerManager.cpp
)
target_link_libraries(xrslam-localization
  PUBLIC
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
)
target_include_directories(xrslam-localization
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_LOCALIZATION_INCLUDE_DIR}>
    ${XRSLAM_LOCALIZATION_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_LOCALIZATION_SOURCE_DIR}
)

set(XRSLAM_LOCALIZATION_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")

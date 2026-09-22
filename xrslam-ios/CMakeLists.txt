cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-ios)

# [pw] 原来用的是 option(),但 option() 的第三参是 BOOL 初值:"XXXXXXXXXX" / "xrslam"
#      都不在 ON/1/TRUE/YES 里 ⇒ 两个变量的实际值是 OFF,于是 DEVELOPMENT_TEAM 被填成
#      字符串 "OFF",BUNDLE_IDENTIFIER 变成 "OFF.visualizer"。字符串必须用 CACHE STRING。
set(IOS_DEVELOPMENT_TEAM "XXXXXXXXXX" CACHE STRING "Team ID for iOS app development")
set(APP_IDENTIFIER_PREFIX "xrslam" CACHE STRING "Common prefix for all iOS app identifiers")

if(TARGET xrslam  AND TARGET xrslam-localization)
  add_subdirectory(visualizer)
endif()

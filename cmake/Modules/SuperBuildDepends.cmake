include(CMakeParseArguments)
include(FetchContent)

# [pw] 全部 CMAKE_SOURCE_DIR -> CMAKE_CURRENT_FUNCTION_LIST_DIR/..(= 仓库根)。
#      CMAKE_SOURCE_DIR 是最外层根:一旦本仓被 add_subdirectory/FetchContent 嵌进
#      Flutter 插件工程,它就指向插件根,所有 depends/options/external 模块全部找不到。
#      ⚠ 不能改成 CMAKE_CURRENT_LIST_DIR:实测在 function 体内它取的是**调用方**
#        listfile 的目录(xrslam/、xrslam-extra/ ...),会静默指错。
#        CMAKE_CURRENT_FUNCTION_LIST_DIR(CMake ≥3.17)才是**定义处**目录,实测正确。
set(SUPERBUILD_MODULES_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${SUPERBUILD_MODULES_DIR}")

if(NOT COMMAND superbuild_option)
  function(superbuild_option option_name)
    include("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../options/${option_name}.cmake")
  endfunction()
endif()

if(NOT COMMAND superbuild_depend)
  function(superbuild_depend depend_name)
    include("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../depends/${depend_name}.cmake")
  endfunction()
endif()

if(NOT COMMAND superbuild_extern)
  function(superbuild_extern extern_name)
    set(platform PLATFORM)
    cmake_parse_arguments(SUPERBUILD_EXTERN "" "${platform}" "" ${ARGN})
    if(NOT DEFINED SUPERBUILD_EXTERN_PLATFORM OR SUPERBUILD_EXTERN_PLATFORM STREQUAL AUTO)
      if(IOS)
        set(SUPERBUILD_EXTERN_PLATFORM IOS)
      elseif(ANDROID)
        set(SUPERBUILD_EXTERN_PLATFORM ANDROID)
      else()
        set(SUPERBUILD_EXTERN_PLATFORM SYSTEM)
      endif()
    endif()
    # [pw] CMAKE_SOURCE_DIR -> CMAKE_CURRENT_FUNCTION_LIST_DIR/..,理由同上。
    set(SUPERBUILD_EXTERN_SYSTEM_MODULE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../external/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_IOS_MODULE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../external/ios/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_ANDROID_MODULE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../external/android/${extern_name}.cmake")
    if(EXISTS "${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
      include("${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
    else()
      include("${SUPERBUILD_EXTERN_SYSTEM_MODULE}")
    endif()
  endfunction()
endif()
if(NOT TARGET depends::yaml-cpp)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::yaml-cpp expects options::modern-cpp")
  endif()
  FetchContent_Declare(
    depends-yaml-cpp
    URL            /Users/kaidongwang/Developer/xrslam-deps-tarballs/yamlcpp.tgz
    # [bench 2026-08-31] 同一 commit 的 tarball;git 长连接在本机网络下挂死。
  )
  FetchContent_GetProperties(depends-yaml-cpp)
  if(NOT depends-yaml-cpp_POPULATED)
    message(STATUS "Fetching yaml-cpp sources")
    FetchContent_Populate(depends-yaml-cpp)
    message(STATUS "Fetching yaml-cpp sources - done")
  endif()
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(${depends-yaml-cpp_SOURCE_DIR} ${depends-yaml-cpp_BINARY_DIR})
  add_library(depends::yaml-cpp INTERFACE IMPORTED GLOBAL)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_definitions(yaml-cpp
      PUBLIC
        _NOEXCEPT=noexcept
    )
  endif()
  target_include_directories(depends::yaml-cpp INTERFACE ${depends-yaml-cpp_SOURCE_DIR}/include)
  target_link_libraries(depends::yaml-cpp INTERFACE yaml-cpp options::modern-cpp)
  set(depends-yaml-cpp-source-dir ${depends-yaml-cpp_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-yaml-cpp-binary-dir ${depends-yaml-cpp_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-yaml-cpp-source-dir)
  mark_as_advanced(depends-yaml-cpp-binary-dir)
endif()

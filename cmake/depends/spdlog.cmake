if(NOT TARGET depends::spdlog)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::spdlog expects options::modern-cpp")
  endif()
  FetchContent_Declare(
    depends-spdlog
    URL            /Users/kaidongwang/Developer/xrslam-deps-tarballs/spdlog.tgz
    # [bench 2026-08-31] 同一 commit 的 tarball;git 长连接在本机网络下挂死。
  )
  FetchContent_GetProperties(depends-spdlog)
  if(NOT depends-spdlog_POPULATED)
    message(STATUS "Fetching spdlog sources")
    FetchContent_Populate(depends-spdlog)
    message(STATUS "Fetching spdlog sources - done")
  endif()
  set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "..." FORCE)
  add_subdirectory(${depends-spdlog_SOURCE_DIR} ${depends-spdlog_BINARY_DIR})
  add_library(depends::spdlog INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::spdlog INTERFACE spdlog::spdlog options::modern-cpp)
endif()

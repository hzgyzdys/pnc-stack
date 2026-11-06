cmake_minimum_required(VERSION 3.16)
project(pnc_core VERSION 0.1.0 LANGUAGES CXX)

# Plain CMake package (no ament): keeps the algorithms testable without ROS.
# colcon builds it through <build_type>cmake</build_type> in package.xml.

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PNC_CORE_BUILD_TESTS "Build unit tests" ON)
option(PNC_CORE_BUILD_TOOLS "Build offline simulation tools" ON)

add_library(pnc_core
  src/vehicle_model.cpp
  src/path_utils.cpp
  src/pure_pursuit.cpp
)
add_library(pnc_core::pnc_core ALIAS pnc_core)

target_compile_features(pnc_core PUBLIC cxx_std_17)
set_target_properties(pnc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(pnc_core PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Werror>
)
target_include_directories(pnc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Tools
if(PNC_CORE_BUILD_TOOLS)
  add_executable(track_sim tools/track_sim.cpp)
  target_link_libraries(track_sim PRIVATE pnc_core)
  target_compile_options(track_sim PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Werror>)
  install(TARGETS track_sim RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

# ---------------------------------------------------------------------------
# Tests
if(PNC_CORE_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(pnc_core_tests
    test/test_math_utils.cpp
    test/test_vehicle_model.cpp
    test/test_path_utils.cpp
    test/test_pure_pursuit.cpp
  )
  target_link_libraries(pnc_core_tests PRIVATE pnc_core GTest::gtest_main)
  gtest_discover_tests(pnc_core_tests)
endif()

# ---------------------------------------------------------------------------
# Install + CMake package export, so ROS packages can find_package(pnc_core).
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS pnc_core EXPORT pnc_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT pnc_coreTargets
  NAMESPACE pnc_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pnc_core
)
configure_package_config_file(cmake/pnc_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/pnc_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pnc_core
)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/pnc_coreConfigVersion.cmake
  COMPATIBILITY SameMinorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/pnc_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/pnc_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pnc_core
)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

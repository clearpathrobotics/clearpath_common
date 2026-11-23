cmake_minimum_required(VERSION 3.8)
project(clearpath_mock_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

option(CLEARPATH_MOCK_VELOCITY_TESTS
  "Enable mock velocity command and odometry integration tests" OFF)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mock_launch test/test_mock_launch.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_velocity_checks test/test_velocity_checks.py
    TIMEOUT 60
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(mock_platforms dd100 dd150 do100 do150 j100 a200 a300 r100 w200)
  foreach(platform IN LISTS mock_platforms)
    add_launch_test(test/test_platform_bringup.launch.py
      TARGET test_mock_${platform}
      TIMEOUT 150
      RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
      ENV
        "MOCK_TEST_PLATFORM=${platform}"
        "MOCK_TEST_RUN_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_runs"
        "MOCK_TEST_VELOCITY=false"
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    )
    set_tests_properties(test_mock_${platform} PROPERTIES
      RUN_SERIAL TRUE
    )
  endforeach()
  if(CLEARPATH_MOCK_VELOCITY_TESTS)
    foreach(platform IN LISTS mock_platforms)
      add_launch_test(test/test_platform_bringup.launch.py
        TARGET test_mock_velocity_${platform}
        TIMEOUT 240
        RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
        ENV
          "MOCK_TEST_PLATFORM=${platform}"
          "MOCK_TEST_RUN_ROOT=${CMAKE_CURRENT_BINARY_DIR}/test_runs"
          "MOCK_TEST_VELOCITY=true"
        APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
      )
      set_tests_properties(test_mock_velocity_${platform} PROPERTIES RUN_SERIAL TRUE)
    endforeach()
  endif()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

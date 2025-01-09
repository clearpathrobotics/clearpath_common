cmake_minimum_required(VERSION 3.8)
project(clearpath_generator_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(moveit_setup_srdf_plugins REQUIRED)

install(PROGRAMS
  ${PROJECT_NAME}/description/generate_description
  ${PROJECT_NAME}/discovery_server/generate_discovery_server
  ${PROJECT_NAME}/bash/generate_bash
  ${PROJECT_NAME}/semantic_description/generate_semantic_description
  ${PROJECT_NAME}/vcan/generate_vcan
  ${PROJECT_NAME}/zenoh_router/generate_zenoh_router
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(moveit_collision_updater src/moveit_collision_updater.cpp)
ament_target_dependencies(moveit_collision_updater
  moveit_setup_srdf_plugins
  moveit_setup_framework
)
install(
  TARGETS moveit_collision_updater
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/clearpath_generator_common
  INCLUDES DESTINATION lib/clearpath_generator_common
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # pytest
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_generator_bash.py
    test/test_generator_description.py
    test/test_generator_discovery_server.py
    test/test_generator_vcan.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()

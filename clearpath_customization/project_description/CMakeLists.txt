cmake_minimum_required(VERSION 3.20)
project(project_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(WARNING "${CMAKE_PROJECT_NAME} PYTHONPATH: $ENV{PYTHONPATH}")
message(WARNING "${CMAKE_PROJECT_NAME} CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH}")
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

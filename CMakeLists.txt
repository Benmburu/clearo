cmake_minimum_required(VERSION 3.5)
project(clearo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS
  scripts/straight_line_waypoints.py
  scripts/straight_line.py
  scripts/pid_straight_line_controller.py
  scripts/floor_coverage_controller.py
  scripts/nmpc_controller.py
  scripts/square_path.py
  scripts/nav2_straight_line_goal.py

  scripts/hybrid_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories - this installs everything in launch/ directory
install(DIRECTORY
  config
  description
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
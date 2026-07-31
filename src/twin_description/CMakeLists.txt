cmake_minimum_required(VERSION 3.8)
project(twin_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
endif()

# gz_ros2_control reads the <parameters> tag as a literal filesystem path. It
# expands neither package:// nor $(find ...), and the Panda URDF is handed to
# Gazebo as raw file contents, so there is no xacro pass to expand anything
# either. Bake this workspace's own install path in at configure time, so a
# fresh clone points at its own copy of panda_controllers.yaml rather than at
# the tree it was copied from.
set(TWIN_CONTROLLERS_YAML
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/panda_controllers.yaml")
configure_file(
  urdf/panda.urdf.in
  "${CMAKE_CURRENT_BINARY_DIR}/urdf/panda.urdf"
  @ONLY
)

# Copy urdf/ and config/ into the install space. Without this, ROS tools that
# resolve paths through the package share directory cannot find them, because
# they read the install space rather than the source tree.
install(
  DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "panda.urdf.in" EXCLUDE
)

# urdf/panda.urdf.in is the source of truth; the usable URDF exists only once
# configured, so it is installed from the build tree.
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/panda.urdf"
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()

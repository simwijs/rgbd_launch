# cmake_minimum_required(VERSION 2.8.3)
# project(rgbd_launch)

# find_package(catkin)

# catkin_package()

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
# )

# install(PROGRAMS scripts/hydro-upgrade-notice
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/
# )

# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   find_package(rostest REQUIRED)
#   roslaunch_add_file_check(launch)
#   roslaunch_add_file_check(test)
#   add_rostest(test/camera.test)
# endif()
cmake_minimum_required(VERSION 3.5)
project(rgbd_launch)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}/
  )
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/camera.test.py)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
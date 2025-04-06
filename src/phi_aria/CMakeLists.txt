cmake_minimum_required(VERSION 3.8)
project(phi_aria)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Aria QUIET)
if(Aria_FOUND)
  # message("Aria found with find_package")
  if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # message("Aria NOT found with find_package, searching manually")
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/include/Aria.h")
	  add_definitions(-DADEPT_PKG)
	  include_directories(/usr/local/Aria/include)
	  link_directories(/usr/local/Aria/lib)
	endif()
endif()

add_executable(phi_p3dx src/phi_p3dx.cpp src/Pose.cpp src/PhiROS2Aria.cpp)
target_include_directories(phi_p3dx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(phi_p3dx PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(phi_p3dx Aria)
ament_target_dependencies(phi_p3dx rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS phi_p3dx
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()

cmake_minimum_required(VERSION 3.8)
project(stereo_active)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  find_package(OpenMP REQUIRED)
  add_compile_options(-Wall -Wextra -Wpedantic -fopenmp)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.5.4 EXACT)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Manually specify the path to libnoise
set(LIBNOISE_INCLUDE_DIRS "/usr/local/include")
set(LIBNOISE_LIBRARIES "/usr/local/lib/libnoise-static.a")
include_directories(include ${LIBNOISE_INCLUDE_DIRS})

# add_executable(dc_motor src/dc_motor.cpp)
# target_link_libraries(dc_motor gpiod)
# ament_target_dependencies(dc_motor rclcpp std_msgs)

# add_executable(dc_pub src/dc_publisher.cpp)
# ament_target_dependencies(dc_pub rclcpp std_msgs)  

add_executable(noise_image src/noise_image.cpp)
target_link_libraries(noise_image ${OpenCV_LIBS} ${LIBNOISE_LIBRARIES} X11 Xinerama OpenMP::OpenMP_CXX)
ament_target_dependencies(noise_image rclcpp std_msgs std_srvs)  

install(TARGETS 
        # dc_motor 
        # dc_pub
        noise_image
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

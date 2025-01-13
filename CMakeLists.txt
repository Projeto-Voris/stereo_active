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
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Manually specify the path to gpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
include_directories(${GPIOD_INCLUDE_DIRS})
link_directories(${GPIOD_LIBRARY_DIRS})

# Manually specify the path to libnoise
set(LIBNOISE_INCLUDE_DIRS "/usr/local/include")
set(LIBNOISE_LIBRARIES "/usr/local/lib/libnoise-static.a")
include_directories(include ${LIBNOISE_INCLUDE_DIRS})

add_executable(gpio_control src/gpio/gpio_control.cpp)
target_link_libraries(gpio_control ${GPIOD_LIBRARIES})
ament_target_dependencies(gpio_control rclcpp std_msgs std_srvs)

add_executable(dc_pub src/gpio/dc_publisher.cpp)
ament_target_dependencies(dc_pub rclcpp std_msgs)  

add_executable(noise_image src/image/noise_image.cpp)
target_link_libraries(noise_image ${OpenCV_LIBS} ${LIBNOISE_LIBRARIES} X11 Xinerama OpenMP::OpenMP_CXX)
ament_target_dependencies(noise_image rclcpp std_msgs std_srvs)  

add_executable(stereo_acq src/image/stereo_acquisition.cpp)
target_link_libraries(stereo_acq ${OpenCV_LIBS})
ament_target_dependencies(stereo_acq rclcpp sensor_msgs cv_bridge message_filters std_srvs)  

add_executable(stereo_image_subscriber src/image/stereo_image_subscriber.cpp)
target_link_libraries(stereo_image_subscriber ${OpenCV_LIBS})
ament_target_dependencies(stereo_image_subscriber rclcpp sensor_msgs cv_bridge)


install(TARGETS 
        gpio_control 
        dc_pub
        stereo_acq
        noise_image
        stereo_image_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})
        
install(DIRECTORY
        scripts
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/inv_correlation_node.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

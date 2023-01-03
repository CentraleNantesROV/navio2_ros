cmake_minimum_required(VERSION 3.5)
project(navio2_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ADC.msg" "msg/PWM.msg" "msg/NavioLED.msg")
if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()
if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

include_directories(include)

# only use actual navio2 libs on the ROV / RaspberryPi
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" MATCHES "^(arm64|aarch64)$")
#if(TRUE)
    include_directories(Navio2/C++/Navio/Navio2 Navio2/C++/Navio/Common Navio2/C++/Navio)

    file(GLOB navio2_SRC
        "Navio2/C++/Navio/Navio2/*.h"
        "Navio2/C++/Navio/Common/*.h"
        "Navio2/C++/Navio/Navio2/*.cpp"
        "Navio2/C++/Navio/Common/*.cpp"
    )
    add_library(navio2 ${navio2_SRC})
    set(navio2 "navio2")
    install(TARGETS navio2 EXPORT navio2 LIBRARY DESTINATION lib)
    ament_export_libraries(navio2)
    ament_export_targets(navio2 HAS_LIBRARY_TARGET)
else()
    set(navio2 "")
    include_directories(src/dummy_navio2_include)
endif()

add_library(pwm src/pwm.cpp)
target_link_libraries(pwm ${navio2})
ament_target_dependencies(pwm rclcpp)

add_executable(pwm_example src/pwm_example.cpp)
target_link_libraries(pwm_example pwm ${navio2})
ament_target_dependencies(pwm_example rclcpp)

add_library(imu_node src/AHRS.cpp)
target_link_libraries(imu_node ${navio2})
ament_target_dependencies(imu_node rclcpp sensor_msgs rclcpp_components)

add_library(barometer_node src/Barometer.cpp)
target_link_libraries(barometer_node ${navio2})
ament_target_dependencies(barometer_node rclcpp sensor_msgs rclcpp_components)

add_library(adc_node src/ADC.cpp)
target_link_libraries(adc_node ${navio2})
ament_target_dependencies(adc_node rclcpp rclcpp_components)

add_library(led_node src/LED.cpp)
target_link_libraries(led_node ${navio2})
ament_target_dependencies(led_node rclcpp rclcpp_components)

# link with generated messages
if(${LEGACY_IDL})
    rosidl_target_interfaces(pwm ${PROJECT_NAME} "rosidl_typesupport_cpp")
    rosidl_target_interfaces(led_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
    rosidl_target_interfaces(adc_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(pwm "${cpp_typesupport_target}")
    target_link_libraries(led_node "${cpp_typesupport_target}")
    target_link_libraries(adc_node "${cpp_typesupport_target}")
endif()

    
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# executable from components
rclcpp_components_register_node(adc_node
  PLUGIN "navio2_ros::ADCReader"
  EXECUTABLE adc)
  
rclcpp_components_register_node(imu_node
  PLUGIN "navio2_ros::AHRS"
  EXECUTABLE imu)

rclcpp_components_register_node(led_node
  PLUGIN "navio2_ros::LED"
  EXECUTABLE led)
  
rclcpp_components_register_node(barometer_node
  PLUGIN "navio2_ros::Barometer"
  EXECUTABLE barometer)
  
# legacy include dir + nested since Humble
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS imu_node adc_node led_node barometer_node pwm
  EXPORT pwm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(pwm HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

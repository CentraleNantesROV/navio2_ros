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

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ADC.msg" "msg/PWM.msg" "msg/LED.msg")

# only use actual navio2 libs on the ROV / RaspberryPi
string(COMPARE EQUAL "${CMAKE_HOST_SYSTEM_PROCESSOR}" "arm64" WITH_NAVIO)

include_directories(include)

if(WITH_NAVIO)

    include_directories(Navio2/C++/Navio/Navio2 Navio2/C++/Navio/Common Navio2/C++/Navio)

    file(GLOB navio2_SRC
        "Navio2/C++/Navio/Navio2/*.h"
        "Navio2/C++/Navio/Common/*.h"
        "Navio2/C++/Navio/Navio2/*.cpp"
        "Navio2/C++/Navio/Common/*.cpp"
    )
    add_library(navio2 ${navio2_SRC})
    set(navio2 "navio2")
else()
    set(navio2 "")
    include_directories(src/dummy_navio2_include)
endif()

add_library(pwm_node src/pwm.cpp)
target_link_libraries(pwm_node ${navio2})
ament_target_dependencies(pwm_node rclcpp rclcpp_components)

add_executable(pwm_example src/pwm_example.cpp)
target_link_libraries(pwm_example pwm_node ${navio2})
ament_target_dependencies(pwm_example rclcpp class_loader)
rosidl_target_interfaces(pwm_example ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(imu_node src/AHRS.cpp)
target_link_libraries(imu_node ${navio2})
ament_target_dependencies(imu_node rclcpp sensor_msgs)

add_library(barometer_node src/Barometer.cpp)
target_link_libraries(barometer_node ${navio2})
ament_target_dependencies(barometer_node rclcpp sensor_msgs)

add_library(adc_node src/ADC.cpp)
target_link_libraries(adc_node ${navio2})
ament_target_dependencies(adc_node rclcpp)
rosidl_target_interfaces(adc_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(led_node src/LED.cpp)
target_link_libraries(led_node ${navio2})
ament_target_dependencies(led_node rclcpp)
rosidl_target_interfaces(led_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
    
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
  
install(DIRECTORY include/ DESTINATION include/)

install(TARGETS imu_node adc_node led_node barometer_node pwm_example 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS pwm_node
  EXPORT pwm_node
  LIBRARY DESTINATION lib)

if(WITH_NAVIO)
    ament_export_libraries(navio2 pwm_node)
else()
    ament_export_libraries(pwm_node)
endif()
ament_export_targets(pwm_node HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

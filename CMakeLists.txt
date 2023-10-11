cmake_minimum_required(VERSION 3.5.0)
project(assembler_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(choreonoid REQUIRED)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++17)
endif()

set(target CnoidAssemblerROSPlugin)

choreonoid_add_plugin(${target}
  src/AssemblerROSPlugin.cpp
  src/AssemblerROSManager.cpp
  )

ament_target_dependencies(${target} PUBLIC
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  )
#Choreonoid::CnoidBodyPlugin
#Choreonoid::RobotAssemblerPlugin
target_link_libraries(
  ${target} PUBLIC
  /home/irsl/sandbox/choreonoid_ws/install/lib/choreonoid-2.0/libCnoidRobotAssemblerPlugin.so
  /home/irsl/sandbox/choreonoid_ws/install/lib/libRobotAssembler.so
  )

#install(
#  TARGETS ${target}
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#  )

ament_package()
## https://qiita.com/osamu0329/items/3b417f8934688fd186ba
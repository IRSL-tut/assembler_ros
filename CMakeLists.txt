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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(choreonoid REQUIRED)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++17)
endif()

#set(target CnoidAssemblerROSPlugin)
set(target CnoidVRROSPlugin)
choreonoid_add_plugin(${target}
  src/VRROSPlugin.cpp
  src/VRROSManager.cpp
  src/ScenePublisher.cpp
  )

ament_target_dependencies(${target} PUBLIC
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  cv_bridge
  image_transport
  )
#Choreonoid::CnoidBodyPlugin
#Choreonoid::RobotAssemblerPlugin
#Choreonoid::RobotAssembler
#Choreonoid::CnoidRobotAssemblerPlugin
#target_link_libraries(
#  ${target} PUBLIC
#  ${CHOREONOID_PLUGIN_DIR}/libCnoidRobotAssemblerPlugin.so
#  ${CHOREONOID_LIB_DIR}/libRobotAssembler.so
#  )

#install(
#  TARGETS ${target}
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#  )

ament_package()
## https://qiita.com/osamu0329/items/3b417f8934688fd186ba

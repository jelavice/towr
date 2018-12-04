cmake_minimum_required(VERSION 2.8.3)
project(towr_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  towr_ros
  towr
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS
   LIBRARIES
   CATKIN_DEPENDS std_msgs roscpp towr_ros
   DEPENDS towr
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(extended_formulation_test
  test/extended_formulation_test.cc
)
target_link_libraries(extended_formulation_test 
    ${towr_LIBRARIES}
)

add_executable(m545_model_with_joints_jacobians_unit_test 
  test/m545_model_with_joints_jacobians_unit_test.cc
)
target_link_libraries(m545_model_with_joints_jacobians_unit_test
  ${catkin_LIBRARIES}
)

add_executable(m545_joints_variables_test 
  test/m545_joints_variables_test.cc
)
target_link_libraries(m545_joints_variables_test
  ${catkin_LIBRARIES}
)


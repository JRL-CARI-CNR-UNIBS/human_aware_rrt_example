cmake_minimum_required(VERSION 3.0.2)
project(human_aware_test)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  moveit_planning_helper
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES
   CATKIN_DEPENDS roscpp eigen_conversions
   DEPENDS EIGEN3
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/human_aware_test_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(multigoal_test_node src/multigoal_test_node.cpp)
add_dependencies(multigoal_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multigoal_test_node
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(roshand_demo)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  roshand_msg
)

catkin_package(
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ActionDemo
               src/ActionDemo.cpp
 )
add_dependencies(ActionDemo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(ActionDemo ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(ServiceDemo
               src/ServiceDemo.cpp
 )
add_dependencies(ServiceDemo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(ServiceDemo ${catkin_LIBRARIES} ${Boost_LIBRARIES})


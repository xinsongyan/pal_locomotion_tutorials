cmake_minimum_required(VERSION 2.8.3)
project(pal_locomotion_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pal_locomotion
  pal_talos_walk_impl
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pal_locomotion_tutorials
  CATKIN_DEPENDS roscpp pal_locomotion pal_talos_walk_impl pluginlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/balance_action.cpp
 )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(balance_action_node src/balance_action_node.cpp)
target_link_libraries(balance_action_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(static_steps_execution_node src/static_steps_execution_node.cpp)
target_link_libraries(static_steps_execution_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(static_steps_command src/static_steps_command.cpp)
target_link_libraries(static_steps_command ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_plugins src/pal_locomotion_tutorials_plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}  ${catkin_LIBRARIES})

###########
## Install ##
###########

install(TARGETS ${PROJECT_NAME}_plugins ${PROJECT_NAME} balance_action_node static_steps_execution_node static_steps_command
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(FILES pal_locomotion_tutorials.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

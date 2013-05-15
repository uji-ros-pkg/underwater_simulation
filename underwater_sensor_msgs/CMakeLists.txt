cmake_minimum_required(VERSION 2.8.3)
project(underwater_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp)
                                        
generate_messages(
  DEPENDENCIES roscpp
) 

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp 
)


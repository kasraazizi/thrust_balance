cmake_minimum_required(VERSION 3.0.2)
project(tb_framework)

 
find_package(catkin REQUIRED
roscpp
rospy
std_msgs
message_generation
serial
)
find_package(Threads REQUIRED)


add_message_files(
    FILES
#   Message1.msg
#   Message2.msg
tb_feedback.msg
tb_pwm.msg
)

 
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   )


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Threads_INCLUDE_DIRS}
                    )
 
catkin_package(
 
   CATKIN_DEPENDS
   message_runtime
   roscpp
)

set (AA_SOURCES
    src/thrust_balance_framework.cpp

    )

add_executable(tb_framework ${AA_SOURCES})
target_link_libraries(tb_framework
    ${catkin_LIBRARIES}
    Threads::Threads
    )

#add_dependencies(framework framework_thrust_balance)


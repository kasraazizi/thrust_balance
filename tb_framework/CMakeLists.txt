cmake_minimum_required(VERSION 3.0.2)
project(tb_framework)

 
find_package(catkin REQUIRED
roscpp
rospy
std_msgs
message_generation
)

 
 
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

 
catkin_package(
 
   CATKIN_DEPENDS message_runtime
)
 

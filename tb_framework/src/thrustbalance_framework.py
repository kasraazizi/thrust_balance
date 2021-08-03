#!/usr/bin/env python 

"""
Software License Agreement (BSD)

\file      thrustbalance_framework.py
\authors   Kasra Azizi <kasraazizi1375@gmail.com>
           Ali Shahrudi <alishahrudi@gmail.com>
\copyright Copyright (c) 2021, MRL UAV Lab (Qazvin Islamic Azad University), All rights reserved.

 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import time 
import serial 
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState 
from tb_framework.msg import tb_feedback, tb_pwm
from functools import partial 

# ros initialization
rospy.init_node('thrustbalance_framework', log_level=rospy.INFO )
arduinoPublisher = rospy.Publisher('/tb/feedback', tb_feedback, queue_size=10)        
joint_state_publisher = rospy.Publisher('tb/joint_states', JointState, queue_size=10)

fb_msg = tb_feedback()
joint_msg = JointState()

def serial_start():
    time.sleep(1)
    # serial initialization
    serial_ = serial.Serial('/dev/ttyUSB0', 9600)    
    serial_.flushInput()
    serial_.flushOutput()
    return serial_ 

def arduino_start(serial_obj):
    # waiting for arduino initialization. loop breaks after recives "OK"
    rospy.loginfo('wait for arduino to initialize')
    serial_respond_time_offset = time.time()
    restart_try = 0 
    while True :       
        sensor_data = serial_obj.read_all()

        if not sensor_data == '':                     
            if sensor_data.rfind('not found') != -1:
                rospy.logerr("mpu6050 not found")                 
                rospy.signal_shutdown("mpu6050 not found")                           
#                serial_obj.close()
                serial_obj.__exit__()
                time.sleep(1)
                serial_obj = serial_start()

            sensor_data.rfind('not found')
            if sensor_data.rfind('OK') != -1:
                rospy.loginfo("Arduino's starting operation done in " + \
                    str(time.time()-serial_respond_time_offset)+ ' seconds')
                break
            rospy.loginfo(sensor_data)

        if (time.time()-serial_respond_time_offset) > 6:
            restart_try += 1 
            serial_respond_time_offset = 0
            serial_obj.flushInput()
            serial_obj.flushOutput()            
            serial_obj.__exit__()
            rospy.logerr("serial not responding --> restarting serial")        
            rospy.logerr("restart try=" + str(restart_try))                                  
            serial_obj = serial_start()
            time.sleep(1)
#            serial_obj = serial_start()
            if restart_try == 3:
                rospy.logerr("couldn't recive propper data from arduino")
                rospy.signal_shutdown("couldn't recive propper data from arduino")
                serial_obj.flushInput()
                serial_obj.flushOutput()
                time.sleep(1)
                serial_obj.__exit__()
                exit(1) 

ser = serial_start()
arduino_start(ser)

# initialize start time 
statring_time_offset = time.time()      
def arduino_framework_cb(cmd_vel_msg): 
    """ros subscriber callback function to send pwm commands to arduino via serial port 
        and publish recived imu data plus it's related pwm

    Args:
        cmd_vel_msg (tb_pwm_msg): custom pwm messages 
    """
    # current time for headers & calculate frequency 
    now =  rospy.get_time() - statring_time_offset
    
    # translating  pwm commands to custom valid message structure(Lxx-xxR\n) & sending to arduino 
    standard_message = "L"+ str(cmd_vel_msg.leftpwm)+"-"+ str(cmd_vel_msg.rightpwm)+"R/n"
    ser.write("L"+ str(cmd_vel_msg.leftpwm)+"-"+ str(cmd_vel_msg.rightpwm)+"R\n")
    
    # a loop to check validation of feedback data from arduino and 
    #    calculating frequency of data communication  
    loop_delay_start = time.time()
    while True:
        sensor_data = ser.read_all() 
        if sensor_data.endswith('R\n'):   

            # allocating data to fb_msg for publish
            fb_msg.header.stamp = rospy.Time.from_sec(now)
            fb_msg.frequency  =  1/(time.time()-loop_delay_start)
            fb_msg.roll = float(sensor_data.split('L')[0])
            fb_msg.leftpwm = int(sensor_data.split('L')[1].split('-')[0])
            fb_msg.rightpwm = int(sensor_data.split('L')[1].split('-')[1].split('R')[0])
            arduinoPublisher.publish(fb_msg)

            # allocating data to joint_msg for publish
            joint_msg.header.stamp =  rospy.Time.from_sec(rospy.get_time())
            joint_msg.position = [3.14 * float(sensor_data.split('L')[0]) / 180 , 0, 0]
            joint_msg.name =  ["roll", "right_motor_to_propeller", "left_motor_to_propeller"]
            joint_msg.effort = []
            joint_msg.velocity = []
            joint_msg.header.frame_id = ''
            joint_state_publisher.publish(joint_msg)

            break      
        elif sensor_data.rfind('wrong') != -1 :
            rospy.logwarn("wrong input data")

            break    
        elif sensor_data.rfind('wire') != -1 :
            rospy.logwarn("arduino reseted")            
            arduino_start(ser)
            break
     
         
if __name__ == '__main__':
    try:         
        arduinoSubscriber = rospy.Subscriber("/tb/cmd_pwm", tb_pwm, arduino_framework_cb)
        rospy.spin() 

    except rospy.ROSInterruptException:        
        rospy.logerr("ROS Interrupt Exception")
    except KeyboardInterrupt:
        rospy.logwarn("framework shutdown")
        rospy.signal_shutdown()        
        ser.close()
        exit(0)
 

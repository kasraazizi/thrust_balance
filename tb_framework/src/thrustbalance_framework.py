#!/usr/bin/env python 
#%%
import time 
import serial 
import rospy
from std_msgs.msg import String, Float32
from tb_framework.msg import tb_feedback, tb_pwm


#%%  
ser = serial.Serial('/dev/ttyUSB0', 9600)
ser.flushInput()
ser.flushOutput()
print("wait")
while True :
    time.sleep(1)
    sensor_data = ser.read_all() 
    print(sensor_data)
    if sensor_data.rfind('OK') != -1:
        break


print(ser.read_all())
 
#%% 
rospy.init_node('thrustbalance_framework') 
arduinoPublisher = rospy.Publisher('/tb/feedback', tb_feedback, queue_size=10)        
fb_msg = tb_feedback()

statring_time_offset = time.time()      
def arduino_framework(cmd_vel_msg): 
    
    now =  rospy.get_time() - statring_time_offset
    fb_msg.header.stamp = rospy.Time.from_sec(now)
   
    standard_message = "L"+ str(cmd_vel_msg.leftpwm)+"-"+ str(cmd_vel_msg.rightpwm)+"R/n"
    ser.write("L"+ str(cmd_vel_msg.leftpwm)+"-"+ str(cmd_vel_msg.rightpwm)+"R\n")
        
    loop_delay_start = time.time()
    while True:
        sensor_data = ser.read_all() 
        if sensor_data.endswith('R\n'):   
            frequency = 1/(time.time()-loop_delay_start)                
            break      
        elif sensor_data.startswith('w') :
            print("wrong input data")
            break        
    
    fb_msg.frequency  = frequency         
    fb_msg.roll = float(sensor_data.split('L')[0])
    fb_msg.leftpwm = int(sensor_data.split('L')[1].split('-')[0])
    fb_msg.rightpwm = int(sensor_data.split('L')[1].split('-')[1].split('R')[0])
    arduinoPublisher.publish(fb_msg)
    print(fb_msg)

    
def main():
    
    arduinoSubscriber = rospy.Subscriber("/tb/cmd_pwm", tb_pwm, arduino_framework)
    rospy.spin() 

if __name__ == '__main__':
    try:
        main()
        
       
    except rospy.ROSInterruptException:        
        pass
    except KeyboardInterrupt:
        rospy.signal_shutdown()
 
# %% 
  

# %%

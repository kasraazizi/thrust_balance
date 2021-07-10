#%%

#!/usr/bin/env python 
import time 
import serial 
import rospy
from std_msgs.msg import String, Float32
from tb_framework.msg import tb_message

import tbpwm 

#%%  
ser = serial.Serial('/dev/ttyUSB0', 9600)

print("wait")
while True :
    time.sleep(1)
    sensor_data = ser.read_all() 
    print(sensor_data)
    if sensor_data.rfind('OK') != -1:
        break


print(ser.read_all())
 
#%% 
rospy.init_node('thrustbalance_feedback') 
arduinoPublisher = rospy.Publisher('/tb/feedback', tb_message, queue_size=10)        
msg = tb_message()


def arduino_publisher():      

    # PWM = pwmGenerator
    statring_time_offset = time.time()  
    while not rospy.is_shutdown():
        now =  rospy.get_time() - statring_time_offset
        msg.header.stamp = rospy.Time.from_sec(now)
        
        # pwm_value = pwmgen()
        
        # ser.write("L",pwmgen,'-',pwmgen,'\n')
        ser.write("L10-10R\n")
        loop_delay_start = time.time()
        while True:
            sensor_data = ser.read_all() 
            if sensor_data.endswith('R\n'):   
                frequency = 1/(time.time()-loop_delay_start)                
                break         
        
        msg.frequency  = frequency         
        msg.roll = float(sensor_data.split("L")[0])
        msg.leftpwm = int(sensor_data[-7 : -5])
        msg.rightpwm = int(sensor_data[-4 : -2])
        arduinoPublisher.publish(msg)
        print(msg)
 

def main():
    
    arduino_publisher()

if __name__ == '__main__':
    try:
        main()
       
    except rospy.ROSInterruptException:        
        pass
    except KeyboardInterrupt:
        rospy.signal_shutdown()
 
# %% 

print("L",pwmgen.lpwm,'-',pwmgen.rpwm,'\n')
# %%
pwm = tbpwm.ManualPwm()
 


# %%

import rospy
import math, numpy
from simple_pid import PID
from tb_framework.msg import tb_feedback, tb_pwm

#%%
rospy.init_node("pwm_generator")
pwm_pub = rospy.Publisher("/tb/cmd_pwm", tb_pwm, queue_size=10)
#%%
def controller_allocation(U_thrust , U_roll ):
         
    left_pwm  =  abs(U_thrust - (U_roll*0.5) ) 
    right_pwm =  abs(U_thrust + (U_roll*0.5) )

    if right_pwm < 1: right_pwm=1
    if left_pwm < 1: left_pwm=1

    return [left_pwm, right_pwm]


def controller(actual, desired=0):
   
  # pid:
  kp = 1
  ki = 10
  kd = 1

  c = PID(kp, ki, kd, desired, output_limits=(-30, 30))
  u = c.__call__(actual)
  u = round(u)
   
  pwm_msg = tb_pwm()
  [pwm_msg.leftpwm, pwm_msg.rightpwm] = controller_allocation(abs(u)+25, u)
    
  pwm_pub.publish(pwm_msg)
  print(u, pwm_msg)
  

def controller_cb(fb_msg):    
  
  controller(round(fb_msg.roll, 4), 0 )  

def main():
  state_sub = rospy.Subscriber("/tb/feedback", tb_feedback, controller_cb)
  rospy.spin()
  
if __name__ == '__main__':
    try:
        main()       
    except rospy.ROSInterruptException:        
        pass
    except KeyboardInterrupt:
        rospy.signal_shutdown()
 
#%%
 
 
# %%

 
#%%
#!/usr/bin/env python 
import sys, select, tty, termios
import rospy
from tb_framework.msg import tb_pwm, tb_feedback
#%% ROS inital
 
rospy.init_node("pwmGenerator")
pwmpublisher = rospy.Publisher("/tb/cmd_pwm", tb_pwm, queue_size=10)
pwm_message = tb_pwm()
rate = rospy.Rate(10)
#%% pwm controller 

def manualpwm():
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    lpwm = 0
    rpwm = 0
    while True:
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1)
            if key == 'w':
                lpwm = lpwm + 1
                rpwm = rpwm + 1
            
            if key == 's':
                lpwm = lpwm - 1
                rpwm = rpwm - 1

            if key == 'a':
                lpwm = lpwm + 1
               

            if key == 'd':
              
                rpwm = rpwm + 1
            
            if key == 'q':
                print("exit")
                break

        if lpwm <0: lpwm=0
        if rpwm <0: rpwm=0
         
        pwm_message.leftpwm = lpwm
        pwm_message.rightpwm = rpwm
        pwmpublisher.publish(pwm_message)
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

manualpwm()
pwm_message.leftpwm = 0
pwm_message.rightpwm = 0
pwmpublisher.publish(pwm_message)
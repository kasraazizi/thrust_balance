#%%
    
from typing import List

def controller_allocation(U_thrust : int , U_roll : int) -> List :
         
    left_pwm  =  abs(U_thrust - (U_roll*0.5) ) 
    right_pwm =  abs(U_thrust + (U_roll*0.5) )

    if right_pwm < 1: right_pwm=1
    if left_pwm < 1: left_pwm=1

    return [left_pwm, right_pwm]

    
 
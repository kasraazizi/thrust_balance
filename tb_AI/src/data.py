import csv
import numpy as np
from matplotlib import pyplot as plot
import rospy






class SubPublisher():
    def __init__(self):
        subscriber = rospy.Subscriber("/tb/feedback", tb_pwm, self.callback)
        publisher  = rospy.Publisher('/tb/cmd_pwm', tb_feedback, queue_size=10)


    def callback(self):
        #TODO: save the incoming callback to csv
        pass




    def pulish(self):
        #TODO: publish the wave form rapiddly
        pass


    def sineGenerator(self, start, stop, step, amp):
        '''
        function that generate sine wave
        input: start , stop , step
        output: a ndarray list of sin values
        '''
        time = np.arange(start,stop,step)
        return np.sin(time) * amp


#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, navio
from moby.msg import PWMCommand

pwms = [navio.pwm.PWM(chan) for chan in range(0, 14)]

for pwm in pwms:
    pwm.initialize()

def callback(data):
    for pwm, throt in zip(pwms, data.duty_cycles):
        if throt >= 0:
            pwm.set_period(50)
            pwm.enable()
            pwm.set_duty_cycle(throt)
        elif pwm.is_enabled:
            pwm.disable()
    
def navio_pwm():
    rospy.init_node("navio_pwm")
    
    rospy.Subscriber("navpwm", PWMCommand, callback)
    
    rospy.spin()
    
if __name__ == "__main__":
    navio_pwm()
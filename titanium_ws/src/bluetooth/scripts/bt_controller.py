#!/usr/bin/env python

import pygame
import rospy
from std_msgs.msg import Int32MultiArray

def remap_value(value, fromLow, fromHigh, toLow, toHigh):
    return int(toLow + (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow))

def kinematic(motor, vy, vw):
    if(motor == 0):
        return int(vy + vw)
    elif(motor == 1):
        return int(vy - vw)

def main():
    rospy.init_node('gamepad_node')
    

    pygame.init()
    pygame.joystick.init()
    

    if pygame.joystick.get_count() == 0:
        rospy.logerr("No gamepad detected!")
        pygame.quit()
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    rospy.loginfo(f"Gamepad detected: {joystick.get_name()}")
    
    pub = rospy.Publisher('/controller/pub', Int32MultiArray, queue_size=10)
    
    rate = rospy.Rate(1000) 
    msg = Int32MultiArray()
    
    while not rospy.is_shutdown():
        pygame.event.pump()
        
        ly = -remap_value(joystick.get_axis(1), -1.0, 1.0, -255, 255)
        rx = remap_value(joystick.get_axis(2), -1.0, 1.0, -255, 255)

        motor_a = kinematic(0, ly, rx)
        motor_b = kinematic(1, ly, rx)

        msg.data = [motor_a, motor_b, 0, 0]
        
        pub.publish(msg)
        rate.sleep()
    
    pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
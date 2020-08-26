#!/usr/bin/env python3

import rospy
from neon.msg import Ref_speed

if __name__ == '__main__':
    rospy.init_node('Temp_user_input', anonymous=True)
    pub = rospy.Publisher('/Ref_speed', Ref_speed, queue_size=5)

    while not rospy.is_shutdown():    
        dir = input("Enter direction, w, a, s, d ")
        msg = Ref_speed()
        speed_L = 0
        speed_R = 0
        if dir == 'w':
            speed_L = 0.2
            speed_R = 0.2
        elif dir == 's':
            speed_L = 0
            speed_R = 0
        elif dir == 'a':
            speed_L = -0.2
            speed_R = 0.2
        elif dir == 'd':
            speed_L = 0.2
            speed_R = -0.2

        msg.left = speed_L
        msg.right = speed_R

        pub.publish(msg)

    
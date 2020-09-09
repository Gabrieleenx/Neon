#!/home/gabriel/anaconda3/bin/python
# for laptop 

import rospy
from neon.msg import Encoder_count


if __name__ == '__main__':
    rospy.init_node('Arduino_com', anonymous=True)
    pub = rospy.Publisher('/encoder_count', Encoder_count, queue_size=5)
    rate = rospy.Rate(10) # 10hz
    encoder_L = 0
    encoder_R = 0
    while not rospy.is_shutdown():
        encoder_L += 10
        encoder_R += 11

        msg = Encoder_count()
        msg.header.stamp = rospy.Time.now()

        msg.left = encoder_L
        msg.right = encoder_R

        pub.publish(msg)

        rate.sleep()
   
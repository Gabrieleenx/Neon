#!/usr/bin/env python3

import serial, time
import rospy
from neon.msg import Encoder_count, Ref_speed

arduino = serial.Serial("/dev/ttyUSB0", 9600)
time.sleep(4)
speed_L = 0 # in m/s
speed_R = 0

encoder_L = 0
encoder_R = 0

# msg form arduino ";value_L,value_R," where value_L = speed_L*1000
def send_ref_speed(data):
        # convert data to sendable int
        Value_L = int(data.left * 1000)
        Value_R = int(data.right * 1000)
        # create message
        msg = ';' + str(Value_L) + ',' + str(Value_R) + ','
        # send message
        arduino.write(msg.encode())
        print(msg)

def revive_enoder():
    pub = rospy.Publisher('/encoder_count', Encoder_count, queue_size=5)
    
    while not rospy.is_shutdown():
    
        # read line is blocking if no data available 
        data = arduino.readline()[:-2] # keep everyting but the new line 
        if data: # if there is data and not just a blank line
            data_str = str(data)
            data_str_vals = data_str[2:-1].split(" ") # remove b' and ' from byte array
            if data_str_vals[0] == "HelloNeon":
                data_str_L_R = data_str_vals[1].split(",") # split numbers
                try: # try turn into int, if data corruption then except
                    encoder_L = int(data_str_L_R[0])
                    encoder_R = int(data_str_L_R[1])

                    msg = Encoder_count()
                    msg.header.stamp = rospy.Time.now()

                    msg.left = encoder_L
                    msg.right = encoder_R

                    pub.publish(msg)
                except:
                    print('could not convert to int')



if __name__ == '__main__':
    rospy.init_node('Arduino_com', anonymous=True)
    rospy.Subscriber("/Ref_speed", Ref_speed, send_ref_speed)
    revive_enoder()
   

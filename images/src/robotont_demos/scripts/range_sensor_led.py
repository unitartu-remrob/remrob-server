#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from robotont_msgs.msg import LedModulePixel
import time
import math

ranges_ = [0]*12
LED_TO_SENSOR_RATIO = 18.0 / 12.0

def range_callback(data):
    #Parse sensor ID from frame_id field.
    sensor_id = int(data.header.frame_id[13:]) #TODO: assuming 'range_sensor_ID' make more dynamic
    ranges_[sensor_id] = data.range


def led_controller():
    # Starts a new node
    rospy.init_node('robotont_range_led_controller', anonymous=True)
    rospy.Subscriber("/robotont/range", Range, range_callback)
    led_pub = rospy.Publisher(
        '/robotont/led_pixel', LedModulePixel, queue_size=10)
    led_msg = LedModulePixel()

    led_msg.idx = 0
    led_msg.color.r = 0
    led_msg.color.g = 0
    led_msg.color.b = 0
    #rospy.sleep(0.05)
    led_pub.publish(led_msg)
    rospy.sleep(1)

    # round effect
    ''''while not rospy.is_shutdown():
	    for i in range(0, 18):
		led_msg.idx = i
		led_msg.color.r = 0
		led_msg.color.g = 255
		led_msg.color.b = 0
		#rospy.sleep(0.05)
		led_pub.publish(led_msg)
		rospy.sleep(0.1)
	    for i in range(0, 18):
		led_msg.idx = i
		led_msg.color.r = 0
		led_msg.color.g = 0
		led_msg.color.b = 0
		#rospy.sleep(0.05)
		led_pub.publish(led_msg)
		rospy.sleep(0.1)
'''
    while not rospy.is_shutdown():
        for i, dist in enumerate(ranges_):
            led_msg.idx = i * LED_TO_SENSOR_RATIO
            # default to green
            led_msg.color.r = 0
            led_msg.color.g = 255
            led_msg.color.b = 0

            if (dist < 2.5): # continuous gradient in range from 0 - 2.5 meters
                led_msg.color.r = max(min(255 - dist / 2.5 * 255, 255), 0)
                led_msg.color.g = max(min(dist / 2.5 * 255, 255), 0)
                led_msg.color.b = 0
            elif (dist > 10.0): # fault out of range readings will appear as blue
                led_msg.color.r = 0
                led_msg.color.g = 0
                led_msg.color.b = 255
            rospy.sleep(0.1)
            led_pub.publish(led_msg) # publish the led message

        rospy.sleep(0.1) # 10 Hz

if __name__ == '__main__':
    try:
        led_controller()
    except rospy.ROSInterruptException:
        pass

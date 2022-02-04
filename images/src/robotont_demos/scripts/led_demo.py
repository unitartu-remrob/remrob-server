#!/usr/bin/env python
import rospy
from robotont_msgs.msg import LedModuleSegment, ColorRGB
import time
import math


def doLED():
    # Starts a new node
    rospy.init_node('robotont_led_publisher', anonymous=True)
    led_publisher = rospy.Publisher(
        '/robotont/led_segment', LedModuleSegment, queue_size=5)
    led_msg = LedModuleSegment()


    LED_COUNT=60

    while not rospy.is_shutdown():

        led_msg.idx_start = 0
        led_msg.colors = []
        color = ColorRGB()

        t=0.1
        t1=0.1
        c_max=200
        c_step=10
        led_step=1

        #One By One
        color.r = 100
        color.g = 0
        color.b = 0
        led_msg.colors = []
        for i in range(led_step):
            led_msg.colors.append(color)
        for i in range(0, LED_COUNT,led_step):
            led_msg.idx_start = i
            led_publisher.publish(led_msg)
            rospy.sleep(t1)
            if rospy.is_shutdown():
                break

        color.r = 0
        color.g = 100
        color.b = 0
        led_msg.colors = []
        for i in range(5):
            led_msg.colors.append(color)
        for i in range(0, LED_COUNT,led_step):
            led_msg.idx_start = LED_COUNT - i - led_step+1
            led_publisher.publish(led_msg)
            rospy.sleep(t1)
            if rospy.is_shutdown():
                break

        color.r = 0
        color.g = 0
        color.b = 100
        led_msg.colors = []
        for i in range(5):
            led_msg.colors.append(color)
        for i in range(0, LED_COUNT,led_step):
            led_msg.idx_start = i
            led_publisher.publish(led_msg)
            rospy.sleep(t1)
            if rospy.is_shutdown():
                break


        color.r = 0
        color.g = 0
        color.b = 0
        led_msg.colors = []
        for i in range(5):
            led_msg.colors.append(color)
        for i in range(0, LED_COUNT,5):
            led_msg.idx_start = i
            led_publisher.publish(led_msg)
            rospy.sleep(t1)
            if rospy.is_shutdown():
                break


        led_msg.idx_start=0
        #UP RED
        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = c
            color.g = 0
            color.b = 0
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)
            if rospy.is_shutdown():
                break

        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = c_max-c-c_step
            color.g = 0
            color.b = 0
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)
            if rospy.is_shutdown():
                break

        #UP GREEN
        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = 0
            color.g = c
            color.b = 0
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)
            if rospy.is_shutdown():
                break

        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = 0
            color.g = c_max-c-c_step
            color.b = 0
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)
            if rospy.is_shutdown():
                break

        #UP BLUE
        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = 0
            color.g = 0
            color.b = c
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)

        for c in range(0, c_max, c_step):
            led_msg.colors = []
            color.r = 0
            color.g = 0
            color.b = c_max-c-c_step
            for i in range(0, LED_COUNT):
                led_msg.colors.append(color)
            led_publisher.publish(led_msg)
            rospy.sleep(t)
            if rospy.is_shutdown():
                break


if __name__ == '__main__':
    try:
        doLED()
    except rospy.ROSInterruptException:
        print("Exiting nicely...")
        pass

#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.cm as cm
from neostate_msgs.msg import StatusLEDArray
from neostate_msgs.msg import StatusLED

max_coord = 6
min_coord = 0
num_pixels = 12
inc_pixels = 1
brightness = 0.05
n_colors = 2500
colors = []
pub_neopixel = None

def callback(data):
    global colors
    global pub_neopixel

    coordinate = data.pose.pose.position.z
    value = np.interp(coordinate, (min_coord, max_coord), (0, 1))
    i_color = int(np.floor(value * (n_colors - 1)))
    color = colors[i_color]

    # create neopixel msg
    led = StatusLED()
    led.blinking = False
    led.red = color[0]*255*brightness
    led.green = color[1]*255*brightness
    led.blue = color[2]*255*brightness

    led_off = StatusLED()
    led_off.blinking = False
    led_off.red = 0
    led_off.green = 0
    led_off.blue = 0
    msg = StatusLEDArray()
    for i in range(0,num_pixels):
        if i%inc_pixels == 0:
            msg.LED_array.append(led)
        else:
            msg.LED_array.append(led_off)


    pub_neopixel.publish(msg)


def listener():
    global colors
    global pub_neopixel

    rospy.init_node('node_name')
    colors = cm.rainbow(np.linspace(0, 1, n_colors))

    pub_neopixel = rospy.Publisher("/set_status_led_array", StatusLEDArray, queue_size=1)

    rospy.Subscriber("odometry", Odometry, callback)
    # spin() simply keeps python from exiting until this node is stopped
    print("hello")
    rospy.spin()


if __name__ == '__main__':
    listener()

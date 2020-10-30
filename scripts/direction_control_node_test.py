import rospy
import serial
import sys
import os
import time
import i2cdev

import numpy as np

from ADS1115 import ADS1115
from PCA9685 import PCA9865
÷from std_msgs.msg import Int32, Float64

PUBLISHER_NAME = "direction"

SCALE_TO_ANGLE = 0.0094 # This scale is wrong. We need to wait for the new poten.
CONSTANT_TO_ANGLE = 0.0

SCALE_TO_ADC = 0.0094 # This scale is wrong. We need to wait for the new poten.
CONSTANT_TO_ADC = 0.0

DIRECTION_0 = 0
DIRECTION_1 = 1
ACTUAL_DIRECTION = DIRECTION_0

ADC_MIN_VALUE = 400       # This value is wrong for now. Wait for the new poten.
ADC_MAX_VALUE = 30000     # This value is wrong for now. Wait for the new poten.

HISTERESIS = 8000

MIN_ANGLE = (ADC_MIN_VALUE-6)*SCALE
MAX_ANGLE = (ADC_MAX_VALUE-6)*SCALE

PWM_DUTY = 10
PWM_FREQ = 6000

# ads115_0 = ADS1115()
pca9865 = PCA9865()

def init_hw ():
    # Setting the ADS1115 for the poten.
    # ads1115_0.set_channel(0)
    # ads1115_0.config_single_ended()
    # time.sleep(0.01)

    pca9865.reset_PWM()
    time.sleep(0.01)

    pca9865.set_freq(PWM_FREQ)
    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)

def left_spin ():

    # adc_value = ads1115_0.read_adc()

    pca9865.set_PWM(left_channel,PWM_FREQ)
    pca9865.set_PWM(right_channel,0)

    # To Do: consider a threshold instead of direct comparison
    # while (adc_value != adc_target):
    #     adc_value = ads1115_0.read_adc()

    time.sleep(100)

    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)


def right_spin ():

    # adc_value = ads1115_0.read_adc()

    pca9865.set_PWM(left_channel,PWM_FREQ)
    pca9865.set_PWM(right_channel,0)

    # To Do: consider a threshold instead of direct comparison
    # while (adc_value != adc_target):
    #     adc_value = ads1115_0.read_adc()

    time.sleep(100)

    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)


def direction_callback (angle):
    # adc_value = ads1115_0.read_adc()
    # actual_angle = adc_value*SCALE_TO_ANGLE + CONSTANT_TO_ANGLE
    # adc_goal = angle*SCALE_TO_ADC + CONSTANT_TO_ADC

    # if (angle > actual_angle):
    #     if ACTUAL_DIRECTION == DIRECTION_0:
    #         right_spin(adc_goal)
    #     elif ACTUAL_DIRECTION == DIRECTION_1:
    #         left_spin(adc_goal)

    # elif (angle < actual_angle):
    #     if ACTUAL_DIRECTION == DIRECTION_0:
    #         left_spin(adc_goal)
    #     elif ACTUAL_DIRECTION == DIRECTION_1:
    #         right_spin(adc_goal)

    if (angle < 0):
        left_spin()
    elif (angle > 0):
        right_spin()
    else:
        pca9865.set_PWM(left_channel,0)
        pca9865.set_PWM(right_channel,0)
        
    ros.spin()

def hw_driver():
    rospy.init_node('direction_control_node', anonymous=True)
    rospy.Subscriber(PUBLISHER_NAME, Float64, direction_callback)

if __name__ == '__main__':
    init_hw()
    hw_driver()
else:
    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)

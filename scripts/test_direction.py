import sys
import os
import time
import i2cdev

import numpy as np

import ADS1115
import PCA9685


PWM_DUTY = 1
PWM_FREQ = 100

ads1115_0 = ADS1115.ADS1115_0()
pca9865 = PCA9685.PCA9865()

left_channel = 0
right_channel = 1

pca9865.reset_PWM()
pca9865.set_freq(PWM_FREQ)
pca9865.set_PWM(left_channel,0)
pca9865.set_PWM(right_channel,0)
pca9865.set_duty(PWM_DUTY)
ads1115_0.set_channel(0)
ads1115_0.config_single_ended()
adc_value = ads1115_0.read_adc()
adc_prev = adc_value

def zeroes():
    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)

def tictac():
    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(left_channel,0)

    pca9865.set_PWM(left_channel,PWM_FREQ)
    time.sleep(0.1)
    pca9865.set_PWM(left_channel,0)
    
    
def test ():

    adc_value = ads1115_0.read_adc()
    adc_prev = adc_value
    if (adc_value > 17500):
        print("girando a la derecha")
        # pca9865.set_PWM(right_channel,PWM_FREQ)
        # pca9865.set_PWM(left_channel,0)
        adc_value = ads1115_0.read_adc()
        time.sleep(0.1)
        print (adc_value)
    pca9865.set_PWM(left_channel,0)
    pca9865.set_PWM(right_channel,0)
    print("Salida")
    time.sleep(0.1)
    if (adc_value < 18000): 
        print("girando a la izquierda")
        # pca9865.set_PWM(left_channel,PWM_FREQ)
        # pca9865.set_PWM(right_channel,0)
        adc_value = ads1115_0.read_adc()
        time.sleep(0.1)
        print(adc_value)
    time.sleep(0.1)
    
if __name__ == '__main__':
    #while (1):
    #    test()
    #zeroes()
    tictac()

import sys
import os
import time
import i2cdev

import numpy as np

import ADS1115

ads1115_0 = ADS1115.ADS1115_0()

def test ():
    ads1115_0.set_channel(0)
    ads1115_0.config_single_ended()
    adc_value = ads1115_0.read_adc()
    print(adc_value)

if __name__ == '__main__':
    test()



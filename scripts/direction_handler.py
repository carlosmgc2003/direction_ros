

import rospy
from std_msgs.msg import Int32
import time
import ADS1115
import PCA9685
import Jetson.GPIO as gpio

PWM_DUTY = 35 #7 valor minimo bueno vacio
PWM_FREQ = 6000 #Frecuencia buena
LEFT_END = 29
RIGHT_END = 33
PULSE_W = 0.05

ads1115_0 = ADS1115.ADS1115_0()
pca9865 = PCA9685.PCA9865()

left_channel = 0
right_channel = 1

pca9865.reset_PWM()
pca9865.set_freq(PWM_FREQ)
pca9865.set_PWM(left_channel,0)
pca9865.set_PWM(right_channel,0)
ads1115_0.set_channel(0)
ads1115_0.config_single_ended()
adc_value = ads1115_0.read_adc()
adc_prev = adc_value

gpio.setmode(gpio.BOARD)
gpio.setup(LEFT_END,gpio.IN)
gpio.setup(RIGHT_END, gpio.IN)

pub_eor = rospy.Publisher('end_of_race', Int32, queue_size=10)

def callback(data):
    if data.data == 0:
        # Blanquear de nuevo el PWM
        pca9865.set_PWM(left_channel,0)
        pca9865.set_PWM(left_channel,0)
        # Blanquear de nuevo el PWM
        pca9865.set_PWM(right_channel,0)
        pca9865.set_PWM(right_channel,0)
    elif data.data == 1 and not gpio.input(LEFT_END):
        # Blanquear el PWM
        pca9865.set_PWM(left_channel,0)
        pca9865.set_PWM(left_channel,0)
        # Pulso de giro
        pca9865.set_PWM(left_channel,PWM_DUTY)
        #time.sleep(PULSE_W)
    elif data.data == -1 and not gpio.input(RIGHT_END):
        # Blanquear el PWM
        pca9865.set_PWM(right_channel,0)
        pca9865.set_PWM(right_channel,0)
        # Pulso de giro
        pca9865.set_PWM(right_channel,PWM_DUTY)
        #time.sleep(PULSE_W)


    if gpio.input(RIGHT_END):
        rospy.loginfo("No puedo girar mas a la Izquierda!")
        pub_eor.publish(-1)
    if gpio.input(LEFT_END):
        rospy.loginfo("No puedo girar mas a la Derecha!")
        pub_eor.publish(1)

def direction_listener():
    rospy.init_node('direction_listener', anonymous=True)
    rospy.Subscriber('direction', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    direction_listener()

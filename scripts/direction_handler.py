

import rospy
from std_msgs.msg import Int32
#import ADS1115
import PCA9685

PWM_DUTY = 35 #7 valor minimo bueno vacio 35 valor maximo vacio
PWM_FREQ = 6000 #Frecuencia buena
LEFT_END = 29
RIGHT_END = 33
PULSE_W = 0.05

#ads1115_0 = ADS1115.ADS1115_0()
pca9865 = PCA9685.PCA9865()

left_channel = 0
right_channel = 1

pca9865.reset_PWM()
pca9865.set_freq(PWM_FREQ)
pca9865.set_PWM(left_channel,0)
pca9865.set_PWM(right_channel,0)
#ads1115_0.set_channel(0)
#ads1115_0.config_single_ended()
#adc_value = ads1115_0.read_adc()
#adc_prev = adc_value

left_eor = False
right_eor = False

def callbackDirection(data):
    if data.data == 0:
        # Blanquear de nuevo el PWM
        pca9865.set_PWM(left_channel,0)
        # Blanquear de nuevo el PWM
        pca9865.set_PWM(right_channel,0)
    elif data.data == 1 and not right_eor == True:
        # Blanquear el PWM
        pca9865.set_PWM(left_channel,0)
        # Pulso de giro
        pca9865.set_PWM(left_channel,PWM_DUTY)
    elif data.data == -1 and not left_eor == True:
        # Blanquear el PWM
        pca9865.set_PWM(right_channel,0)
        # Pulso de giro
        pca9865.set_PWM(right_channel,PWM_DUTY)


def callbackLeftEor(data):
    if data.data == 0:
        left_eor = False
    else:
        left_eor = True

def callbackRightEor(data):
    if data.data == 0:
        right_eor = False
    else:
        right_eor = True


def direction_listener():
    rospy.init_node('direction_listener', anonymous=True)
    rospy.Subscriber('direction', Int32, callbackDirection)
    rospy.Subscriber('eor_izquierdo', Int32, callbackLeftEor)
    rospy.Subscriber('eor_derecho', Int32, callbackRightEor)
    rospy.spin()

if __name__ == '__main__':
    direction_listener()

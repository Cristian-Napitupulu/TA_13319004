import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Byte

import time
import numpy as np

SETPOINT = 1

# Max encoder value (for calibration)
ENCOCALIB = 28000

# Motor speed
PWM1CALIBSPEED = 40
SCANNINGSPEED = 30

# PID-PD gain parameter
#============================================
#Gain settling time 6 detik
# FPA
KP   = 5.817591138435942888
KI   = 5.981472137038461138e-4
KD   = -1.863467370986635341e-1
KP_S = 1.158741366998004807e1
KD_S = 5.424602874698116750

SAMPLING_TIME = 50
STEADY_STATE_TIMEOUT = 500

# PWM Range (depend on deadband)
IN_MIN = 0
IN_MAX = 5
OUT_MIN = 17
OUT_MAX = 55

# Hoisting motor related
HOISTING_MIN = 20
HOISTING_CABLE_LENGTH = 50
HOISTING_UP_SPEED = -100
HOISTING_DOWN_SPEED = 100
HOIST_MAX = 60
HOIST_MIN = 30


# Define
#============================================
SERVO_LOCK = True
SERVO_UNLOCK = False

# Controller Variable
PWM_trolley = 0
PWM_hoisting = 0
servo_state = False


# Callback Subscriber ROS 2
def encoder_callback(msg):
    encoder_value = msg.data

def limitSW_L_callback(msg):
    limitSW_L = msg.data

def limitSW_R_callback(msg):
    limitSW_R = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('ros2_controlgantry')

    pwm_trolley_pub = node.create_publisher(UInt16, 'pwm_trolley', 10)
    pwm_hoisting_pub = node.create_publisher(UInt16, 'pwm_hoisting', 10)
    servo_state_pub = node.create_publisher(Bool, 'servo', 10)

    encoder_sub = node.create_subscription(Int32, 'encoder', encoder_callback, 10)
    limitSW_L_sub = node.create_subscription(Bool, 'limitSW_L', limitSW_L_callback, 10)
    limitSW_R_sub = node.create_subscription(Bool, 'limitSW_R', limitSW_R_callback, 10)

    pwm_trolley_pub.publish(UInt16(data=PWM_trolley))
    pwm_hoisting_pub.publish(UInt16(data=PWM_hoisting))
    servo_state_pub.publish(Bool(data=servo_state))

    

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

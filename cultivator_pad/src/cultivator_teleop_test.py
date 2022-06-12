#!/usr/bin/env/ python

import rospy
from std_msgs.msg import Empty, UInt16
from enum import IntEnum

class TestType(IntEnum):
    dc = 1
    led = 2
    servo = 3

class TestTeleop():
    def __init__(self):
        # Publisher
        self.test_servo_angle = 0
        self.test_dc_pub = rospy.Publisher('/test_dc', Empty)
        self.test_led_pub = rospy.Publisher('/test_led', Empty)
        self.test_servo_pub = rospy.Publisher('/test_servo', UInt16, queue_size=1)
    
    def send_test_dc(self):
        self.test_dc_pub.publish()

    def send_test_led(self):
        self.test_led_pub.publish()
    
    def send_test_servo(self):
        for num in range(2):
            for angle in range(0, 181, 1):
                self.test_servo_angle = angle
                self.test_servo_pub.publish(self.test_servo_angle)
            for angle in range(180, -1, -1):
                self.test_servo_angle = angle
                self.test_servo_pub.publish(self.test_servo_angle)

def main():
    rospy.init_node('test_teleop')
    test = TestTeleop()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        test_select = input('1:DC, 2:LED, 3:SERVO : ')
        if (int(test_select) == TestType.dc):
            test.send_test_dc()
        elif (int(test_select) == TestType.led):
            test.send_test_led()
        elif (int(test_select) == TestType.servo):
            test.send_test_servo()
        else:
            print('Please Reselect.')
        rate.sleep()

if __name__=="__main__":
    main()

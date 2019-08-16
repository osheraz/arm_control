#!/usr/bin/env python

import numpy as np
import pprint
import rospy
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Int32MultiArray

# Global Variables
pp = pprint.PrettyPrinter(indent=4)
model_name = 'komodo2'
node_name = 'arm_controller'
motor_con = 4
limit = 1000
crit = 0
pwm_to_pub = Int32MultiArray()


def main():
    ArmController()
    rospy.spin()

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))


class ArmController:

    rospy.init_node(node_name)
    Hz = 50
    rate = rospy.Rate(Hz)
    kp_ac=10
    ki_ac=.1
    kd_ac =0
    kp_sc=20
    ki_sc=.2
    kd_sc = 0
    des_cmd = np.array([250, 250, 120,120],dtype=np.int32)
    pwm_temp = np.zeros((motor_con,),dtype=np.int32)
    fb = np.zeros((motor_con,),dtype=np.int32)
    error =np.zeros((motor_con,),dtype=np.int32)
    error_sum = np.zeros((motor_con,),dtype=np.int32)

    def __init__(self):

        rospy.Subscriber('/arm/des_cmd',Int32MultiArray, self.update_cmd)
        rospy.Subscriber('/arm/pot_fb',Int32MultiArray, self.update_fb)
        self.motor_pub = rospy.Publisher('/arm/motor_cmd', Int32MultiArray, queue_size=10)

        while not rospy.is_shutdown():


            self.hold_untill_get_fb()
            #rospy.loginfo("des_cmd " + str(self.des_cmd) + "      fb " + str(self.fb))
            self.des_cmd=np.asarray(self.des_cmd)
            self.error = self.des_cmd-self.fb

            ##self.error = np.where(abs(self.error_sum) < crit, 0, self.error)
            self.error = np.where(abs(self.error) < crit, 0, self.error)

            self.error_sum += self.error
            self.error_sum = np.where(abs(self.error_sum) < limit, self.error_sum, np.sign(self.error_sum)*limit)
            #self.error_sum =  np.where(abs(self.error_sum) > crit, self.error_sum, 0)
            rospy.loginfo("error: " + str(self.error) + "      sum: " + str(self.error_sum))

            #if self.error_sum < limit : self.error_sum += self.error_load
            #if self.error_sum > crit : self.error_sum += self.error_load

            self.pwm_temp[:2] = self.kp_ac*self.error[:2] + self.ki_ac*self.error_sum[:2] # first 2 is P16
            self.pwm_temp[2:] = self.kp_sc*self.error[2:] + self.ki_sc*self.error_sum[2:] # next  2 is HD50


            self.pwm_temp = np.clip(self.pwm_temp, -250, 250)


            self.pwm_temp =self.pwm_temp.tolist()

            #rospy.loginfo("value: " + str(self.pwm_temp))
            #rospy.loginfo("type: " + str(type(self.pwm_temp)))
            np.set_printoptions(precision=1)
            rospy.loginfo("feedback : " + str(self.fb) + "    pwm applied : " + str(self.pwm_temp))

            pwm_to_pub.data=self.pwm_temp
            #rospy.loginfo("type: " + str(type(pwm_to_pub)))
            #rospy.loginfo("pwm_to_pub : " + str(pwm_to_pub))

            self.motor_pub.publish(pwm_to_pub)

            self.rate.sleep()



    def update_fb(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        self.fb = data.data


    def update_cmd(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        #self.des_cmd = data.data
        if (data.data[0] == data.data[1] and data.data[2] == data.data[3]):
            self.des_cmd = data.data
        else:
            rospy.loginfo("DONT BREAK MY ARM!")


    def hold_untill_get_fb(self):
        """

        :param data:
        :type data: BobcatControl
        :return:
        :rtype:
        """


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

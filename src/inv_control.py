#!/usr/bin/env python

import numpy as np
import pprint
import rospy
import math
import time
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray, Float32

# Global Variables
pp = pprint.PrettyPrinter(indent=4)
model_name = 'komodo2'
node_name = 'arm_controller'
pwm_to_pub = Int32MultiArray()
velocity_to_pub = Int32MultiArray()
arm_data = Float32MultiArray()
motor_con = 4
limit = 1000
crit = 2
dx = 30
dy = dx
# Motor specs: [mm]
x = np.linspace(270, 339, dx)  # HDA50 Stroke
y = np.linspace(200, 260, dy)  # P16 Stroke
y_ = 197  # P16 Extracted length
x_ = 246  # HDA50 Extracted length
h = 140
L = 550
l4 = 80
l1 = 285
r = 237
l3 = 46  # l5
l2 = 293
lp = 206
l33 = 398
r0 = 94
H = 330 + h
l44 = 669.5
minPWM = 20




def main():
    ArmController()
    rospy.spin()


def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))


class ArmController:

    rospy.init_node(node_name)
    flag = 1
    flag_stop = 1
    Hz = 50
    rate = rospy.Rate(Hz)

    des_cmd = np.array([350, 350, 120, 120], dtype=np.int32)
    current = np.array([0, 0, 0, 0], dtype=np.float)
    pwm_temp = np.zeros((motor_con,), dtype=np.int32)
    fb = np.zeros((motor_con,), dtype=np.int32)
    old_fb = np.zeros((motor_con,), dtype=np.int32)
    error = np.zeros((motor_con,), dtype=np.int32)
    error_sum = np.zeros((motor_con,), dtype=np.int32)

    velocityCurrent = np.zeros((motor_con,), dtype=np.int32)
    velocityTarget = np.zeros((motor_con,), dtype=np.int32)
    velocityError = np.zeros((motor_con,), dtype=np.float)
    velocityError_sum = np.zeros((motor_con,), dtype=np.float)

    raw_force = np.zeros((1,), dtype=np.float)
    old_raw_force = np.zeros((1,), dtype=np.float)
    cal_force = np.zeros((1,), dtype=np.float)

    timer = 0
    check_time_start = 0
    max_time = 3.0
    MAX_ARM = 100
    MAX_BUK = 100

    Xp = np.zeros((20, 20))
    Yp = np.zeros((20, 20))
    delta = np.zeros((20, 20))



    def __init__(self):

        rospy.Subscriber('/arm/angle_height', Int32MultiArray, self.update_cmd_angle_height)
        rospy.Subscriber('/arm/des_cmd', Int32MultiArray, self.update_cmd)
        rospy.Subscriber('/arm/pot_fb', Int32MultiArray, self.update_fb)
        rospy.Subscriber('/arm/current', Float32MultiArray, self.update_current)
        rospy.Subscriber('/arm/force', Float32, self.update_raw_force)

        self.arm_data_pub = rospy.Publisher('/arm/data', Float32MultiArray, queue_size=10)
        self.velocity_pub = rospy.Publisher('/arm/velocity', Int32MultiArray, queue_size=10)
        self.cal_force_pub = rospy.Publisher('/arm/calibrated_force', Float32, queue_size=10)

        self.motor_pub = rospy.Publisher('/arm/motor_cmd', Int32MultiArray, queue_size=10)
        self.Xp, self.Yp, self.delta = self.calc_working_area()

        while not rospy.is_shutdown() and self.flag_stop:

            self.timer = rospy.get_time()
            rospy.logdebug("Time: "+str(self.timer))

            self.update_arm_data()

            self.des_cmd = np.asarray(self.des_cmd)
            self.des_cmd[:2] = np.clip(self.des_cmd[:2], 250, 780)  # 780 is the max without vision block
            self.des_cmd[2:] = np.clip(self.des_cmd[2:], 10, 450)

            self.pwm_temp = self.PID_Position()
            #self.pwm_temp = self.PID_Velocity()

            if self.pwm_temp[0] > 0:
                self.pwm_temp[0] = np.clip(self.pwm_temp[0], -250, 250)
                self.pwm_temp[1] = np.clip(self.pwm_temp[1], -250, 250) # 55
            else: # TODO: fix direction diffrence.
                self.pwm_temp[0] = np.clip(self.pwm_temp[0], -250, 250) # 40
                self.pwm_temp[1] = np.clip(self.pwm_temp[1], -250, 250) # 55

            self.pwm_temp[2:] = np.clip(self.pwm_temp[2:], -255, 255)

            self.pwm_temp = np.where(abs(self.pwm_temp) < minPWM, 0, self.pwm_temp)
            self.pwm_temp = self.pwm_temp.tolist()

            rospy.loginfo("Feedback : " + str(np.round(self.fb, 2)) + "    Pwm Applied : " + str(np.round(self.pwm_temp, 2)))
            rospy.loginfo("Old Feedback : " + str(np.round(self.old_fb,2)))
            rospy.loginfo("Target SC : " + str(self.des_cmd[0]) + "    Target AC : " + str(self.des_cmd[2]))

            self.safety_checks()

            velocity_to_pub.data = self.velocityCurrent.tolist()
            pwm_to_pub.data = self.pwm_temp
            self.motor_pub.publish(pwm_to_pub)
            self.velocity_pub.publish(velocity_to_pub)

            self.rate.sleep()

    def PID_Position(self):

        kp_ac = 20
        ki_ac = .1
        kp_sc = 10
        ki_sc = .2
        pwm_temp = np.zeros((motor_con,), dtype=np.int32)

        self.error = self.des_cmd - self.fb  # Compute Error Position
        self.error = np.where(abs(self.error) < crit, 0, self.error)  # Check if converge
        self.error_sum += self.error  # Sum Position Error
        self.error_sum = np.where(abs(self.error_sum) < limit, self.error_sum, np.sign(self.error_sum) * limit)  # Constrain Error

        pwm_temp[0] = kp_sc * self.error[0] + ki_sc * self.error_sum[0]
        pwm_temp[1] = kp_sc * self.error[1] + ki_sc * self.error_sum[1]
        pwm_temp[2] = kp_ac * self.error[2] + ki_ac * self.error_sum[2]
        pwm_temp[3] = kp_ac * self.error[3] + ki_ac * self.error_sum[3]

        return pwm_temp

    def PID_Velocity(self):

        dt = 1 / 50
        kp_ac = 10
        ki_ac = .1

        kp_sc = 0.5
        ki_sc = 0.02
        kv = 50

        MAX_SPEED = 700  # SensorValue per second

        pwm_temp = np.zeros((motor_con,), dtype=np.int32)

        self.velocityTarget = (self.des_cmd - self.fb) * kv

        self.velocityTarget = np.where(abs(self.velocityTarget) < MAX_SPEED, self.velocityTarget, np.sign(self.velocityTarget) * MAX_SPEED)  # Constrain Error

        self.error = self.des_cmd - self.fb  # Compute Error Position
        self.error = np.where(abs(self.error) < crit, 0, self.error)  # Check if converge

        self.velocityError = self.velocityTarget - self.velocityCurrent  # Sum Position Error

        self.velocityError_sum += self.velocityError * dt
        self.velocityError_sum = np.where(abs(self.velocityError_sum) < limit, self.velocityError_sum, np.sign(self.velocityError_sum) * limit)  # Constrain Error

        pwm_temp[0] = kp_sc * self.velocityError[0] + ki_sc * self.velocityError_sum[0]
        pwm_temp[1] = kp_sc * self.velocityError[1] + ki_sc * self.velocityError_sum[1]
        pwm_temp[2] = kp_ac * self.velocityError[2] + ki_ac * self.velocityError_sum[2]
        pwm_temp[3] = kp_ac * self.velocityError[3] + ki_ac * self.velocityError_sum[3]

        rospy.loginfo("Velocity : " + str(np.round(self.velocityCurrent, 2)) + "    Velocity Target : " + str(
            np.round(self.velocityTarget, 2)))

        return pwm_temp

    def update_fb(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        dt = 0.02
        self.old_fb = np.array(self.fb)
        self.fb = np.array(data.data)
        self.velocityCurrent = (self.fb - self.old_fb) / dt  # SensorValue per second

        # if abs(data.data[0] - data.data[1]) > self.MAX_ARM or abs(data.data[2] - data.data[3]) > self.MAX_BUK:
        #     self.stop()

    def update_cmd(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        if (data.data[0] == data.data[1] and data.data[2] == data.data[3]):
            self.des_cmd = data.data
        else:
            rospy.loginfo("DONT BREAK MY ARM!")

    def update_current(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        self.current = data.data

    def update_raw_force(self, data):
        """

        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        self.old_raw_force = np.array(self.raw_force)
        self.raw_force = data.data
        if abs(self.raw_force - self.old_raw_force) > 5:
            self.raw_force = self.old_raw_force

    def safety_checks(self):

        if (self.current[0] > 2 or self.current[1] > 2) and self.flag:
            self.flag = 0
            self.check_time_start = rospy.get_time()
        if (self.current[2] > 1 or self.current[3] > 1) and self.flag:
            self.flag = 0
            self.check_time_start = rospy.get_time()
        # if self.fb[2] < 30 or self.fb[3] < 30:
        #     self.stop()

        rospy.logdebug("time: " + str(self.timer) + "    check : " + str(self.check_time_start))

        if (self.timer - self.check_time_start) > self.max_time and self.flag == 0: # current limits
            if self.current[0] > 2 or self.current[1] > 2:
                self.stop('SC motors current alert')
            elif self.current[2] > 1 or self.current[3] > 1:
                self.stop('AC motors current alert')
            else:
                self.check_time_start = rospy.get_time()

    def stop(self,msg):
        pwm_to_pub.data = np.zeros((motor_con,), dtype=np.int32)
        self.motor_pub.publish(pwm_to_pub)
        self.flag_stop = 0
        rospy.loginfo('Problem in:  ' + msg)

    def update_arm_data(self):

        xf = (self.fb[0] * 101 / 1023 + x_).astype(float)
        yf = (self.fb[2] * 150 / 1023 + y_).astype(float)
        # Arm mech

        q = np.arcsin((xf ** 2 + h ** 2 - l1 ** 2) / (2 * xf * h))
        # a = np.arcsin(-h + x*np.sin(q)) /(l1)
        b_ = np.arcsin((h * np.sin(np.pi / 2 - q)) / l1) - 12.6 * np.pi / 180
        b = b_ + 36.35 * np.pi / 180

        # Bucket mech

        phi = np.arccos((yf ** 2 - r ** 2 - l3 ** 2) / (-2 * r * l3))
        psi = np.arcsin(l3*np.sin(phi) / yf)
        alpha = phi + psi
        gama = 87.21 * np.pi / 180 - phi
        delta = q - b + gama

        # Bucket tip location

        buc_x = xf * np.cos(q) + l33 * np.cos(q - b)
        tip_x= lp * np.cos(delta)
        buc_z = xf * np.sin(q) + l33 * np.sin(q - b) + H
        tip_z = lp * np.sin(delta)
        Xp = np.add(buc_x, tip_x)
        Zp = np.add(buc_z, tip_z)

        cur_data = np.array([Xp,Zp,buc_x,buc_z])
        arm_data.data = cur_data.tolist()
        self.arm_data_pub.publish(arm_data)

        self.cal_force = self.raw_force * (0.5) * np.sin(alpha) * 1.96
        self.cal_force_pub.publish(self.cal_force)
        # rospy.loginfo(self.cal_force)


    def update_cmd_angle_height(self, data):
        """

        :param data:ang
        :type data:
        :return:       range      0 - 1023
        :rtype: self.des_cmd = data.data
        """
        angle = data.data[0] * np.pi / 180
        height = data.data[1] - H


        PSI = np.arcsin((height) / (l44))
        alpha = np.arccos((l44 ** 2 + l1 ** 2 - l33 ** 2) / (2 * l44 * l1)) + PSI
        PHI = 77.31 * np.pi / 180 - angle - abs(PSI)

        x_c = np.sqrt(h ** 2 + l1 ** 2 + 2 * h * l1 * np.sin(alpha))
        y_c = np.sqrt(r ** 2 + l3 ** 2 - 2 * r * l3 * np.cos(PHI))


        x_mm = (x_c - x_)
        y_mm = (y_c - y_)

        x_cmd = x_mm * (1023 / 101)
        y_cmd = y_mm * (1023 / 150)

        self.des_cmd = np.array([x_cmd, x_cmd, y_cmd, y_cmd]).astype(int)

        #if (x_cmd < 550): # TODO add collosion detection
        #    y_check = self.collision(y_cmd)
        #    if y_cmd > y_check:
        #        y_cmd = y_check

    def calc_working_area(self):
        """

        :param data:
        :type data: BobcatControl
        :return:
        :rtype:
        """
        # Arm mech

        q = np.arcsin((x ** 2 + h ** 2 - l1 ** 2) / (2 * x * h))
        # a = np.arcsin(-h + x*np.sin(q)) /(l1)
        b_ = np.arcsin((h * np.sin(np.pi / 2 - q)) / (l1)) - 12.6 * np.pi / 180
        b = b_ + 36.35 * np.pi / 180

        # Bucket mech

        phi = np.arccos((y ** 2 - r ** 2 - l3 ** 2) / (-2 * r * l3))
        gama = 87.21 * np.pi / 180 - phi
        [QB, G] = np.meshgrid(q - b, gama)
        delta = QB + G

        # Bucket tip location

        p1 = x * np.cos(q) + l33 * np.cos(q - b)
        p2 = lp * np.cos(delta)
        p3 = x * np.sin(q) + l33 * np.sin(q - b) + H
        p4 = lp * np.sin(delta)
        Xp = np.add(p1, p2)
        Yp = np.add(p3, p4)
        return Xp, Yp, delta

    def collision(self, x):
        """
        check if commanded extraction isnt collide with the ground

        :param data: y_cmd
        :type data: int32
        :return:
        :rtype:
        """
        return (1.092 * x - 171)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

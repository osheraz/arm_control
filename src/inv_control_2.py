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
crit = 10
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
rb = 50
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

    des_cmd = np.zeros((motor_con,), dtype=np.int32)
    current = np.array([0, 0, 0, 0], dtype=np.float)
    pwm_temp = np.zeros((motor_con,), dtype=np.int32)
    fb = np.zeros((motor_con,), dtype=np.int32)
    old_fb = np.zeros((motor_con,), dtype=np.int32)
    error = np.zeros((motor_con,), dtype=np.int32)
    error_sum = np.zeros((motor_con,), dtype=np.int32)
    error_derivative = np.zeros((motor_con,), dtype=np.int32)
    sync_errors = np.zeros((motor_con,), dtype=np.int32)

    velocityCurrent = np.zeros((motor_con,), dtype=np.int32)
    velocityTarget = np.zeros((motor_con,), dtype=np.int32)
    velocityError = np.zeros((motor_con,), dtype=np.float)
    velocityError_sum = np.zeros((motor_con,), dtype=np.float)

    measured_force = np.zeros((1,), dtype=np.float)
    raw_force = np.zeros((1,), dtype=np.float)
    old_raw_force = np.zeros((1,), dtype=np.float)
    cal_force = np.zeros((1,), dtype=np.float)
    cal_torque = np.zeros((1,), dtype=np.float)

    timer = 0
    check_time_start = 0
    max_time = 3.0
    MAX_SYNC_ERROR = 5
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
        self.measured_force_pub = rospy.Publisher('/arm/measured_force', Float32, queue_size=10)
        self.cal_torque_pub = rospy.Publisher('/arm/calibrated_torque', Float32, queue_size=10)
        self.motor_pub = rospy.Publisher('/arm/motor_cmd', Int32MultiArray, queue_size=10)
        self.Xp, self.Yp, self.delta = self.calc_working_area()

        rospy.on_shutdown(self.stop)

        while not rospy.is_shutdown() and self.flag_stop:
            self.timer = rospy.get_time()
            rospy.logdebug("Time: " + str(self.timer))

            self.update_arm_data()
            self.compute_position_errors()
            #self.pwm_temp = np.zeros((motor_con,), dtype=np.int32) if np.all(self.error < crit) else self.PID_Position()
            self.pwm_temp = np.zeros((motor_con,), dtype=np.int32) if np.all(self.error < crit) else self.max_pwm_position_control()
            rospy.loginfo("Feedback: " + str(np.round(self.fb, 2)) + " Pwm Applied: "
                          + str(np.round(self.pwm_temp, 2)) + " errors: " + str(np.round(self.error, 2))
                          + " sync_errors: " + str(np.round(self.sync_errors, 2)))

            # For safety reason prevent for sending PWM to the small motor, To remove after testing.
            self.pwm_temp[2:] = 0

            self.safety_checks()
            velocity_to_pub.data = self.velocityCurrent.tolist()
            pwm_to_pub.data = self.pwm_temp.tolist()
            self.motor_pub.publish(pwm_to_pub)
            self.velocity_pub.publish(velocity_to_pub)

            self.rate.sleep()

    def compute_position_errors(self):
        dt = 1 / 50
        new_error = self.des_cmd - self.fb  # Compute Error Position
        self.error_derivative = (new_error - self.error) / dt
        self.error = new_error
        self.error_sum += self.error  # Sum Position Error
        # Constrain Error
        self.error_sum = np.where(abs(self.error_sum) < limit, self.error_sum, np.sign(self.error_sum) * limit)
        self.sync_errors = self.calc_sync_errors()

    def PID_Position(self):
        kp_ac = 20
        ki_ac = .1
        kd_ac = 1
        kp_sc = 10
        ki_sc = .2
        kd_sc = 1
        kp_s_sc = 1

        pwm_temp = np.zeros((motor_con,), dtype=np.int32)
        pwm_temp[:2] = kp_sc * self.error[:2] + ki_sc * self.error_sum[:2] + kd_sc * self.error_derivative[:2]
        pwm_temp[2:] = kp_ac * self.error[2:] + ki_ac * self.error_sum[2:] + kd_ac * self.error_derivative[2:]
        pwm_temp = self.map_range([-1000, 1000], [-255, 255], pwm_temp, True)
        # Add sync error after map and clip in order to always allow to slow down the nearest motor.
        pwm_temp[:2] += kp_s_sc * self.sync_errors[:2]
        return pwm_temp

    def max_pwm_position_control(self):
        kp = 10
        pwm = 255 * np.sign(self.error) + kp * self.sync_errors
        return np.clip(pwm, -255, 255)

    def calc_sync_errors(self):
        sync_errors = np.zeros((motor_con,), dtype=np.int32)
        # Calc sync error for any motor relative to himself
        sync_errors[::2] = self.fb[::2] - self.fb[1::2]
        sync_errors[1::2] = self.fb[1::2] - self.fb[::2]
        # Slow down the nearest actuator (The error sign present the direction of movement)
        return -1 * np.sign(self.error) * np.abs(sync_errors) * self.calc_the_nearest_actuator()

    # Return bool array according to the nearest actuator to the target position.
    def calc_the_nearest_actuator(self):
        nearest_motor = np.zeros((4,), dtype=np.int32)
        nearest_motor[::2] = np.sign(np.abs(self.error[::2]) - np.abs(self.error[1::2]))
        nearest_motor[1::2] = np.sign(np.abs(self.error[1::2]) - np.abs(self.error[::2]))
        return np.sign(nearest_motor) == -1

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

        self.velocityTarget = np.where(abs(self.velocityTarget) < MAX_SPEED, self.velocityTarget,
                                       np.sign(self.velocityTarget) * MAX_SPEED)  # Constrain Error

        self.error = self.des_cmd - self.fb  # Compute Error Position
        self.error = np.where(abs(self.error) < crit, 0, self.error)  # Check if converge

        self.velocityError = self.velocityTarget - self.velocityCurrent  # Sum Position Error

        self.velocityError_sum += self.velocityError * dt
        self.velocityError_sum = np.where(abs(self.velocityError_sum) < limit, self.velocityError_sum,
                                          np.sign(self.velocityError_sum) * limit)  # Constrain Error

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
        if np.all(self.des_cmd == np.asarray(data.data)):  # Test if new command arrived.
            return
        if data.data[0] == data.data[1] and data.data[2] == data.data[3]:
            self.des_cmd = np.asarray(data.data)
            self.des_cmd[:2] = np.clip(self.des_cmd[:2], 300, 780)  # 780 is the max without vision block
            self.des_cmd[2:] = np.clip(self.des_cmd[2:], 10, 450)
            self.error_sum = 0  # Reset error sum when new command arrived.
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
        if not np.all(self.sync_errors < self.MAX_SYNC_ERROR):
            self.stop('Max sync error exceed!')
            return

        if (self.current[0] > 2 or self.current[1] > 2) and self.flag:
            self.flag = 0
            self.check_time_start = rospy.get_time()
        if (self.current[2] > 1 or self.current[3] > 1) and self.flag:
            self.flag = 0
            self.check_time_start = rospy.get_time()
        # if self.fb[2] < 30 or self.fb[3] < 30:
        #     self.stop()

        rospy.logdebug("time: " + str(self.timer) + "    check : " + str(self.check_time_start))

        if (self.timer - self.check_time_start) > self.max_time and self.flag == 0:  # current limits
            if self.current[0] > 3 or self.current[1] > 3:
                self.stop('Problem in: SC motors current alert')
            elif self.current[2] > 1 or self.current[3] > 1:
                self.stop('Problem in: AC motors current alert')
            else:
                self.check_time_start = rospy.get_time()

    def stop(self, msg='Shutdown motors!'):
        pwm_to_pub.data = np.zeros((motor_con,), dtype=np.int32)
        self.motor_pub.publish(pwm_to_pub)
        self.flag_stop = 0
        rospy.loginfo(msg)

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
        psi = np.arcsin(l3 * np.sin(phi) / yf)
        alpha = phi + psi
        gama = 87.21 * np.pi / 180 - phi
        delta = q - b + gama

        # Bucket tip location

        buc_x = xf * np.cos(q) + l33 * np.cos(q - b)
        tip_x = lp * np.cos(delta)
        buc_z = xf * np.sin(q) + l33 * np.sin(q - b) + H
        tip_z = lp * np.sin(delta)
        Xp = np.add(buc_x, tip_x)
        Zp = np.add(buc_z, tip_z)

        cur_data = np.array([Xp, Zp, buc_x, buc_z])
        arm_data.data = cur_data.tolist()
        self.arm_data_pub.publish(arm_data)

        self.measured_force = self.raw_force
        self.cal_force = self.raw_force * np.sin(alpha) * 1.96 * (rb / (r0 * np.cos(delta)))  # on the bucket
        self.cal_torque = self.raw_force * np.sin(alpha) * 1.96 * rb / 1000  # on the arm
        self.measured_force_pub.publish(self.measured_force)
        self.cal_force_pub.publish(self.cal_force)
        self.cal_torque_pub.publish(self.cal_torque)

        rospy.loginfo(
            "Raw Force : " + str(np.round(self.raw_force, 2)) + "     Force : " + str(np.round(self.cal_force, 2)) +
            "     Torque : " + str(np.round(self.cal_torque, 2)) + "    Alpha : " + str(
                np.round(alpha, 2)) + "    Delta: " + str(np.round(delta, 2)))

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

        # if (x_cmd < 550): # TODO add collosion detection
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

    @staticmethod
    def map_range(a, b, s, to_clip=False):
        (a1, a2), (b1, b2) = a, b
        res = b1 + ((s - a1) * (b2 - b1) / (a2 - a1))
        if to_clip:
            return np.clip(res, [b1, b2])
        return res


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# Reuven Amalah.
# make a joint exactly what the command wants it to be- this only works
# for position control.

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return  b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

class CommandToJointState:
    def __init__(self):
        self.joint_name1 = 'arm_joint'
        self.joint_state1 = JointState()
        self.joint_state1.name.append(self.joint_name1)
        self.joint_state1.position.append(0.0)
        self.joint_state1.velocity.append(0.0)

        self.joint_name2 = 'bucket_joint'
        self.joint_state2 = JointState()
        self.joint_state2.name.append(self.joint_name2)
        self.joint_state2.position.append(0.0)
        self.joint_state2.velocity.append(0.0)

        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("/arm/pot_fb", Int32MultiArray,
                                            self.command_callback, queue_size=1)

    def command_callback(self, msg):
        fb = np.array(msg.data)

        self.joint_state1.position[0] = maprange([300, 780], [0.32, -0.1], fb[0])
        self.joint_state2.position[0] = maprange([10, 450], [-0.5,0.9], fb[2])

        self.joint_state1.header.stamp = rospy.Time.now()
        self.joint_state2.header.stamp = rospy.Time.now()

        self.joint_pub.publish(self.joint_state1)
        self.joint_pub.publish(self.joint_state2)

if __name__ == '__main__':
    rospy.init_node('command_to_joint_state')
    command_to_joint_state = CommandToJointState()
    rospy.spin()
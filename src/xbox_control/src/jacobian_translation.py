#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import math
import numpy as np

#In order to run this script, joy_node needs to be running as well as new_rviz.launch

l_1 = 35
l_2 = 50
l_3 = 50

q_1 = 0
q_2 = math.pi/4
q_3 = -math.pi/2

a = 0
b = 0


hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'ankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
hello_str.velocity = []
hello_str.effort = []


def callback(data):
    global q_1
    global q_2
    global q_3

    joy_l_vert = data.axes[1]
    #print(joy_l_vert)
    joy_r_vert = data.axes[4]
    #print(joy_r_vert)
    joy_r_hor = data.axes[3]
    #print(joy_r_hor)

    x_des = joy_r_vert
    y_des = joy_r_hor
    z_des = joy_l_vert
    delta_x = np.matrix([[x_des], [y_des], [z_des]])
    #print(delta_x)
    # origin to base_link transform (not shown in dh parameters)
    origin_base_rot = np.matrix([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    # base_link to joint_0 transforms
    base_zero_trans = np.matrix([[1, 0, 0, a],[0, 1, 0, -b],[0, 0, 1, 0], [0, 0, 0, 1]])
    #joint_0 to joint_1 transforms
    #l_1 translation will be -l_1 if it is the right side of the vehicle and l_1 if on the left
    zero_one_rot_z = np.matrix([[math.cos(q_1), -1*math.sin(q_1), 0, 0], [math.sin(q_1), math.cos(q_1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    zero_one_trans = np.matrix([[1, 0, 0, l_1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    zero_one_rot_x = np.matrix([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    #joint_1 to joint_2 transforms
    one_two_rot_z = np.matrix([[math.cos(q_2), -1*math.sin(q_2), 0, 0], [math.sin(q_2), math.cos(q_2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    one_two_trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, l_2], [0, 0, 1, 0], [0, 0, 0, 1]])
    #joint_2 to end effector transforms
    two_ee_rot_z = np.matrix([[math.cos(q_3), -1*math.sin(q_3), 0, 0], [math.sin(q_3), math.cos(q_3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    two_ee_trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, l_3], [0, 0, 1, 0], [0, 0, 0, 1]])
    base_zero = base_zero_trans
    zero_one = zero_one_rot_z * zero_one_trans * zero_one_rot_x
    one_two = one_two_rot_z * one_two_trans
    two_ee = two_ee_rot_z * two_ee_trans
    origin_ee = origin_base_rot* base_zero * zero_one * one_two * two_ee
    base_ee = base_zero * zero_one * one_two * two_ee

    dx_dq_1 = l_3*math.cos(q_1)*math.sin(q_2)*math.sin(q_3) - l_2*math.cos(q_1)*math.cos(q_2) - l_1*math.sin(q_1)
    dx_dq_2 = l_3*(math.cos(q_2)*math.sin(q_1)*math.sin(q_3) + math.cos(q_3)*math.sin(q_1)*math.sin(q_2)) + l_2*math.sin(q_1)*math.sin(q_2)
    dx_dq_3 = l_3*(math.cos(q_2)*math.sin(q_1)*math.sin(q_3) + math.cos(q_3)*math.sin(q_1)*math.sin(q_2))
    dy_dq_1 = l_3*(math.sin(q_1)*math.sin(q_2)*math.sin(q_3) - math.cos(q_2)*math.cos(q_3)*math.sin(q_1)) + l_1*math.cos(q_1) - l_2*math.cos(q_2)*math.sin(q_1)
    dy_dq_2 = -1*l_3*(math.cos(q_1)*math.cos(q_2)*math.sin(q_3) + math.cos(q_1)*math.cos(q_3)*math.sin(q_2)) - l_2*math.cos(q_1)*math.sin(q_2)
    dy_dq_3 = -1*l_3*(math.cos(q_1)*math.cos(q_2)*math.sin(q_3) + math.cos(q_1)*math.cos(q_3)*math.sin(q_2))
    dz_dq_1 = 0
    dz_dq_2 = l_3*(math.cos(q_2)*math.cos(q_3) - math.sin(q_2)*math.sin(q_3)) + l_2*math.cos(q_2)
    dz_dq_3 = l_3*(math.cos(q_2)*math.cos(q_3) - math.sin(q_2)*math.sin(q_3))
    jacobian = np.matrix([[dx_dq_1, dx_dq_2, dx_dq_3], [dy_dq_1, dy_dq_2, dy_dq_3], [dz_dq_1, dz_dq_2, dz_dq_3]])
    #print(jacobian)
    jacobian_det = ((dx_dq_1)*((dy_dq_2 * dz_dq_3)-(dy_dq_3*dz_dq_2))) - ((dx_dq_2)*((dy_dq_1*dz_dq_3)-(dy_dq_3*dz_dq_1))) + ((dx_dq_3)*((dy_dq_1*dz_dq_2)-(dy_dq_2*dz_dq_1)))
    #print(jacobian_det)
    jacobian_inv = np.linalg.inv(jacobian)
    delta_q = jacobian_inv*delta_x
    print(delta_q)
    q_1_delta = delta_q[0][0]
    q_2_delta = delta_q[1][0]
    q_3_delta = delta_q[2][0]
    q_1 = q_1 + q_1_delta
    q_2 = q_2 + q_2_delta
    q_3 = q_3 + q_3_delta
    print(q_1)
    #print(jacobian_inv)
    #jacobian_inv

    hello_str.position = [q_1, q_2, q_3, 0, 0, q_1, q_2, q_3, 0, 0, q_1, q_2, q_3, 0, 0, q_1, q_2, q_3, 0, 0]





def talker():

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("joy", Joy, callback)
    hello_str.header.stamp = rospy.Time.now()
    pub.publish(hello_str)
    rate.sleep()

    while not rospy.is_shutdown():

      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

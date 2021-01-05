#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import math
import numpy as np
import bubastis_back_plane as backplane

a_global = 0
b_global = 0
#l_1 is the shoulder
l_1_global = .35
#l_2 is the elbow
l_2 = .5
#l_3 is the knee
l_3 = .5
q_1_nom =  0
q_2_nom =  math.pi/4
q_3_nom = -math.pi/2

rf = backplane.quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lf = backplane.quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)
rb = backplane.quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lb = backplane.quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)

hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'ankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
hello_str.velocity = []
hello_str.effort = []


def callback(data):
    joy_l_vert = data.axes[1]
    #print(joy_l_vert)
    joy_r_vert = data.axes[4]
    #print(joy_r_vert)
    joy_r_hor = data.axes[3]
    #print(joy_r_hor)

    x_des = joy_r_vert/100
    y_des = joy_r_hor/100
    z_des = joy_l_vert/100
    rf.get_delta_q(x_des, y_des, z_des)
    lf.get_delta_q(x_des, y_des, z_des)
    rb.get_delta_q(x_des, y_des, z_des)
    lb.get_delta_q(x_des, y_des, z_des)

    print "rf_position:", rf.ee_position()
    print "lf_position:", lf.ee_position()
    print "rb_position:", rb.ee_position()
    print "lb_position:", lb.ee_position()

    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
    time_now = rospy.Time.now()
    print(time_now)

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

#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import math
import numpy as np
import bubastis_back_plane

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
#In order to run this script, joy_node needs to be running as well as new_rviz.launch



rf = quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lf = quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)
rb = quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lb = quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)

hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'fffankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
hello_str.velocity = []
hello_str.effort = []


def callback(data):
    rf.set_nom_position()
    rospy.sleep(.05)
    step_joystick = data.axes[1]
    step_button = data.buttons[0]
    if step_joystick == 1:
        recal_step = step_joystick/4
        vertex_y = 1
        #vertex_x is the x position of the apex of the step, which is exactly half of the b_parabola
        vertex_x =  recal_step/2
        #b_parabola also represents how far you want to step forward
        b_parabola = recal_step
        a_parabola = (vertex_y)/((vertex_x**2)+vertex_x)
        #vertex_y is the height of the step we always want to see
        x_parabola = [0, recal_step/4, recal_step/2, 3*recal_step/4, recal_step]
        y_parabola =[]
        for i in x_parabola:
            footstep = -a_parabola * (i**2) + (b_parabola* a_parabola * i)
            y_parabola.append(footstep)
            delta_y_parabola = [y_parabola[i+1]-y_parabola[i] for i in range(len(y_parabola)-1)]
            print "delta_y_parabola", delta_y_parabola
            delta_x_parabola = [x_parabola[(i/1)+1]-x_parabola[i/1] for i in range(len(x_parabola)-1)]
            print "delta_x_parabola", delta_x_parabola

        rf.get_delta_q(-delta_x_parabola[0], 0, delta_y_parabola[0])
        rospy.sleep(.5)
        hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
        print "step 1 complete"
        rf.get_delta_q(-delta_x_parabola[1], 0, delta_y_parabola[1])
        rospy.sleep(.5)
        hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
        print "step 2 complete"
        rf.get_delta_q(-delta_x_parabola[2], 0, delta_y_parabola[2])
        rospy.sleep(.5)
        hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
        print "step 3 complete"
        rf.get_delta_q(-delta_x_parabola[3], 0, delta_y_parabola[3])
        rospy.sleep(.5)
        hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0] 

    time_now = rospy.Time.now()
    #print(time_now)


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

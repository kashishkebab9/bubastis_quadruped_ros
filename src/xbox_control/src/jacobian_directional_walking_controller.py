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

#In order to run this script, joy_node needs to be running as well as new_rviz.launch

rf = backplane.quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lf = backplane.quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)
rb = backplane.quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lb = backplane.quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)

hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'fffankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
hello_str.velocity = []
hello_str.effort = []


def callback(data):
    
    rospy.sleep(.05)
    step_trigger = data.axes[5]
    step_button = data.buttons[0]
    #left joystick horizontal is [0] and vertical is [1]
    step_joystick_hor = data.axes[0]
    step_joystick_vert = data.axes[1]
    #have to make the horizontal joystick negative to match a standard cartesian graph
    step_joystick_hor = step_joystick_hor * 1
    joystick_magnitude = math.sqrt((step_joystick_hor**2) + (step_joystick_vert**2))
    
    if (step_trigger == -1) and (joystick_magnitude != 0):

        delta_x_ratio = step_joystick_vert / joystick_magnitude
        delta_y_ratio = step_joystick_hor / joystick_magnitude
        print(delta_y_ratio)
        recal_step = joystick_magnitude/4
        
        #vertex_y is the height of the step we always want to see
        vertex_z = 1
        #vertex_x is the x position of the apex of the step, which is exactly half of the b_parabola
        vertex_x =  recal_step/2
        #b_parabola also represents how far you want to step forward
        b_parabola = recal_step
        a_parabola = (vertex_z)/((vertex_x**2)+vertex_x)
        land_parabola = [0, recal_step/4, recal_step/2, 3*recal_step/4, recal_step]
        z_parabola = [-a_parabola * (elem**2) + (b_parabola * a_parabola * elem) for elem in land_parabola]
        delta_land_parabola = [land_parabola[1] - land_parabola[0], land_parabola[2] - land_parabola[1], land_parabola[3] - land_parabola[2], land_parabola[4] - land_parabola[3]]
        delta_parabola_z = [z_parabola[1] - z_parabola[0], z_parabola[2] - z_parabola[1], z_parabola[3] - z_parabola[2], z_parabola[4] - z_parabola[3]]
        delta_land_parabola_x = [elem * delta_x_ratio for elem in delta_land_parabola]
        delta_land_parabola_y = [elem * delta_y_ratio for elem in delta_land_parabola]

        feet = [0, 1, 2, 3, 4]
        
        for f in feet:
            if f == 0:
                for i in range(4):
                    rf.get_delta_q(-delta_land_parabola_x[i], -delta_land_parabola_y[i], delta_parabola_z[i])
                    rospy.sleep(.1)
                    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]

            elif f == 1:            
                for i in range(4):
                    rb.get_delta_q(-delta_land_parabola_x[i], -delta_land_parabola_y[i], delta_parabola_z[i])
                    rospy.sleep(.1)
                    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
        
            elif f == 2:        
                for i in range(4):    
                    lb.get_delta_q(-delta_land_parabola_x[i], -delta_land_parabola_y[i], delta_parabola_z[i])
                    rospy.sleep(.1)
                    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
        
            elif f == 3:        
                for i in range(4):
                    lf.get_delta_q(-delta_land_parabola_x[i], -delta_land_parabola_y[i], delta_parabola_z[i])
                    rospy.sleep(.1)
                    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
                    
            elif f == 4:
                rospy.sleep(1)
                rf.set_joint_angle(q_1_nom, q_2_nom, q_3_nom)
                rb.set_joint_angle(q_1_nom, q_2_nom, q_3_nom)
                lf.set_joint_angle(q_1_nom, q_2_nom, q_3_nom)
                lb.set_joint_angle(q_1_nom, q_2_nom, q_3_nom)
                hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]
                rospy.sleep(.5)
                print("hello")

    time_now = rospy.Time.now()



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
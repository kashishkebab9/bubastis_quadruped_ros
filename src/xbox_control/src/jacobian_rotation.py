#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import math
import numpy as np


a_global = .05
b_global = .035
#l_1 is the shoulder
l_1_global = .035
#l_2 is the elbow
l_2 = .05
#l_3 is the knee
l_3 = .05


q_1_nom =  0
q_2_nom =  math.pi/4
q_3_nom = -math.pi/2

rf_pitch_delta = ([0, 0, 0])
#In order to run this script, joy_node needs to be running as well as new_rviz.launch

def pitch_function(pitch_angle):

    global a_global
    global b_global
    global rf_pitch_delta
    global lf_pitch_delta
    global rb_pitch_delta
    global lb_pitch_delta
    pitch_a = math.cos(pitch_angle) * (a_global)
    pitch_delta_z = (a_global)*(math.sin(pitch_angle))
    if pitch_angle < 0:
        pitch_delta_x = -1*(a_global - pitch_a)
    else:
        pitch_delta_x = (a_global - pitch_a)

    rf_pitch_delta = np.matrix([[-pitch_delta_x], [0], [pitch_delta_z]])
    lf_pitch_delta = np.matrix([[-pitch_delta_x], [0], [pitch_delta_z]])
    lb_pitch_delta = np.matrix([[pitch_delta_x], [0], [-pitch_delta_z]])
    rb_pitch_delta = np.matrix([[pitch_delta_x], [0], [-pitch_delta_z]])
    #print(rf_pitch_delta)

    #fronts are going to move in the opposite translation as the backs 100% of the time

def roll_function(roll_angle):
    global a_global
    global b_global
    global rf_roll_delta
    global lf_roll_delta
    global rb_roll_delta
    global lb_roll_delta

    roll_delta_z = b_global *math.sin(roll_angle)
    roll_a = b_global * math.cos(roll_angle)
    if roll_angle < 0:
        roll_delta_y = -1*(b_global-roll_a)
    else:
        roll_delta_y = b_global-roll_a
    rf_roll_delta = np.matrix([0, -roll_delta_y, roll_delta_z])
    lf_roll_delta = np.matrix([0, roll_delta_y, -roll_delta_z])
    rb_roll_delta = np.matrix([0, -roll_delta_y, roll_delta_z])
    lb_roll_delta = np.matrix([0, roll_delta_y, -roll_delta_z])

def yaw_function(yaw_angle):
    global a_global
    global b_global
    global rf_yaw_delta
    global lf_yaw_delta
    global rb_yaw_delta
    global lb_yaw_delta

    yaw_delta_x = b_global *math.tan(yaw_angle)
    yaw_delta_y = a_global * math.sin(yaw_angle)
    # yaw_a = a_global*math.tan(yaw_angle)
    # yaw_delta_y = math.cos(yaw_angle) * yaw_a
    # if yaw_angle < 0:
    #     yaw_delta_x = (math.sin(yaw_angle) * yaw_a)
    # else:
    #     yaw_delta_x = -1*(math.sin(yaw_angle) * yaw_a)

    rf_yaw_delta = np.matrix([-yaw_delta_x, -yaw_delta_y, 0])
    lf_yaw_delta = np.matrix([yaw_delta_x, -yaw_delta_y, 0])
    rb_yaw_delta = np.matrix([-yaw_delta_x, yaw_delta_y, 0])
    lb_yaw_delta = np.matrix([yaw_delta_x, yaw_delta_y, 0])

class quad_leg:

    def __init__(self, a_offset, b_offset, q_1, q_2, q_3, l_1):
        #constructor to define certain leg variables
        self.a_offset = a_offset
        self.b_offset = b_offset
        self.q_1 = q_1
        self.q_2 = q_2
        self.q_3 = q_3
        self.l_1 = l_1
    def set_joint_angle(self, theta_1, theta_2, theta_3):
        #set shoulder, elbow, knee angles in radians
        self.q_1 = theta_1
        self.q_2 = theta_2
        self.q_3 = theta_3
        print(self.q_1)
        print(self.q_2)
        print(self.q_3)

    def set_nom_position(self):
        #set angles to q_1_nom, q_2_nom, q_3_nom
        self.q_1 = q_1_nom
        self.q_2 = q_2_nom
        self.q_2 = q_2_nom

    def ee_position(self):
        #algorithm to determine the current x, y, z position of the foot

        global l_2
        global l_3

        #origin_base_rot = np.matrix([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        # base_link to joint_0 transforms
        base_zero_trans = np.matrix([[1, 0, 0, self.a_offset],[0, 1, 0, self.b_offset],[0, 0, 1, 0], [0, 0, 0, 1]])
        #joint_0 to joint_1 transforms
        #l_1 translation will be -l_1 if it is the right side of the vehicle and l_1 if on the left
        zero_one_rot_z = np.matrix([[math.cos(self.q_1), -1*math.sin(self.q_1), 0, 0], [math.sin(self.q_1), math.cos(self.q_1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        zero_one_trans = np.matrix([[1, 0, 0, self.l_1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        zero_one_rot_x = np.matrix([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
        #joint_1 to joint_2 transforms
        one_two_rot_z = np.matrix([[math.cos(self.q_2), -1*math.sin(self.q_2), 0, 0], [math.sin(self.q_2), math.cos(self.q_2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        one_two_trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, l_2], [0, 0, 1, 0], [0, 0, 0, 1]])
        #joint_2 to end effector transforms
        two_ee_rot_z = np.matrix([[math.cos(self.q_3), -1*math.sin(self.q_3), 0, 0], [math.sin(self.q_3), math.cos(self.q_3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        two_ee_trans = np.matrix([[1, 0, 0, 0], [0, 1, 0, l_3], [0, 0, 1, 0], [0, 0, 0, 1]])
        base_zero = base_zero_trans
        zero_one = zero_one_rot_z * zero_one_trans * zero_one_rot_x
        one_two = one_two_rot_z * one_two_trans
        two_ee = two_ee_rot_z * two_ee_trans
        #origin_ee = origin_base_rot* base_zero * zero_one * one_two * two_ee
        base_ee = base_zero * zero_one * one_two * two_ee
        #print(origin_ee)
        position_matrix = np.matrix([[base_ee.item(0, 3)], [-base_ee.item(1, 3)], [-base_ee.item(2, 3)]])
        return position_matrix

    def get_delta_q(self, z, x, y):
        #OK THIS IS SOME JANKY SHIT, BUUUUUT YOU HAVE TO INPUT (xyz) & FOR REASONS UNKNOWN, THE INDEXING GETS FUCKY
        #X RESULTS IN CHANGING Y
        #Y RESULTS IN CHANGING Z
        #Z RESULTS IN CHANGING
        #NOT JUST THAT, BUT SOME BECOME NEGATIVE?????? HELLLO?
        delta_x = np.matrix([[-x],[-y],[z]])
        dx_dq_1 = l_3*math.cos(self.q_1)*math.sin(self.q_2)*math.sin(self.q_3) - l_2*math.cos(self.q_1)*math.cos(self.q_2) - self.l_1*math.sin(self.q_1)
        dx_dq_2 = l_3*(math.cos(self.q_2)*math.sin(self.q_1)*math.sin(self.q_3) + math.cos(self.q_3)*math.sin(self.q_1)*math.sin(self.q_2)) + l_2*math.sin(self.q_1)*math.sin(self.q_2)
        dx_dq_3 = l_3*(math.cos(self.q_2)*math.sin(self.q_1)*math.sin(self.q_3) + math.cos(self.q_3)*math.sin(self.q_1)*math.sin(self.q_2))
        dy_dq_1 = l_3*(math.sin(self.q_1)*math.sin(self.q_2)*math.sin(self.q_3) - math.cos(self.q_2)*math.cos(self.q_3)*math.sin(self.q_1)) + self.l_1*math.cos(self.q_1) - l_2*math.cos(self.q_2)*math.sin(self.q_1)
        dy_dq_2 = -1*l_3*(math.cos(self.q_1)*math.cos(self.q_2)*math.sin(self.q_3) + math.cos(self.q_1)*math.cos(self.q_3)*math.sin(self.q_2)) - l_2*math.cos(self.q_1)*math.sin(self.q_2)
        dy_dq_3 = -1*l_3*(math.cos(self.q_1)*math.cos(self.q_2)*math.sin(self.q_3) + math.cos(self.q_1)*math.cos(self.q_3)*math.sin(self.q_2))
        dz_dq_1 = 0
        dz_dq_2 = l_3*(math.cos(self.q_2)*math.cos(self.q_3) - math.sin(self.q_2)*math.sin(self.q_3)) + l_2*math.cos(self.q_2)
        dz_dq_3 = l_3*(math.cos(self.q_2)*math.cos(self.q_3) - math.sin(self.q_2)*math.sin(self.q_3))
        jacobian = np.matrix([[dx_dq_1, dx_dq_2, dx_dq_3], [dy_dq_1, dy_dq_2, dy_dq_3], [dz_dq_1, dz_dq_2, dz_dq_3]])
        #print(jacobian)
        jacobian_det = ((dx_dq_1)*((dy_dq_2 * dz_dq_3)-(dy_dq_3*dz_dq_2))) - ((dx_dq_2)*((dy_dq_1*dz_dq_3)-(dy_dq_3*dz_dq_1))) + ((dx_dq_3)*((dy_dq_1*dz_dq_2)-(dy_dq_2*dz_dq_1)))
        #print(jacobian_det)
        jacobian_inv = np.linalg.inv(jacobian)
        delta_q = jacobian_inv * delta_x
        q_1_delta = delta_q[0][0]
        q_2_delta = delta_q[1][0]
        q_3_delta = delta_q[2][0]
        self.q_1 = self.q_1 + q_1_delta
        self.q_2 = self.q_2 + q_2_delta
        self.q_3 = self.q_3 + q_3_delta

rf = quad_leg(a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
lf = quad_leg(a_global, b_global, q_1_nom, q_2_nom, q_3_nom, l_1_global)
rb = quad_leg(-a_global, -b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)
lb = quad_leg(-a_global, b_global, q_1_nom, q_2_nom, q_3_nom, -l_1_global)

hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'ankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
hello_str.velocity = []
hello_str.effort = []


def callback(data):
    global rf_pitch_delta
    #output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    pitch_joystick = data.axes[1]
    #print(joy_l_vert)
    roll_joystick = data.axes[3]
    #print(joy_r_vert)
    yaw_joystick = data.axes[0]
    #print(joy_r_hor)

    pitch_radian = -math.pi/4 + ((math.pi/4 +math.pi/4) / (2)) * (pitch_joystick + 1)
    roll_radian = -math.pi/4 + ((math.pi/4 +math.pi/4) / (2)) * (roll_joystick + 1)
    yaw_radian = -math.pi/4 + ((math.pi/4 +math.pi/4) / (2)) * (yaw_joystick + 1)
    pitch_radian = pitch_radian/100
    roll_radian = roll_radian/100
    yaw_radian = yaw_radian/100

    print 'yaw_rad', yaw_radian
    pitch_function(pitch_radian)
    roll_function(roll_radian)
    yaw_function(yaw_radian)

    rf_pitch_delta_x = rf_pitch_delta.item(0)
    rf_pitch_delta_y = rf_pitch_delta.item(1)
    rf_pitch_delta_z = rf_pitch_delta.item(2)

    lf_pitch_delta_x = lf_pitch_delta.item(0)
    lf_pitch_delta_y = lf_pitch_delta.item(1)
    lf_pitch_delta_z = lf_pitch_delta.item(2)

    rb_pitch_delta_x = rb_pitch_delta.item(0)
    rb_pitch_delta_y = rb_pitch_delta.item(1)
    rb_pitch_delta_z = rb_pitch_delta.item(2)

    lb_pitch_delta_x = lb_pitch_delta.item(0)
    lb_pitch_delta_y = lb_pitch_delta.item(1)
    lb_pitch_delta_z = lb_pitch_delta.item(2)

    rf_roll_delta_x = rf_roll_delta.item(0)
    rf_roll_delta_y = rf_roll_delta.item(1)
    rf_roll_delta_z = rf_roll_delta.item(2)

    lf_roll_delta_x = lf_roll_delta.item(0)
    lf_roll_delta_y = lf_roll_delta.item(1)
    lf_roll_delta_z = lf_roll_delta.item(2)

    rb_roll_delta_x = rb_roll_delta.item(0)
    rb_roll_delta_y = rb_roll_delta.item(1)
    rb_roll_delta_z = rb_roll_delta.item(2)

    lb_roll_delta_x = lb_roll_delta.item(0)
    lb_roll_delta_y = lb_roll_delta.item(1)
    lb_roll_delta_z = lb_roll_delta.item(2)

    rf_yaw_delta_x = rf_yaw_delta.item(0)
    rf_yaw_delta_y = rf_yaw_delta.item(1)
    rf_yaw_delta_z = rf_yaw_delta.item(2)

    lf_yaw_delta_x = lf_yaw_delta.item(0)
    lf_yaw_delta_y = lf_yaw_delta.item(1)
    lf_yaw_delta_z = lf_yaw_delta.item(2)

    rb_yaw_delta_x = rb_yaw_delta.item(0)
    rb_yaw_delta_y = rb_yaw_delta.item(1)
    rb_yaw_delta_z = rb_yaw_delta.item(2)

    lb_yaw_delta_x = lb_yaw_delta.item(0)
    lb_yaw_delta_y = lb_yaw_delta.item(1)
    lb_yaw_delta_z = lb_yaw_delta.item(2)

    print 'pitch:', pitch_radian
    print 'roll:', roll_radian
    print 'yaw:', yaw_radian
    print lb_pitch_delta_x
    print lb_pitch_delta_y
    print lb_pitch_delta_z

    rf.get_delta_q(rf_pitch_delta_x + rf_roll_delta_x + rf_yaw_delta_x, rf_pitch_delta_y + rf_roll_delta_y + rf_yaw_delta_y, rf_pitch_delta_z + rf_roll_delta_z + rf_yaw_delta_z)
    lf.get_delta_q(lf_pitch_delta_x + lf_roll_delta_x + lf_yaw_delta_x, lf_pitch_delta_y + lf_roll_delta_y + lf_yaw_delta_y, lf_pitch_delta_z + lf_roll_delta_z + lf_yaw_delta_z)
    rb.get_delta_q(rb_pitch_delta_x + rb_roll_delta_x + rb_yaw_delta_x, rb_pitch_delta_y + rb_roll_delta_y + rb_yaw_delta_y, rb_pitch_delta_z + rb_roll_delta_z + rb_yaw_delta_z)
    lb.get_delta_q(lb_pitch_delta_x + lb_roll_delta_x + lb_yaw_delta_x, lb_pitch_delta_y + lb_roll_delta_y + lb_yaw_delta_y, lb_pitch_delta_z + lb_roll_delta_z + lb_yaw_delta_z)

    print "rf_position:", rf.ee_position()
    print "lf_position:", lf.ee_position()
    print "rb_position:", rb.ee_position()
    print "lb_position:", lb.ee_position()

    hello_str.position = [lf.q_1, lf.q_2, lf.q_3, 0, 0, rf.q_1, rf.q_2, rf.q_3, 0, 0, lb.q_1, lb.q_2, lb.q_3, 0, 0, rb.q_1, rb.q_2, rb.q_3, 0, 0]


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
import math
import numpy as np 

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
        #origin_ee =  base_zero * zero_one * one_two * two_ee
        base_ee = base_zero * zero_one * one_two * two_ee
        #print(origin_ee)
        position_matrix = np.array([[base_ee.item(0, 3)], [base_ee.item(1, 3)], [base_ee.item(2, 3)]])
        return position_matrix


    def get_delta_q(self, z, x, y):
        
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




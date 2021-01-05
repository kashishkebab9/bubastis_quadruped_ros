
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from bubastis_back_plane import bubastis_back_plane

#We are going to be publishing joint commands from the pi, which will be receiving imu commands 
#we can also save computation on raspberry pi
#and have it publish the imu values
#my comp can subscribe the imu values and publish the joint states

joints_str = JointState()
joints_str.header = Header()
joints_str.name = ['shoulder_joint_lf', 'elbow_joint_lf', 'wrist_joint_lf', 'ankle_joint_lf', 'shoe_joint_lf',
  'shoulder_joint_rf', 'elbow_joint_rf', 'wrist_joint_rf', 'ankle_joint_rf', 'shoe_joint_rf',
  'shoulder_joint_lb', 'elbow_joint_lb', 'wrist_joint_lb', 'ankle_joint_lb', 'shoe_joint_lb',
  'shoulder_joint_rb', 'elbow_joint_rb', 'wrist_joint_rb', 'ankle_joint_rb', 'shoe_joint_rb']
joints_str.velocity = []
joints_str.effort = []


def callback(data):
    print(hello_str.position)

def talker():
    pub = rospy.Publisher('gyro', Imu, queue_size=10)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.init_node('imu')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

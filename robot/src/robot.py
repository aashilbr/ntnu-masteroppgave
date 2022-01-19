#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

def add_model():
    rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    rospy.loginfo('Adding robot model ...')
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    with open('/home/catkin_ws/src/robot/src/robot/robot.sdf') as file:
        robot_xml = file.read()
    
    q = quaternion_from_euler(0, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=0, y=0, z=0), orientation)

    spawn_sdf_model("robot", robot_xml, "", pose, "world")

    rospy.loginfo("Robot model added.")

if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)
    try:
        add_model()
    except rospy.ROSInterruptException:
        pass
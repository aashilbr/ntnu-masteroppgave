#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from tf.transformations import quaternion_from_euler
from time import sleep

def add_model():
    rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    rospy.loginfo('Adding Inspector Robot model (this can take some time - camera model will be downloaded)...')
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    with open('/home/catkin_ws/src/inspector/src/inspector_robot/inspector_robot.sdf') as file:
        inspector_robot_xml = file.read()
    
    q = quaternion_from_euler(0, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=-2, y=-2, z=0), orientation)

    spawn_sdf_model("inspector_robot", inspector_robot_xml, "", pose, "world")

    rospy.loginfo("Inspector Robot model added.")

class Inspector():
    def __init__(self):
        self._poi_poses = None
        self._inspection_poses = None

        rospy.loginfo('Listening for POI poses and inspection poses...')
        rospy.Subscriber("inspector/poi_poses", PoseArray, self._store_poi_poses)
        rospy.Subscriber("inspector/inspection_poses", PoseArray, self._store_inspection_poses)
    
    def _store_poi_poses(self, poses_array: PoseArray):
        self._poi_poses = poses_array.poses
    
    def _store_inspection_poses(self, poses_array: PoseArray):
        self._inspection_poses = poses_array.poses

    def has_poses(self):
        if self._poi_poses != None and self._inspection_poses != None:
            return True
        return False

    def inspect(self):
        pass

if __name__ == '__main__':
    rospy.init_node('inspector', anonymous=True)
    try:
        add_model()
        inspector = Inspector()
        while not inspector.has_poses():
            print(inspector._poi_poses)
            sleep(1)
        print('Inspector has poses')
        inspector.inspect()
        
    except rospy.ROSInterruptException:
        pass
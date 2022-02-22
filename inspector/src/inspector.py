#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from gazebo_msgs.msg import ModelState, ModelStates
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
        self._robot_state = None

        rospy.loginfo('Listening for POI poses and inspection poses...')
        rospy.Subscriber("inspector/poi_poses", PoseArray, self._store_poi_poses)
        rospy.Subscriber("inspector/inspection_poses", PoseArray, self._store_inspection_poses)
        rospy.Subscriber("gazebo/model_states", ModelStates, self._store_robot_state)
    
    def _store_poi_poses(self, poses_array: PoseArray):
        self._poi_poses = poses_array.poses
    
    def _store_inspection_poses(self, poses_array: PoseArray):
        self._inspection_poses = poses_array.poses
    
    def _store_robot_state(self, model_states: ModelStates):
        if len(model_states.name) < 1:
            return
        
        robot_state_index = False
        for i in range(0, len(model_states.name)):
            if model_states.name[i] == 'inspector_robot':
                robot_state_index = i
                break
        if not robot_state_index:
            return
        
        new_robot_state = ModelState(
            model_states.name[robot_state_index],
            model_states.pose[robot_state_index],
            model_states.twist[robot_state_index],
            'map'
        )

        if self._robot_state == None:
            print('Storing Inspector Robot state.')
            self._robot_state = new_robot_state

    def has_poses(self):
        if self._poi_poses != None and self._inspection_poses != None:
            return True
        return False
    
    def has_robot_state(self):
        if self._robot_state != None:
            return True
        return False

    def inspect(self):
        print('Inspecting...')
        print(self._inspection_poses)
        print()

        print('Publishing new Inspector Robot state')
        pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        new_robot_state = self._robot_state
        new_robot_state.pose = self._inspection_poses[0]
        print(new_robot_state)

        i = 0
        while i <= 3:
            pub.publish(new_robot_state)
            sleep(1)

if __name__ == '__main__':
    rospy.init_node('inspector', anonymous=True)
    try:
        add_model()
        inspector = Inspector()
        while not inspector.has_poses() or not inspector.has_robot_state():
            sleep(1)
        inspector.inspect()
        
    except rospy.ROSInterruptException:
        pass
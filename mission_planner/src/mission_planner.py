#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler
from walkway import *
from time import sleep

class MissionPlanner:
    def __init__(self, points_of_interest = []):
        self.points_of_interest = points_of_interest
    
    def plan(self):
        self.subscribe_to_model_states()
        
        # Wait 5 seconds to load all model states
        i = 0
        while (i < 5):
            sleep(1)
            i += 1

        inspection_poses = self.find_inspection_poses()
        return inspection_poses
    
    def subscribe_to_model_states(self):
        rospy.Subscriber("gazebo/model_states", ModelStates, self.store_model_states)
    
    def store_model_states(self, model_states):
        self.model_states = model_states

    def find_inspection_poses(self):
        inspection_poses = [None] * len(self.points_of_interest)
        
        for i in range(0, len(points_of_interest)):
            inspection_poses[i] = self.find_inspection_pose(points_of_interest[i])

        return inspection_poses

    def find_inspection_pose(self, point_of_interest):
        inspection_pose = pose_from_position_and_orientation(0, 0, 0, 0, 0, 0)
        return inspection_pose

if __name__ == '__main__':
    rospy.init_node('mission_planner', anonymous=True)
    try:
        #points_of_interest = ["valve"]
        #mission_planner = MissionPlanner(points_of_interest)
        #inspection_poses = mission_planner.plan()
        #print(inspection_poses)

        wp = WalkwayProcessor(huldra_models_path = '/home/catkin_ws/src/mission_planner/src/huldra-models/')
        walkway_line = wp.find_walkway_line()
        print('Walkway line')
        print(walkway_line)
        publish_markers(walkway_line)

        while True:
            sleep(1)

    except rospy.ROSInterruptException:
        pass
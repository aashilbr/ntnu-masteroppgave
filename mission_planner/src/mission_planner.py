#!/usr/bin/env python3
import rospy
from utils import *
from walkway import *
from time import sleep

class MissionPlanner:
    def __init__(self, points_of_interest = []):
        self.points_of_interest = points_of_interest

        self.wp = WalkwayProcessor(huldra_models_path = '/home/catkin_ws/src/mission_planner/src/huldra-models/')
        self.walkway_line = self.wp.get_walkway_line()
        publish_markers(self.walkway_line)

    def find_inspection_poses(self):
        inspection_poses = [None] * len(self.points_of_interest)
        
        for i in range(0, len(self.points_of_interest)):
            inspection_poses[i] = self.find_inspection_pose(self.points_of_interest[i])

        return inspection_poses

    def find_inspection_pose(self, point_of_interest):

        # TODO: Use walkway points
        # TODO: Check if robot crashes at the given point
        # TODO: Find distance to valve
        # TODO: Find obstacles through raycasting
        # TODO: (Check if we can see the front of the valve using image processing solely on the valve)
        # TODO: Use image processing to find if the inspection is useful
        # TODO: Sort the possible poses by a weighted sum of the checks mentioned above
        # TODO: Return the best pose

        inspection_pose = pose_from_position_and_orientation(0, 0, 0, 0, 0, 0) # Dummy pose
        return inspection_pose

if __name__ == '__main__':
    rospy.init_node('mission_planner', anonymous=True)
    try:
        # TODO: Use Huldra model with multiple points of interest
        points_of_interest = [
            [0, -20, 2.5]
        ]
        mission_planner = MissionPlanner(points_of_interest)
        inspection_poses = mission_planner.find_inspection_poses()
        print('Inspection poses')
        print(inspection_poses)

        while True:
            sleep(1)

    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
from utils import *
from walkway import WalkwayProcessor
from POI import POI
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

    def find_inspection_pose(self, poi: POI):

        possible_inspection_points = self.wp.get_points_along_walkway_with_resolution(0)

        possible_inspection_points_scores = [0] * len(possible_inspection_points)
        for i in range(0, len(possible_inspection_points)):
            possible_inspection_point = possible_inspection_points[i]
            score = 0

            will_robot_crash = self.will_robot_crash_at_position(possible_inspection_point)
            distance_to_poi = self.get_distance_between_points(possible_inspection_point, poi.position)
            are_there_obstacles = self.are_there_obstacles_between(possible_inspection_point, poi.position)
            is_inspection_possible = self.is_inspection_possible_from_point(poi, point)

            # TODO: Use image processing to find if the inspection is useful

            # TODO: Make a better score function
            score = distance_to_poi

            possible_inspection_points_scores[i] = score

        # TODO: Sort the possible poses by a weighted sum of the checks mentioned above
        # TODO: Return the best pose

        inspection_pose = pose_from_position_and_orientation(0, 0, 0, 0, 0, 0) # Dummy pose
        return inspection_pose
    
    def will_robot_crash_at_position(self, position):
        # TODO: Check if robot crashes at the given point
        return False # Dummy return value
    
    def get_distance_between_points(p1, p2):
        # TODO: Return real distance between p1, p2
        return 0 # Dummy distance
    
    def are_there_obstacles_between(p1, p2):
        # TODO: Find obstacles with raycasting
        return False # Dummy return value
    
    def is_inspection_possible_from_point(self, poi: POI, point):
        # TODO: Check if we can inspect the valve, using the discovered valve inspection direction (poi.direction)
        return True # Dummy return value


if __name__ == '__main__':
    rospy.init_node('mission_planner', anonymous=True)
    try:
        # TODO: Use Huldra model with multiple points of interest
        points_of_interest = [
            POI('valve', [0, -20, 2.5])
        ]
        mission_planner = MissionPlanner(points_of_interest)
        inspection_poses = mission_planner.find_inspection_poses()
        print('Inspection poses')
        print(inspection_poses)

        while True:
            sleep(1)

    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
from utils import *
from walkway import Walkway
from POI import POI
from time import sleep
import constants

class MissionPlanner:
    def __init__(self, points_of_interest = []):
        self.points_of_interest = points_of_interest

        self.wp = Walkway(
            models_path = '/home/catkin_ws/src/mission_planner/src/huldra-models/',
            walkway_path = 'huldra-smaller-walkway/meshes/',
            walkway_filename = 'huldra-smaller-walkway.obj'
        )

        self.wp.walkway_line = constants.smaller_area_walkway_line # Use sorted walkway line constant
        self.walkway_line = self.wp.get_walkway_line()
        publish_markers(self.walkway_line)
        publish_line_marker(self.walkway_line)

    def find_inspection_poses(self):
        inspection_poses = [None] * len(self.points_of_interest)
        
        for i in range(0, len(self.points_of_interest)):
            inspection_poses[i] = self.find_inspection_pose(self.points_of_interest[i])

        return inspection_poses

    def find_inspection_pose(self, poi: POI):

        # TODO: Redo all of this with higher resolution, if we find any interesting parts of the walkway

        resolution = 1
        possible_inspection_points = self.wp.get_points_along_walkway_with_resolution(resolution)
        publish_markers(possible_inspection_points, r=0.1, g=0.5, b=0.1)
        
        print('Possible inspection points (resolution', resolution, '):')
        for point in possible_inspection_points:
            print(point)
        print()

        possible_inspection_points_scores = [0] * len(possible_inspection_points)
        for i in range(0, len(possible_inspection_points)):
            possible_inspection_point = possible_inspection_points[i]
            score = 0

            distance_to_poi = get_distance_between_points(possible_inspection_point, poi.position)
            will_inspector_crash = self.will_inspector_crash_at_position(possible_inspection_point)
            are_there_obstacles = self.are_there_obstacles_between(possible_inspection_point, poi.position)
            is_poi_face_against_inspection_point = self.is_poi_face_against_inspection_point(poi, possible_inspection_point)
            score_from_image_analysis = self.get_score_from_image_analysis(possible_inspection_point, poi.position)

            score = distance_to_poi # TODO: Find a better score function
            possible_inspection_points_scores[i] = score

        possible_inspection_points_sorted = [x for _, x in sorted(zip(possible_inspection_points_scores, possible_inspection_points), key=lambda pair: pair[0])]
        inspection_pose = possible_inspection_points_sorted[0]
        return inspection_pose
    
    def will_inspector_crash_at_position(self, position):
        # TODO: Check if robot crashes at the given point. E.g. are there anything placed on the walkway?
        return False # Dummy return value
    
    def are_there_obstacles_between(self, p1, p2):
        # TODO: Find obstacles with raycasting
        return False # Dummy return value
    
    def is_poi_face_against_inspection_point(self, poi: POI, point):
        # TODO: Check if we can inspect the valve, using the discovered valve inspection direction (poi.direction). Need to implement poi.direction first
        return True # Dummy return value

    def get_score_from_image_analysis(self, inspection_point, poi_position):
        # Use camera plugin to take an image of POI from inspection position, apply image analysis and get a score.
        return 0 # Dummy return value

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
        print()

        # TODO: Send the proposed inspection poses to an Inspector node

        while True:
            sleep(1)

    except rospy.ROSInterruptException:
        pass
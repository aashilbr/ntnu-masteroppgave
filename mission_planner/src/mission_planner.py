#!/usr/bin/env python3
import rospy
from utils import *
from walkway import Walkway
from POI import POI
from time import sleep
import constants
import trimesh
import numpy as np

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

        self.mesh = trimesh.load('/home/catkin_ws/src/mission_planner/src/huldra-models/huldra-smaller/meshes/huldra-smaller.obj', force='mesh')

    def find_inspection_poses(self):
        inspection_poses = [None] * len(self.points_of_interest)
        
        for i in range(0, len(self.points_of_interest)):
            inspection_poses[i] = self.find_inspection_pose(self.points_of_interest[i])

        return inspection_poses

    def find_inspection_pose(self, poi: POI):

        # TODO: Redo all of this with higher resolution, if we find any interesting parts of the walkway

        resolution = 0.1
        possible_inspection_points = self.wp.get_points_along_walkway_with_resolution(resolution)
        publish_markers(possible_inspection_points, r=0.1, g=0.5, b=0.1)
        print('Finding possible inspection points with resolution', resolution)

        possible_inspection_points_scores = [0] * len(possible_inspection_points)
        for i in range(0, len(possible_inspection_points)):
            print(i, '/', len(possible_inspection_points))

            possible_inspection_point = possible_inspection_points[i]
            score = 0

            distance_to_poi = get_distance_between_points(possible_inspection_point, poi.point)
            will_inspector_crash = self.will_inspector_crash_at_point(possible_inspection_point)
            obstacles_count = self.get_obstacles_between(possible_inspection_point, poi.point)
            is_poi_face_against_inspection_point = self.is_poi_face_against_inspection_point(poi, possible_inspection_point)
            score_from_image_analysis = self.get_score_from_image_analysis(possible_inspection_point, poi.point)

            # Lower score is better
            score = (100)*obstacles_count + (1)*distance_to_poi # TODO: Find a better score function
            possible_inspection_points_scores[i] = score
        
        print('Inspection scores for point with identifier', poi.identifier)
        print(possible_inspection_points_scores)
        print('Lowest score:', min(possible_inspection_points_scores))
        print('Highest score:', max(possible_inspection_points_scores))

        # Publish markers of possible inspection points with color grading based on its score
        colors = values_to_colors(possible_inspection_points_scores)
        for i in range(0, len(possible_inspection_points)):
            publish_marker(possible_inspection_points[i], r=colors[i][0], g=colors[i][1], b=colors[i][2], scale=0.2)
            print(possible_inspection_points_scores[i], ':', colors[i])

        possible_inspection_points_sorted = [x for _, x in sorted(zip(possible_inspection_points_scores, possible_inspection_points), key=lambda pair: pair[0])]
        inspection_point = possible_inspection_points_sorted[0]
        inspection_orientation = get_orientation_towards_point(inspection_point, poi.point)
        inspection_pose = Pose(inspection_point, inspection_orientation)

        publish_marker(inspection_point, r=0.0, g=1.0, b=0.0, scale=0.3)

        return inspection_pose
    
    def will_inspector_crash_at_point(self, point):
        # TODO: Check if robot crashes at the given point. E.g. are there anything placed on the walkway?
        return False # Dummy return value
    
    def get_obstacles_between(self, p1: Point, p2: Point):
        ray_origin = np.array(gazebo_to_obj_coordinates([p1.x, p1.y, p1.z]))
        ray_origin_point = Point(ray_origin[0], ray_origin[1], ray_origin[2])
        ray_direction = np.array(normalize(gazebo_to_obj_coordinates_only_roll([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])))

        intersections, index_ray, index_tri = self.mesh.ray.intersects_location(ray_origins=[ray_origin], ray_directions=[ray_direction])

        distance_p1_p2 = get_distance_between_points(p1, p2)

        intersections_indices = []
        for i in range(0, len(intersections)):
            intersection_point = Point(x = intersections[i][0], y = intersections[i][1], z = intersections[i][2])
            distance_ray_origin_intersection = get_distance_between_points(ray_origin_point, intersection_point)
            if distance_ray_origin_intersection < distance_p1_p2:
                intersections_indices.append(i)
        return len(intersections_indices)
    
    def is_poi_face_against_inspection_point(self, poi: POI, point):
        # TODO: Check if we can inspect the valve, using the discovered valve inspection direction (poi.direction). Need to implement poi.direction first
        return True # Dummy return value

    def get_score_from_image_analysis(self, inspection_point, poi_point):
        # Use camera plugin to take an image of POI from inspection position, apply image analysis and get a score.
        return 0 # Dummy return value

if __name__ == '__main__':
    rospy.init_node('mission_planner', anonymous=True)
    trimesh.util.attach_to_log()
    try:
        coords0 = obj_to_gazebo_coordinates([-117.014, 30.016, 304.500])
        coords1 = obj_to_gazebo_coordinates([-115.739, 31.149, 304.950])
        coords2 = obj_to_gazebo_coordinates([-114.401, 30.099, 307.900])
        coords3 = obj_to_gazebo_coordinates([-112.550, 30.238, 310.292])

        points_of_interest = [
            #POI('valve1', Point(0, -20, 2), Quaternion(0, 0, 0, 1)),
            #POI('valve2', Point(5, -23, 5), Quaternion(0, 0, 0, 1)),
            #POI('valve3', Point(0, -26, 10), Quaternion(0, 0, 0, 1)),
            #POI('valve3', Point(-2, -24, 5), Quaternion(0, 0, 0, 1))

            POI('20-2000VF', Point(coords0[0], coords0[1], coords0[2]), Quaternion(0, 0, 0, 1)), # "x": 304500, "y": 117014, "z": 30016
            #POI('20-2007VF', Point(coords1[0], coords1[1], coords1[2]), Quaternion(0, 0, 0, 1)), # "x": 304950, "y": 115739, "z": 31149
            #POI('20-2003VF', Point(coords2[0], coords2[1], coords2[2]), Quaternion(0, 0, 0, 1)), # "x": 307900, "y": 114401, "z": 30099
            #POI('20-2006PL', Point(coords3[0], coords3[1], coords3[2]), Quaternion(0, 0, 0, 1))  # "x": 310292, "y": 112550, "z": 30238
        ]

        mission_planner = MissionPlanner(points_of_interest)
        inspection_poses = mission_planner.find_inspection_poses()

        i = 0
        while i <= 3:
            send_to_inspector([point.pose for point in points_of_interest], inspection_poses)
            sleep(1)
            i += 1

    except rospy.ROSInterruptException:
        pass
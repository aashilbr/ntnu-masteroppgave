#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler

def pose_from_position_and_orientation(x, y, z, a, b, c):
    q = quaternion_from_euler(a, b, c)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=x, y=y, z=z), orientation)
    return pose

class MissionPlanner:
    def __init__(self, points_of_interest = []):
        self.points_of_interest = points_of_interest
    
    def plan(self):
        inspection_poses = self.find_inspection_poses()
        return inspection_poses

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
        points_of_interest = ["valve"]
        mission_planner = MissionPlanner(points_of_interest)
        inspection_poses = mission_planner.plan()
        print(inspection_poses)
    except rospy.ROSInterruptException:
        pass
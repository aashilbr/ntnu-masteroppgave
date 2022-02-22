#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from generate_valve import Valve
from math import pi
from tf.transformations import quaternion_from_euler

class ModelSpawner():
    huldra_offset_position = Point(x=116, y=287, z=-27)
    huldra_offset_roll = pi/2
    valve_index = 1

    def __init__(self):
        pass
    
    def add_valve(self, x, y, z, yaw):
        rospy.loginfo('Generating valve ...')

        valve = Valve(
            model_path = "/home/catkin_ws/src/model_spawner/src/models/valve",
            x_offset = 0,
            y_offset = 0,
            z_offset = 1,
            roll_offset = 0,
            pitch_offset = 0,
            yaw_offset = 0,
            pipe_radius = 0.2,
            pipe_length = 1,
            valve_pipe_radius = 0.2,
            valve_pipe_length = 0.5,
            valve_radius = 0.2
        )
        valve.generate_sdf()

        rospy.loginfo('Adding generated valve ...')

        with open('/home/catkin_ws/src/model_spawner/src/models/valve/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(q[0],q[1],q[2],q[3])
        pose = Pose(Point(x=x, y=y, z=z), orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("valve" + str(self.valve_index), model_xml, "", pose, "world")
        self.valve_index += 1

    def add_huldra_small_area(self):
        rospy.loginfo('Adding Huldra-small-area...')

        with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-small-area/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(self.huldra_offset_roll, 0, 0)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(self.huldra_offset_position, orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("huldra-small-area", model_xml, "", pose, "world")

    def add_huldra_small_area_walkway(self):
        rospy.loginfo('Adding Huldra-small-area-walkway...')

        with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-small-area-walkway/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(self.huldra_offset_roll, 0, 0)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(self.huldra_offset_position, orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("huldra-small-area-walkway", model_xml, "", pose, "world")

    def add_huldra_smaller(self):
        rospy.loginfo('Adding Huldra-smaller...')

        with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-smaller/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(self.huldra_offset_roll, 0, 0)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(self.huldra_offset_position, orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("huldra-smaller", model_xml, "", pose, "world")

    def add_huldra_smaller_walkway(self):
        rospy.loginfo('Adding Huldra-smaller-walkway...')

        with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-smaller-walkway/model.sdf') as file:
            model_xml = file.read()
        
        q = quaternion_from_euler(self.huldra_offset_roll, 0, 0)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(self.huldra_offset_position, orientation)

        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model("huldra-smaller-walkway", model_xml, "", pose, "world")

if __name__ == '__main__':
    try:
        rospy.init_node('model_spawner', anonymous=True)
        rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawner = ModelSpawner()
        spawner.add_valve(x=0, y=-20, z=2.5, yaw=-pi/2)
        spawner.add_valve(x=5, y=-23, z=2.5, yaw=-pi/2)
        #spawner.add_floor()
        #spawner.add_huldra_small_area()
        #spawner.add_huldra_small_area_walkway()
        #spawner.add_huldra_smaller()
        spawner.add_huldra_smaller_walkway()
    except rospy.ROSInterruptException:
        pass
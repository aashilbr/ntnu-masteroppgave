#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
from generate_valve import Valve
from math import pi
from tf.transformations import quaternion_from_euler

def add_valve():    
    rospy.loginfo('Generating valve ...')

    model_path = "/home/catkin_ws/src/model_spawner/src/valve"
    x_offset = 0
    y_offset = 0
    z_offset = 1
    roll_offset = 0
    pitch_offset = 0
    yaw_offset = 0
    pipe_radius = 0.2
    pipe_length = 1
    valve_pipe_radius = 0.2
    valve_pipe_length = 0.5
    valve_radius = 0.2

    valve = Valve(
        model_path = model_path,
        x_offset = x_offset,
        y_offset = y_offset,
        z_offset = z_offset,
        roll_offset = roll_offset,
        pitch_offset = pitch_offset,
        yaw_offset = yaw_offset,
        pipe_radius = pipe_radius,
        pipe_length = pipe_length,
        valve_pipe_radius = valve_pipe_radius,
        valve_pipe_length = valve_pipe_length,
        valve_radius = valve_radius
    )
    valve.generate_sdf()

    rospy.loginfo('Adding generated valve ...')

    with open('/home/catkin_ws/src/model_spawner/src/models/valve/model.sdf') as file:
        model_xml = file.read()
    
    orientation = Quaternion(0,0,0,0)
    pose = Pose(Point(x=0, y=0, z=0.5), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("valve", model_xml, "", pose, "world")

def add_floor():
    rospy.loginfo('Adding floor ...')

    with open('/home/catkin_ws/src/model_spawner/src/models/floor/model.sdf') as file:
        model_xml = file.read()
    
    orientation = Quaternion(0,0,0,0)
    pose = Pose(Point(x=0, y=-2, z=0.5), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("floor", model_xml, "", pose, "world")

def add_huldra_small_area():
    rospy.loginfo('Adding Huldra-small-area...')

    with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-small-area/model.sdf') as file:
        model_xml = file.read()
    
    q = quaternion_from_euler(pi/2, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=116, y=287, z=-27), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("huldra-small-area", model_xml, "", pose, "world")

def add_huldra_small_area_walkway():
    rospy.loginfo('Adding Huldra-small-area-walkway...')

    with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-small-area-walkway/model.sdf') as file:
        model_xml = file.read()
    
    q = quaternion_from_euler(pi/2, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=116, y=287, z=-27), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("huldra-small-area-walkway", model_xml, "", pose, "world")

def add_huldra_smaller():
    rospy.loginfo('Adding Huldra-smaller...')

    with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-smaller/model.sdf') as file:
        model_xml = file.read()
    
    q = quaternion_from_euler(pi/2, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=116, y=287, z=-27), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("huldra-smaller", model_xml, "", pose, "world")

def add_huldra_smaller_walkway():
    rospy.loginfo('Adding Huldra-smaller-walkway...')

    with open('/home/catkin_ws/src/model_spawner/src/huldra-models/huldra-smaller-walkway/model.sdf') as file:
        model_xml = file.read()
    
    q = quaternion_from_euler(pi/2, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=116, y=287, z=-27), orientation)

    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model("huldra-smaller-walkway", model_xml, "", pose, "world")

if __name__ == '__main__':
    try:
        rospy.init_node('model_spawner', anonymous=True)
        rospy.loginfo("Waiting for service gazebo/spawn_sdf_model ...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        #add_valve()
        #add_floor()
        #add_huldra_small_area()
        #add_huldra_small_area_walkway()
        #add_huldra_smaller()
        add_huldra_smaller_walkway()
    except rospy.ROSInterruptException:
        pass
from cmath import pi
import rospy
import rostopic
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from tf.transformations import quaternion_from_euler, quaternion_conjugate, quaternion_multiply
from time import sleep
from math import sqrt, cos, atan2, sin, asin
from typing import List
import os

# Default offsets for Huldra models
huldra_offset_x = 116
huldra_offset_y = 287
huldra_offset_z = -27
huldra_offset_yaw = 0
huldra_offset_pitch = 0
huldra_offset_roll = pi/2

# Offsets for Huldra models, from Docker environment variables
if 'HULDRA_OFFSET_X' in os.environ:
    huldra_offset_x = float(os.environ['HULDRA_OFFSET_X'])
if 'HULDRA_OFFSET_Y' in os.environ:
    huldra_offset_y = float(os.environ['HULDRA_OFFSET_Y'])
if 'HULDRA_OFFSET_Z' in os.environ:
    huldra_offset_z = float(os.environ['HULDRA_OFFSET_Z'])
if 'HULDRA_OFFSET_YAW' in os.environ:
    huldra_offset_yaw = float(os.environ['HULDRA_OFFSET_YAW'])
if 'HULDRA_OFFSET_PITCH' in os.environ:
    huldra_offset_pitch = float(os.environ['HULDRA_OFFSET_PITCH'])
if 'HULDRA_OFFSET_ROLL' in os.environ:
    huldra_offset_roll = float(os.environ['HULDRA_OFFSET_ROLL'])

def pose_from_position_and_orientation(
    x, 
    y, 
    z, 
    a = 0, 
    b = 0, 
    c = 0):
    
    q = quaternion_from_euler(a, b, c)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    pose = Pose(Point(x=x, y=y, z=z), orientation)
    return pose

def obj_to_gazebo_coordinates(coordinates):
    a = coordinates[0]
    b = coordinates[1]
    c = coordinates[2]

    # TODO: Use huldra_offset_yaw, _pitch, _roll for this:
    x = a
    y = -c
    z = b

    x = x + huldra_offset_x
    y = y + huldra_offset_y
    z = z + huldra_offset_z
    return [x, y, z]

def obj_to_gazebo_coordinates_only_roll(coordinates):
    a = coordinates[0]
    b = coordinates[1]
    c = coordinates[2]

    # TODO: Use huldra_offset_yaw, _pitch, _roll for this:
    x = a
    y = -c
    z = b
    return [x, y, z]

def gazebo_to_obj_coordinates(coordinates):
    x = coordinates[0]
    y = coordinates[1]
    z = coordinates[2]

    x = x - huldra_offset_x
    y = y - huldra_offset_y
    z = z - huldra_offset_z

    # TODO: Use huldra_offset_yaw, _pitch, _roll for this:
    a = x
    c = -y
    b = z

    return [a, b, c]

def gazebo_to_obj_coordinates_only_roll(coordinates):
    x = coordinates[0]
    y = coordinates[1]
    z = coordinates[2]

    # TODO: Use huldra_offset_yaw, _pitch, _roll for this:
    a = x
    c = -y
    b = z

    return [a, b, c]

previous_marker_id = 0
def make_marker(
    point: Point, 
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2,
    scale = 0.15
    ):

    global previous_marker_id
    this_marker_id = previous_marker_id + 1
    previous_marker_id = this_marker_id

    pose = pose_from_position_and_orientation(point.x, point.y, point.z)

    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.id = this_marker_id
    marker.type = type # 0: Arrow 1: Cube, 2: Sphere, 3: Cylinder
    marker.action = 0 # 0: Add
    marker.pose = pose
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.frame_locked = False
    marker.ns = "markers"
    
    return marker

def publish_marker(
    point,
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2,
    scale = 0.15
    ):

    publish_markers([point], r, g, b, a, type, scale)

have_slept_in_publish_markers = False
def publish_markers(
    points,
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2,
    scale = 0.15
    ):
    
    markers = []
    for i in range(0, len(points)):
        marker = make_marker(points[i], r, g, b, a, type, scale)
        markers.append(marker)
    
    pub = rospy.Publisher('markers', MarkerArray, queue_size=10)        
    marker_array_msg = MarkerArray()
    marker_array_msg.markers = markers

    print('Waiting for /markers topic...')
    markersTopicAvailable = False
    while not markersTopicAvailable:
        _, subs = rostopic.get_topic_list()
        for topic, _, _ in subs:
            if topic == '/markers':
                markersTopicAvailable = True
                break
    
    global have_slept_in_publish_markers
    if not have_slept_in_publish_markers:
        have_slept_in_publish_markers = True
        sleep(3)

    pub.publish(marker_array_msg)

def make_line_marker(
    points: List[Point], 
    r = 0.5, 
    g = 0.1, 
    b = 0.1, 
    a = 1.0,
    type = 4,
    scale = 0.15
    ):

    global previous_marker_id
    this_marker_id = previous_marker_id + 1
    previous_marker_id = this_marker_id

    marker_points = []
    for point in points:
        marker_points.append(Point(point.x, point.y, point.z))

    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.id = this_marker_id
    marker.points = marker_points
    marker.type = type # 0: Arrow 1: Cube, 2: Sphere, 3: Cylinder
    marker.action = 0 # 0: Add
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.frame_locked = False
    marker.ns = "markers"
    
    return marker

def publish_line_marker(
    points,
    r = 0.5, 
    g = 0.1, 
    b = 0.1, 
    a = 1.0,
    type = 4,
    scale = 0.15
    ):

    publish_line_markers([points], r, g, b, a, type, scale)

def publish_line_markers(
    pointss,
    r = 0.5, 
    g = 0.1, 
    b = 0.1, 
    a = 1.0,
    type = 4,
    scale = 0.15
    ):
    
    markers = []
    for i in range(0, len(pointss)):
        marker = make_line_marker(pointss[i], r, g, b, a, type, scale)
        markers.append(marker)
    
    pub = rospy.Publisher('markers', MarkerArray, queue_size=10)        
    marker_array_msg = MarkerArray()
    marker_array_msg.markers = markers

    print('Waiting for /markers topic...')
    _, subs = rostopic.get_topic_list()
    markersTopicAvailable = False
    while not markersTopicAvailable:
        for topic, _, _ in subs:
            if topic == '/markers':
                markersTopicAvailable = True
                break
    
    sleep(3)
    pub.publish(marker_array_msg)
    
def get_distance_between_points(p1: Point, p2: Point):
    distance = sqrt( (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2 )
    return distance

def send_to_inspector(poi_poses: List[Pose], inspection_poses: List[Pose]):
    print('Publishing poses to inspector node...')

    inspector_poi_pub = rospy.Publisher('inspector/poi_poses', PoseArray, queue_size=10)
    inspection_inspection_pub = rospy.Publisher('inspector/inspection_poses', PoseArray, queue_size=10)

    poi_pose_array = PoseArray()
    poi_pose_array.poses = poi_poses
    inspection_pose_array = PoseArray()
    inspection_pose_array.poses = inspection_poses

    inspector_poi_pub.publish(poi_pose_array)
    inspection_inspection_pub.publish(inspection_pose_array)

def get_point_between_at_distance(p1: Point, p2: Point, distance):
    # Return a point p3 between p1 and p2 at given distance from p1
    distance_between_points = get_distance_between_points(p1, p2)
    p3 = Point(0, 0, 0)
    p3.x = p1.x + (distance/distance_between_points) * (p2.x - p1.x)
    p3.y = p1.y + (distance/distance_between_points) * (p2.y - p1.y)
    # TODO: Take z coordinates into account
    p3.z = p1.z
    
    return p3

def get_orientation_towards_point(initial_point: Point, target_point: Point):
    world_up = [0, 0, 1]
    world_front = [1, 0, 0]
    
    vector_initial_point = [initial_point.x, initial_point.y, initial_point.z]
    vector_target_point = [target_point.x, target_point.y, target_point.z]

    look_direction = sub(vector_target_point, vector_initial_point)

    yaw_angle = atan2(look_direction[1], look_direction[0]) - atan2(world_front[1], world_front[0])
    pitch_angle = atan2(sqrt(look_direction[0] * look_direction[0] + look_direction[1] * look_direction[1]), look_direction[2]) - pi/2

    q = quaternion_from_euler(0, pitch_angle, yaw_angle)
    q = normalize(q)

    orientation = Quaternion(q[0], q[1], q[2], q[3])
    return orientation

def magnitude(v):
    return sqrt(sum(v[i]*v[i] for i in range(len(v))))

def normalize(v):
    vmag = magnitude(v)
    return [ v[i]/vmag  for i in range(len(v)) ]

def sub(u, v):
    return [ u[i]-v[i] for i in range(len(u)) ]

def get_angle_between_quaternions(q1: Quaternion, q2: Quaternion):
    _q1 = [q1.x, q1.y, q1.z, q1.w]
    _q2 = [q2.x, q2.y, q2.z, q2.w]
    q = quaternion_multiply(_q1, quaternion_conjugate(_q2))
    vector = [q[0], q[1], q[2]]
    theta = 2*asin(magnitude(vector))
    return theta

def values_to_colors(scores, lower_is_greener = True):
    min_score = min(scores)
    max_score = max(scores)
    max_normalized = max_score - min_score

    colors = [0, 0, 0] * len(scores)
    for i in range(0, len(scores)):
        r = ( (scores[i] - min_score) / max_normalized)
        g = 1 - ( (scores[i] - min_score) / max_normalized)
        b = 0
        
        if not lower_is_greener:
            tmp_r = r
            r = g
            g = tmp_r

        colors[i] = [r, g, b]

    return colors

# Not sure if this thing is finished
def extract_object_from_obj(file, object_name):
    with open(file) as f:
        i = 0
        is_writing_object = False
        new_object_string = ''
        for line in f:
            if line[:2] == 'o ' and ( line[2:2+len(object_name)] == object_name or line[3:3+len(object_name)] == object_name ):
                object_line_number = i
                is_writing_object = True
            if is_writing_object:
                if line[:2] == 'o ' and i > object_line_number:
                    is_writing_object = False
                    break
                else:
                    new_object_string += line
            
            i += 1
        
        return new_object_string
import rospy
import rostopic
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from tf.transformations import quaternion_from_euler
from time import sleep
from math import sqrt, cos, atan2, sin
from typing import List

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

    x = a
    y = -c
    z = b

    x = x + 116
    y = y + 287
    z = z - 27
    return [x, y, z]

def obj_to_gazebo_coordinates_only_roll(coordinates):
    a = coordinates[0]
    b = coordinates[1]
    c = coordinates[2]

    x = a
    y = -c
    z = b
    return [x, y, z]

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
    look_direction = normalize(look_direction)

    # Angle between world_front and look_direction (in xy plane). Rotated around world_up
    yaw_angle = atan2(look_direction[1], look_direction[0]) - atan2(world_front[1], world_front[0])

    # Angle between yaw_direction and look_direction, rotated around side_axis
    look_direction_xz = [
        look_direction[0] * cos(yaw_angle) - look_direction[1] * sin(yaw_angle),
        look_direction[0] * sin(yaw_angle) - look_direction[1] * cos(yaw_angle),
        look_direction[2]
    ]
    yaw_direction_xz = [look_direction_xz[0], look_direction_xz[1], 0]

    pitch_angle = atan2(look_direction_xz[2], look_direction_xz[0]) - atan2(yaw_direction_xz[2], yaw_direction_xz[0])
    if pitch_angle > 0:
        pitch_angle = -pitch_angle

    q = quaternion_from_euler(0, pitch_angle, yaw_angle)
    q = normalize(q)

    orientation = Quaternion(q[0], q[1], q[2], q[3])
    return orientation

def cross(a, b):
    # https://stackoverflow.com/questions/1984799/cross-product-of-two-vectors-in-python
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    return c

def magnitude(v):
    return sqrt(sum(v[i]*v[i] for i in range(len(v))))

def add(u, v):
    return [ u[i]+v[i] for i in range(len(u)) ]

def sub(u, v):
    return [ u[i]-v[i] for i in range(len(u)) ]

def dot(u, v):
    return sum(u[i]*v[i] for i in range(len(u)))

def normalize(v):
    vmag = magnitude(v)
    return [ v[i]/vmag  for i in range(len(v)) ]
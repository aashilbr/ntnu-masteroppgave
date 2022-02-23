import rospy
import rostopic
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Pose, Point, PoseArray
from tf.transformations import quaternion_from_euler
from time import sleep
from math import sqrt
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

def get_orientation_towards_point(p1: Point, p2: Point):
    # Return the orientation that points p1 towards p2
    # https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another/1171995#1171995

    v1 = [p1.x, p1.y, p1.z]
    v2 = [p2.x, p2.y, p2.z]

    xyz = cross_product(v1, v2)
    z = sqrt( (len(v1)**2) * (len(v2)**2) ) + dot_product(v1, v2)

    orientation = Quaternion(xyz[0], xyz[1], xyz[2], z)
    return orientation

def cross_product(a, b):
    # https://stackoverflow.com/questions/1984799/cross-product-of-two-vectors-in-python
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    return c

def dot_product(a, b):
    return sum( [a[i]*b[i] for i in range(len(b))] )
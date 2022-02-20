import rospy
import rostopic
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler
from time import sleep

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
    point, 
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2
    ):

    global previous_marker_id
    this_marker_id = previous_marker_id + 1
    previous_marker_id = this_marker_id

    pose = pose_from_position_and_orientation(point[0], point[1], point[2])

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
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    marker.frame_locked = False
    marker.ns = "markers"
    
    return marker

def publish_marker(
    point,
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2
    ):

    publish_markers([point], r, g, b, type)

def publish_markers(
    points,
    r = 0.1, 
    g = 0.1, 
    b = 0.5, 
    a = 1.0,
    type = 2
    ):
    
    markers = []
    for i in range(0, len(points)):
        marker = make_marker(points[i], r, g, b, a, type)
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
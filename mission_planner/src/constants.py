from geometry_msgs.msg import Point
from POI import POI
from utils import *

smaller_area_walkway_line = [
    Point(3.433365333333356, -24.15834533333333, 2.4600009999999948),
    Point(0.866699666666662, -24.4916786666667, 2.4600009999999948),
    Point(-1.9056573333333375, -24.45834333333329, 2.4600009999999948),
    Point(-2.0086616666666686, -24.108693333333292, 2.4600009999999948),
    Point(-2.4005176666666586, -24.31470733333333, 2.4600009999999948),
    Point(-2.3904343333333173, -23.950347666666687, 2.4600009999999948),
    Point(-2.8267746666666653, -24.025187000000017, 2.4600009999999948),
    Point(-2.7045896666666636, -23.68177266666669, 2.4600009999999948),
    Point(-3.1426999999999907, -23.61811300000005, 2.4600009999999948),
    Point(-2.920374333333328, -23.329264000000023, 2.4600009999999948),
    Point(-3.3173699999999826, -23.133331, 2.4600009999999948),
    Point(-3.016664999999989, -22.92731699999996, 2.4600009999999948),
    Point(-3.366635333333349, -17.716674666666677, 2.4600009999999948),
    Point(-3.033304666666666, -12.608337333333338, 2.4600009999999948)
]

huldra_smaller_points_of_interest = [
    POI('20-2000VF', obj_to_gazebo_point([-117.014, 30.016, 304.500]), orientation_from_euler(0, 0, -pi/2)), # "x": 304500, "y": 117014, "z": 30016
    POI('20-2007VF', obj_to_gazebo_point([-115.739, 31.149, 304.950]), orientation_from_euler(0, 0, -pi)),
    POI('20-2003VF', obj_to_gazebo_point([-114.401, 30.099, 307.900]), orientation_from_euler(0, -pi/4, -pi)), # not successful (angle weighted too little)
    POI('20-2006PL', obj_to_gazebo_point([-112.550, 30.238, 310.292]), orientation_from_euler(0, 0, -pi)),
    POI('20-2001PL', obj_to_gazebo_point([-111.588, 30.156, 310.117]), orientation_from_euler(0, 0, -pi)),
    POI('20-2031PL', obj_to_gazebo_point([-112.772, 29.722, 309.125]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2001WI', obj_to_gazebo_point([-113.725, 29.311, 301.05]), orientation_from_euler(0, 0, -pi/2)), # not successful (3/4 impossible to see, the last one due to mission outer edge of walkway)
    POI('20-2002WI', obj_to_gazebo_point([-113.725, 29.311, 305.1]), orientation_from_euler(0, 0, pi/2)), # not successful
    POI('20-2003WI', obj_to_gazebo_point([-111.975, 29.602, 304.68]), orientation_from_euler(0, -pi/4, pi/2)), # not successful
    POI('20-2004WI', obj_to_gazebo_point([-113.725, 29.581, 302.844]), orientation_from_euler(0, -pi/4, -pi)), # not successful
    POI('20-2005WI', obj_to_gazebo_point([-113.425, 29.445, 305.985]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2006WI', obj_to_gazebo_point([-113.703, 29.456, 306.387]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2001VF', obj_to_gazebo_point([-115.739, 30.729, 304.041]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2008VF', obj_to_gazebo_point([-115.739, 31.296, 305.2]), orientation_from_euler(0, 0, -pi)),
    POI('43-4507VF', obj_to_gazebo_point([-118.245, 31.027, 304.477]), orientation_from_euler(0, 0, -pi))
]
from geometry_msgs.msg import Point
from POI import POI
from utils import *

huldra_smaller_walkway_line = [
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

huldra_small_area_walkway_line = [
    Point(3.16668966666667, 4.666666666666686, 2.4600009999999948),
    Point(0.33335633333332737, 4.333333333333314, 2.4600009999999948),
    Point(-3.133308333333332, 4.866658666666638, 2.4600009999999948),
    Point(-3.766639666666663, 4.433329333333347, 2.4600009999999948),
    Point(-4.966323666666668, 4.214141666666649, 2.4600009999999948),
    Point(-5.532989333333333, 3.1281433333332984, 2.4600009999999948),
    Point(-4.732991333333345, 0.37547833333331937, 2.4600009999999948),
    Point(-5.066324666666674, -1.2911883333333094, 2.4600009999999948),
    Point(-4.736719666666659, -2.918691000000024, 2.4600009999999948),
    Point(-5.070462666666671, -2.9239096666666455, 2.4600009999999948),
    Point(-4.777092333333329, -3.0903729999999996, 2.4600009999999948),
    Point(-5.109542999999988, -3.048146666666696, 2.4600009999999948),
    Point(-4.883194000000003, -3.3022053333333474, 2.4600009999999948),
    Point(-5.186327666666671, -3.1593116666666106, 2.4600009999999948),
    Point(-4.962880333333331, -3.4118553333333352, 2.4600009999999948),
    Point(-5.266013999999998, -3.268961666666655, 2.4600009999999948),
    Point(-5.039664999999999, -3.5230203333333066, 2.4600009999999948),
    Point(-5.372116000000005, -3.480794000000003, 2.4600009999999948),
    Point(-5.078745666666649, -3.6472573333333003, 2.4600009999999948),
    Point(-5.412488666666675, -3.6524760000000356, 2.4600009999999948),
    Point(-5.083669333333333, -3.9331359999999904, 2.4600009999999948),
    Point(-5.417002666666676, -4.256439, 2.4600009999999948),
    Point(-5.082046333333324, -4.590047000000027, 2.4600009999999948),
    Point(-5.39743533333332, -4.70334866666667, 2.4600009999999948),
    Point(-5.057736666666656, -4.722940999999992, 2.4600009999999948),
    Point(-5.32267233333333, -4.928161333333321, 2.4600009999999948),
    Point(-4.997520333333327, -4.838429666666627, 2.4600009999999948),
    Point(-5.229825333333338, -5.078114666666636, 2.4600009999999948),
    Point(-4.714045333333331, -5.600260666666657, 2.4600009999999948),
    Point(-3.9998676666666597, -5.8430380000000355, 2.4600009999999948),
    Point(-3.4535220000000066, -6.401041666666686, 2.4600009999999948),
    Point(-3.6552560000000085, -6.676513666666665, 2.4600009999999948),
    Point(-3.256919999999994, -6.7061970000000315, 2.4600009999999948),
    Point(-3.479245666666671, -6.995046000000002, 2.4600009999999948),
    Point(-3.082250000000002, -7.19097899999997, 2.4600009999999948),
    Point(-3.3829549999999955, -7.396993000000009, 2.4600009999999948)
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
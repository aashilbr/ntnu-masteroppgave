#!/usr/bin/env python3

import rospy

def add_model():
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('model_spawner', anonymous=True)
        add_model()
    except rospy.ROSInterruptException:
        pass
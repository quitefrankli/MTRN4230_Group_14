#!/usr/bin/env python
from kinematics_client import Kinematics
import rospy

def main():
    rospy.init_node('run_once', anonymous=True)
    kinematics = Kinematics()
    kinematics.go_to_idle()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

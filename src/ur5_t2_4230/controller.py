#!/usr/bin/env python

import sys
from exceptions import *

import cv2
import numpy as np
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ur5_t2_4230.srv import *

from object_detection import colour_image_converter


def UI():
    end = 0
    queue = []
    object_dict = objectCoor()
    while not end:
        try:
            obj = raw_input("Object: ")
            if obj.lower() not in object_dict.keys():
                print("Invalid object")
                continue
            num = input("Amount: ")
            if len(object_dict[obj]) < int(num):
                print("Invalid number")
                continue
            end = input("? Yes or No (1/0): ")
            print(str(num) + ' ' + obj)
            for i in range(num):
                queue.append(obj.lower())
        except Exception:
            continue


    Xo = []
    Yo = []
    offset = 0.35
    while len(queue) > 0:
        item = queue.pop()
        (_x, _y) = object_dict[item].pop()
        Xo.append(_x)
        Yo.append(_y)
    return Xo, Yo, offset


def objectCoor():
    # dict { 'object names' : list(tuple(x,y)) }
    rospy.wait_for_service('object_detection')
    try:
        detect = rospy.ServiceProxy('object_detection', object_detection)
        resp = detect()

        result_dict = {}
        for i, obj_name in enumerate(resp.st):
            if obj_name not in result_dict.keys():
                result_dict[obj_name] = [(resp.X[i], resp.Y[i])]
            else:
                result_dict[obj_name].append((resp.X[i], resp.Y[i]))
        return result_dict
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def controller_service_callback(req):
    x, y, z = UI()
    print('x')
    print(x)
    print('y')
    print(y)
    return controllerResponse(x, y, z)


def controller_server():
    rospy.init_node('controller_server')
    rospy.Service('controller', controller, controller_service_callback)
    rospy.Subscriber('ui_exit', String, lambda msg: rospy.signal_shutdown("Exit controller"), queue_size=1)
    print("In controller service")
    rospy.spin()


if __name__ == '__main__':
    controller_server()

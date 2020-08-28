#!/usr/bin/env python

import sys
from exceptions import EOFError, KeyError

import cv2
import numpy as np
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ur5_t2_4230.srv import *

from object_detection import colour_image_converter


def create_options(object_dict, choice):
    colour_dict = {}
    shape_dict = {}
    mix_dict = {}
    for (k, v) in object_dict.items():
        colour, shape = k.split(" ")
        amount = len(v)
        try:
            colour_dict[colour] += amount
        except KeyError:
            colour_dict[colour] = amount
        try:
            shape_dict[shape] += amount
        except KeyError:
            shape_dict[shape] = amount
        mix_dict[k] = amount
    if choice == 1:
        return shape_dict
    elif choice == 2:
        return colour_dict
    else:
        return mix_dict


def UI():
    end = True
    object_dict = get_object_coor_dict()
    Xo = []
    Yo = []
    offset = 0.35
    while end:
        try:
            print("Menu\n\t1. Select by shape\n\t2. Select by colour\n\t3. Select shape and colour")
            choice_1 = int(input("Choice: "))
            if choice_1 not in (1, 2, 3):
                print("Invalid choice")
                continue
            options = create_options(object_dict, choice_1)
            print("Your Options:")
            for i, item in enumerate(options.keys()):
                print("\t{}. {} {}".format(i+1, item, options[item]))
            choice_2 = int(input("Choice: ")) - 1
            # TODO: Invalid choice
            num = input("Amount: ")
            # TODO: Invalid num
            end = bool(input("Pick more Yes or No (1/0)? : "))
            for _ in range(num):
                if choice_1 == 3:
                    _x, _y = object_dict[options.keys()[choice_2]].pop(0)
                    if len(object_dict[options.keys()[choice_2]]) == 0:
                        object_dict.pop(options.keys()[choice_2], None)
                else:
                    for k, _ in object_dict.items():
                        if options.keys()[choice_2] in k:
                            _x, _y = object_dict[k].pop(0)
                            if len(object_dict[k]) == 0:
                                object_dict.pop(k, None)
                            break
                Xo.append(_x)
                Yo.append(_y)
        except EOFError:
            continue

    return Xo, Yo, offset


def get_object_coor_dict():
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
    return controllerResponse(x, y, z)


def controller_server():
    rospy.init_node('controller_server')
    rospy.Service('controller', controller, controller_service_callback)
    rospy.Subscriber('ui_exit', String, lambda msg: rospy.signal_shutdown(
        "Exit controller"), queue_size=1)
    print("In controller service")
    rospy.spin()


if __name__ == '__main__':
    controller_server()

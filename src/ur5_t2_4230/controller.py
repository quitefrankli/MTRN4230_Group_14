#!/usr/bin/env python

from object_detection import colour_image_converter
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ur5_t2_4230.srv import *

# def UI(args):
#   end = 0
#   queue = []
#   while not end:
#     obj = input("Object: ")
#     num = input("Amount: ")
#     end = input("Done? Yes or No (1/0): ")
#     print(str(num) + ' ' + obj)
#     for i in range(num):
#         queue.append(obj)
#   print('total objects: ')
#   rospy.init_node('image_converter_rgb', anonymous=True)
#   while len(queue) not 0:
#     key = queue.pop(0)
#     (x,y,z) = colour_image_converter(key)
#     kinematics(x,y,z)

#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")

#   cv2.destroyAllWindows()

def objectCoor():
  rospy.wait_for_service('object_detection')
  try:
    detect = rospy.ServiceProxy('object_detection', object_detection)
    resp = detect()
    print('in proxy')
    return resp.X, resp.Y
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

def controller_service_callback(req):
  x, y = objectCoor()
  print('in main')
  print(x)
  print(y)
  return controllerResponse(x,y,0.5)

def controller_server():
    rospy.init_node('controller_server')
    s = rospy.Service('controller', controller, controller_service_callback)
    print("In controller service")
    rospy.spin()

if __name__ == '__main__':
  controller_server()

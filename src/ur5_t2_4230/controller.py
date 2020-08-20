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

def UI():
  end = 0
  queue = []
  while not end:
    obj = input("Object: ")
    num = input("Amount: ")
    end = input("Done? Yes or No (1/0): ")
    print(str(num) + ' ' + obj)
    for i in range(num):
        queue.append(obj.lower())

  x,y,st = objectCoor()
  x = list(x)
  y = list(y)
  Xo = []
  Yo = []
  offset = 0.35

  prev = ""
  item_count = 1
  exist = True
  # loop through a list of shapes
  while len(queue) is not 0:
    top = queue.pop(0)
    if prev == top:
      item_count+=1
    # try to match input shapes to detected shapes
    for i in range(len(st)):
      if top == st[i]:
        Xo.append(x.pop(i))
        Yo.append(y.pop(i))
        st.pop(i)
        exist = True
        break
      exist = False
    prev = top

    if exist is False:
      item_count-=1
      print("{} ".format(item_count) + top + " found")
      pass
  
  print('Xo')
  print(Xo)
  print('Yo')
  print(Yo)

  return Xo, Yo, offset

def objectCoor():
  rospy.wait_for_service('object_detection')
  try:
    detect = rospy.ServiceProxy('object_detection', object_detection)
    resp = detect()
    print('in detect proxy')
    return resp.X, resp.Y, resp.st
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

def controller_service_callback(req):
  x, y, z = UI()
  print('x')
  print(x)
  print('y')
  print(y)
  return controllerResponse(x,y,z)

def controller_server():
    rospy.init_node('controller_server')
    s = rospy.Service('controller', controller, controller_service_callback)
    print("In controller service")
    rospy.spin()


if __name__ == '__main__':
  controller_server()


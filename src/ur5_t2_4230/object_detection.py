#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spawn_objects import URDF_Object, get_random_string, spawn_products
from geometry_msgs.msg import Point
import time
from matplotlib import pyplot as plt


def find_objects(original_image):
  spawned_objects = []

  #
  # Preprocessing
  #
  
  # first crop the image
  image = original_image[140:340, 150:490]
  # we may possibly want to subsample the image to increase performance

  # filter out anything that isn't of interest
  mask = cv2.inRange(
    cv2.cvtColor(image, cv2.COLOR_BGR2HSV), 
    np.array([0, 250, 250]), 
    np.array([125, 255, 255])
  )

  #
  # Processing
  #

  # get all contours
  _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(image, contours, -1, (0, 0, 0), 3)

  # process all contours
  for contour in contours:
    perimeter = cv2.arcLength(contour, True)
    approximation = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

    # get center of contour
    center = np.array([0, 0])
    for vertex in approximation:
      center += vertex[0]
    center /= len(approximation) # local x, y

    # discern shape of contour
    if len(approximation) == 4: # cube
      name = 'box'
    else: # cylinder
      name = 'cylinder'

    # gets the color
    color = image[center[1], center[0]]
    if color[0] > 240:
      color = 'blue'
    elif color[1] > 240:
      color = 'green'
    else:
      color = 'red'

    cv2.circle(image, tuple(np.round(center).astype(int)), 5, (0, 0, 0))
    cv2.putText(image, color+' '+name, 
      tuple(np.round(center).astype(int)), 
      cv2.FONT_HERSHEY_PLAIN, 0.5, (0, 255, 255), 1
    )

    #
    # gets global center
    #

    # we know the center of the input container is at x=0,y=0.7
    container_global_center = np.array([0, 0.7]) # global x, y
    # we know the container is approx 0.5m long on the longer side
    container_length = 0.51

    scale = (len(image[0])-40)/container_length # -40 to get rid of border part
    
    container_center = np.array([len(image[0]), len(image)])/2 # local x, y

    offset = (center - container_center)/scale
    global_center = np.array([-offset[1], -offset[0]]) \
      + container_global_center # global x, y
    
    print(color, name, 'at', global_center)

    spawned_objects.append(URDF_Object(
      get_random_string(), 
      color+'_'+name,
      Point(global_center[0], global_center[1], 0.2)
    ))

  plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
  plt.pause(0.01)

  return spawned_objects

class colour_image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("cv_rgb_image",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    self.image_exists = False

  def callback(self,data):
    try:
      self.original_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # note that this is an expensive function, only run when another ros node requests it
    self.image_exists = True

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.original_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter_rgb', anonymous=True)
  
  cic = colour_image_converter()

  original_spawned_products = spawn_products(10)
  time.sleep(2)

  abort = False
  while abort is False:
    if cic.image_exists == False:
      time.sleep(0.1)
      continue
    spawned_objects = find_objects(cic.original_image)
    raw_input('press enter to continue...')
    print('deleting objects')
    for spawned_object in spawned_objects:
      spawned_object.delete()
    ui = raw_input('enter q to quit ')
    abort = (ui == 'q')

  for product in original_spawned_products:
    product.delete()

if __name__ == '__main__':
    main(sys.argv)
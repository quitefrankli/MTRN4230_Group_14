#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class depth_image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cv_depth_image",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback) 

  def callback(self,data):
    try:
      data.encoding = "32FC1"
      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
      cv2.normalize(cv_image,cv_image,0,1,cv2.NORM_MINMAX)
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Depth Image", cv_image)
    cv2.waitKey(0)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))
    except CvBridgeError as e:
      print(e)

def main(args):
  depth_image_converter()
  rospy.init_node('image_converter_depth', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
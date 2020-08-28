#!/usr/bin/env python

from __future__ import print_function

import sys

import cv2
import numpy as np
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ur5_t2_4230.srv import object_detection, object_detectionResponse


def find_objects(original_image):
    #
    # Preprocessing
    #
    global_center_list = np.array([[0, 0]])
    # first crop the image
    # left_offset = 135
    # top_offset = 130
    # image = original_image[top_offset:355, left_offset:505]
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
    _, contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 0, 0), 3)

    # process all contours
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        approximation = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

        # get center of contour
        center = np.array([0, 0])
        for vertex in approximation:
            center += vertex[0]
        center /= len(approximation)  # local x, y

        # discern shape of contour
        if len(approximation) == 4:  # cube
            name = 'cube'
        else:  # cylinder
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
        container_global_center = np.array([[0, 0.7]])  # global x, y
        # we know the container is approx 0.5m long on the longer side
        container_length = 0.5

        scale = len(image[0])/container_length
        container_center = np.array(
            [[len(image[0]), len(image)]])/2  # local x, y

        offset = (center - container_center)/scale
        print(offset)
        #global_center = np.array([[offset[0,1], offset[0,0]]]) \
        global_center = np.array([[-offset[0, 1], offset[0, 0]]]) \
            + container_global_center  # global x, y

        print(color, name, 'at', global_center)
        # append global centers (x,y) onto a list
        global_center_list = np.append(
            global_center_list, global_center, axis=0)

    cv2.imshow('image', image)
    cv2.waitKey(1)
    print(global_center_list)
    return global_center_list


class colour_image_converter:
    def __init__(self):
        #self.image_pub = rospy.Publisher("cv_rgb_image",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            self.original_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # note that this is an expensive function, only run when another ros node requests it
        self.global_center_list = find_objects(self.original_image)

        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(self.original_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


x = np.array([0])
y = np.array([0])


def service_callback(req):

    def callback(data):
        try:
            print('enter try')
            global x
            global y
            original_image = bridge.imgmsg_to_cv2(data, "bgr8")
            coor = find_objects(original_image)
            # print(original)
            print('coor')
            print(coor)
            x = coor[:, 0]
            y = coor[:, 1]
            # print(original_image.shape)
        except CvBridgeError as e:
            print(e)

    bridge = CvBridge()
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.wait_for_message("/camera/color/image_raw", Image)
    print('in service callback')
    return object_detectionResponse(x, y)


def test_server():
    rospy.init_node('object_detection_server')
    s = rospy.Service('object_detection', object_detection, service_callback)
    print("In service")
    rospy.spin()


if __name__ == "__main__":
    test_server()

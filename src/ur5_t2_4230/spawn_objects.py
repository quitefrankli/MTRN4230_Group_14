#!/usr/bin/env python

# Creation Date: 04/07/2020
# Description:
#   Spawns objects

import tf
import rospy
import random
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy
import string
import os


# useful for debugging
def get_random_string(length=15):
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for _ in range(length))

def np_to_point(np_array):
    return Point(np_array[0], np_array[1], np_array[2])

class Static_Object(object):
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    MODEL_PATH = rospy.get_param('MODEL_PATH')
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    all_current_models = []

    def __init__(self, node_name, object_name, point, orient=Quaternion()):
        '''Args: str, str, Point, Point, Quaternion'''
        with open(self.MODEL_PATH + object_name + self.extension) as file:
            xml = file.read()
            self.spawn_model(node_name, xml, '', Pose(point, orient), 'world')
        self.node_name = node_name
        self.all_current_models.append(node_name)

    # def __del__(self):
    #     self.delete()

    def delete(self):
        os.system('rosservice call /gazebo/delete_model ' + self.node_name)
        return

        rospy.wait_for_service("gazebo/delete_model")
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        delete_model(str(self.node_name))

    @classmethod
    def delete_all(cls):
        for model in cls.all_current_models:
            os.system('rosservice call /gazebo/delete_model ' + model)
        cls.all_current_models = []

class URDF_Object(Static_Object):
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    extension = '.urdf'

class SDF_Object(Static_Object):
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    extension = '.sdf'

class Random_Grid(object):
    def __init__(self, cols, rows, origin, offset):
        bottom_left = origin - offset
        cell_size_x = (offset*2.0/float(cols-1))[0]
        cell_size_y = (offset*2.0/float(rows-1))[1]

        self.unused_coordinates = []
        for col in range(cols):
            for row in range(rows):
                self.unused_coordinates.append(Point(
                    (bottom_left + cell_size_x * col)[0],
                    (bottom_left + cell_size_y * row)[1],
                    origin[2]
                ))
        random.shuffle(self.unused_coordinates)

    def __call__(self):
        if not self.unused_coordinates:
            raise Exception("TOO MANY OBJECTS")
        return self.unused_coordinates.pop()

def spawn_products(num_products):
    object_list = ['red_box', 'green_box', 'blue_box',
                   'red_cylinder', 'green_cylinder', 'blue_cylinder']

    products = []

    origin = numpy.array([0, 0.7, 0.1]) # 0.7 is input container y offset
    offset = numpy.array([0.13, 0.25, 0]) # offset in terms of container internal walls

    # actually the other way around swap row
    rand_grid = Random_Grid(5, 7, origin, offset)

    for _ in range(num_products):
        products.append(URDF_Object(
            get_random_string(),
            random.choice(object_list), 
            rand_grid()
        ))

    return products

def main():
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    input_container_y = 0.7
    container_orientation = Quaternion(0, 1, 0, -1)
    input_container_point = Point(0.2, input_container_y, 0.1)
    output_container_point = Point(0.2, -input_container_y, 0.1)

    input_container = SDF_Object('input_container', 'input_container', 
        input_container_point, container_orientation)
    output_container = SDF_Object('output_container', 'output_container', 
        output_container_point, container_orientation)

    # spawn_products()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
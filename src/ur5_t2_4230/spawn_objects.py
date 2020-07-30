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


class Static_Object:
    MODEL_PATH = rospy.get_param('MODEL_PATH')
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    def __init__(self, node_name, object_name, point, orient=Quaternion()):
        '''Args: str, str, Point, Point, Quaternion'''

        with open(self.MODEL_PATH + object_name + self.extension) as file:
            xml = file.read()
            self.spawn_model(node_name, xml, '', Pose(point, orient), 'world')
        self.node_name = node_name

    # def __del__(self):
    #     self.delete_model(self.node_name)

class URDF_Object(Static_Object):
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    extension = '.urdf'

class SDF_Object(Static_Object):
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    extension = '.sdf'

def main():
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    input_container_y = 0.7
    container_orientation = Quaternion(0, 1, 0, -1)
    input_container_point = Point(0.2, input_container_y, 0.1)
    output_container_point = Point(0.2, -0.7, 0.1)

    input_container = SDF_Object('input_container', 'input_container', 
        input_container_point, container_orientation)
    output_container = SDF_Object('output_container', 'output_container', 
        output_container_point, container_orientation)

    object_list = ['red_box', 'green_box', 'blue_box',
                   'red_cylinder', 'green_cylinder', 'blue_cylinder']

    products = []

    origin = numpy.array([0, input_container_y, 0.2])
    offset = numpy.array([0.125, 0.25, 0])

    num_products = 10
    for i in range(num_products):
        products.append(URDF_Object(
            '_ProductNum ' + str(i),
            random.choice(object_list), 
            get_random_coordinate(origin, offset)
        ))


# will add collision avoidance in the future
def get_random_coordinate(origin, offset):
    'origin: Point, offset: Point'
    
    dist = numpy.random.uniform(origin - offset, origin + offset)
    return Point(dist[0], dist[1], dist[2])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# old code for reference
    # spawns products in input container
    # for i in range(10):
    #     if len(used_coordinates) == 0:
    #         x, y = get_randomized_xy()
    #         choice = random.choice(object_list)
    #         URDF_Object("object_%d" % (i), choice, Point(x, y, 0.2))
    #         used_coordinates.append((x, y))
    #         continue
    #     found = False
    #     while not found:
    #         x, y = get_randomized_xy()
    #         for coor in used_coordinates:
    #             if ((x-0.025*2) <= coor[0] <= (x+0.025*2)) and ((y-0.025*2) <= coor[1] <= (y+0.025*2)):
    #                 print("Collide")
    #                 found = False
    #                 break
    #             else:
    #                 found = True
    #     choice = random.choice(object_list)
    #     URDF_Object("object_%d" % (i), choice, Point(x, y, 0.2))
    #     used_coordinates.append((x, y))

# def get_randomized_xy():
#     offset = 0.05
#     y = random.uniform(-0.27305 + offset, 0.27305 - offset)
#     x = random.uniform(-0.0675 + offset - 0.019050, 0.25 - offset - 0.019050)
#     return x, y
#!/usr/bin/env python

# Creation Date: 04/07/2020
# Last Modified: 05/07/2020
# Description:
#   Spawns objects

import tf
import rospy
# import typing
import random
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion


def test():
    print("hi")


class URDF_Object:
    orientation = Quaternion(0, 0, 0, 0)
    item_pose = Pose(Point(x=0, y=0, z=0), orientation)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    def __init__(self, node_name, object_name, point):
        '''Args: str, Point'''

        self.delete_model(node_name)
        MODEL_PATH = rospy.get_param('MODEL_PATH') + 'urdf/'

        with open(MODEL_PATH + object_name + '.urdf') as file:
            xml = file.read()
            item_pose = Pose(point, self.orientation)
            self.spawn_model(node_name, xml, '', item_pose, 'world')


class SDF_Object:
    orientation = Quaternion(0, 0, 0, 0)
    item_pose = Pose(Point(x=0, y=0, z=0), orientation)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    def __init__(self, node_name, object_name, pose):
        '''Args: str, Point'''

        self.delete_model(node_name)
        MODEL_PATH = rospy.get_param('MODEL_PATH') + 'sdf/'

        with open(MODEL_PATH + object_name + '.sdf') as file:
            xml = file.read()
            # item_pose = Pose(point, self.orientation)
            self.spawn_model(node_name, xml, '', pose, 'world')


def get_randomized_xy():
    offset = 0.05
    y = random.uniform(-0.27305 + offset, 0.27305 - offset)
    x = random.uniform(-0.0675 + offset - 0.019050, 0.25 - offset - 0.019050)
    return x, y


def main():
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/delete_model")

    # Quaternion is not being used
    input_container_x = 0.25
    input_container_y = 0
    input_container = SDF_Object('input_container', 'input_container', Pose(
        Point(input_container_x, input_container_y, 0.1), Quaternion(0, 1, 0, -1)))
    output_container = SDF_Object('output_container', 'output_container', Pose(
        Point(0, 2, 0.1), Quaternion(0, 1, 0, -1)))

    object_list = ['red_box', 'green_box', 'blue_box',
                   'red_cylinder', 'green_cylinder', 'blue_cylinder']

    used_coordinates = list()

    for i in range(10):
        if len(used_coordinates) == 0:
            x, y = get_randomized_xy()
            choice = random.choice(object_list)
            URDF_Object("object_%d" % (i), choice, Point(x, y, 0.2))
            used_coordinates.append((x, y))
            continue
        found = False
        while not found:
            x, y = get_randomized_xy()
            for coor in used_coordinates:
                if ((x-0.025*2) <= coor[0] <= (x+0.025*2)) and ((y-0.025*2) <= coor[1] <= (y+0.025*2)):
                    print("Collide")
                    found = False
                    break
                else:
                    found = True
        choice = random.choice(object_list)
        URDF_Object("object_%d" % (i), choice, Point(x, y, 0.2))
        used_coordinates.append((x, y))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

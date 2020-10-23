#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import GetPosition, GetPositionResponse
from robot_pet.msg import Point2d, Point2dOnOff
from sensor_msgs.msg import Image

import rospy

#imports for visualization
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os



class map_node:
    def __init__(self):

        self.map_width = rospy.get_param("/map_width")
        self.map_height = rospy.get_param("/map_height")

        self.icon_size = 32

        self.positions = {
            "user": [rospy.get_param("/user_pos_x"), rospy.get_param("/user_pos_y")],
            "house": [rospy.get_param("/house_pos_x"), rospy.get_param("/house_pos_y")],
            "pet": [rospy.get_param("/pet_pos_x"), rospy.get_param("/pet_pos_y")],
            "pointer": [0, 0]
        }

        #Read images for printing
        os.path.join(os.path.dirname(__file__), 'relative/path/to/file/you/want')
        self.icons = {
            "user"   : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/user.jpg'),0),
            "house"  : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/house.jpg'),0),
            "pet"    : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/pet.jpg'),0),
            "pointer": cv2.imread(os.path.join(os.path.dirname(__file__), 'images/pointer.jpg'),0)
        }

        self.pointer_on = False #Pointer is off in the beginning

        self.bridge = CvBridge()

        #Initialize services
        self.pos_serv = rospy.Service('get_position', GetPosition, self.handle_get_position)
        rospy.loginfo("Position server ready")
        
        #Initialize subscribers 
        self.sub_pet = rospy.Subscriber("pet_position", Point2d, self.callback_pet_position)
        self.sub_user = rospy.Subscriber("user_position", Point2d, self.callback_user_position)
        self.sub_pointer = rospy.Subscriber("pointer_position", Point2dOnOff, self.callback_pointer_position)

        #initialize publishers
        self.pub = rospy.Publisher('map', Image, queue_size=1)

        #Publish initial map
        self.publish_map()

    def handle_get_position(self, req):
        rospy.loginfo("Position of {} requested".format(req.object))
        resp = GetPositionResponse()
        if req.object in self.positions:
            resp.success = True
            resp.point.x = self.positions[req.object][0]
            resp.point.y = self.positions[req.object][1]
            rospy.loginfo("Returning x={} y={}".format(resp.point.x, resp.point.y))
        else:
            rospy.loginfo("Object {} does not exist in dataset! Available objects are: {}".format(req.object, self.positions.keys()))
            resp.success = False
            resp.point.x = 0
            resp.point.y = 0  
        return resp

    def publish_map(self):
        version = 2
        if version == 1:
            map_image = 255*np.ones((self.map_height, self.map_width,3), np.uint8) #White map
            map_image[self.positions["user"][1],  self.positions["user"][0]] = (255,0,0)   #User in Blue
            map_image[self.positions["house"][1], self.positions["house"][0]] = (0,255,0)   #house in Green
            map_image[self.positions["pet"][1],   self.positions["pet"][0]] = (0,0,255)   #Pet in Red
            if self.pointer_on:
                map_image[self.positions["pointer"][1],   self.positions["pointer"][0]] = (255,255,0)   #Pointer in Yellow?
            self.pub.publish((self.bridge.cv2_to_imgmsg(map_image, "bgr8")))

        if version == 2:
            self.map_image = 255*np.ones((self.icon_size*self.map_height, self.icon_size*self.map_width), np.uint8) #White map
            for icon in self.icons:
                self.draw_icon(icon)
            self.pub.publish((self.bridge.cv2_to_imgmsg(self.map_image)))

    def draw_icon(self, icon):
        #If pointer off, do not draw it
        if icon == "pointer" and not self.pointer_on:
            return

        #Coordinates in Grid
        x = self.positions[icon][0]
        y = self.positions[icon][1]

        #Pixel coordinates
        x_start = self.icon_size * x
        y_start = self.icon_size * y
        x_end = x_start + self.icon_size
        y_end = y_start + self.icon_size

        #
        self.map_image[y_start:y_end , x_start:x_end] = np.minimum(self.map_image[y_start:y_end , x_start:x_end], self.icons[icon])



    def callback_pet_position(self, data):
        if not self.positions["pet"] == [data.x, data.y]:
            self.positions["pet"] = [data.x, data.y]
            rospy.loginfo("New pet position: x={} y={}".format(self.positions["pet"][0], self.positions["pet"][1]))
            self.publish_map()

    def callback_user_position(self, data):
        if not self.positions["user"] == [data.x, data.y]:
            self.positions["user"] = [data.x, data.y]
            rospy.loginfo("New user position: x={} y={}".format(self.positions["user"][0], self.positions["user"][1]))
            self.publish_map()

    def callback_pointer_position(self, data):
        self.positions["pointer"] = [data.point.x, data.point.y]
        self.pointer_on = data.on
        if self.pointer_on:
            rospy.loginfo("New Pointer position: x={} y={}".format(self.positions["pointer"][0], self.positions["pointer"][1]))
        else:
            rospy.loginfo("Pointer was switched off")
        self.publish_map()

if __name__ == "__main__":
    rospy.init_node('map')

    #instanciate map_node class
    map_node()

    rospy.spin()
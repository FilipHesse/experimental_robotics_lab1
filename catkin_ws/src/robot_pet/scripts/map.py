#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import GetPosition, GetPositionResponse
from robot_pet.msg import Point2d
import rospy

#imports for visualization
import cv2
import numpy as np



class map_node:
    def __init__(self):

        self.positions = {
            "user": [0, 1],
            "house": [2, 3],
            "pet": [4, 5]
        }

        #Initialize services
        self.pos_serv = rospy.Service('get_position', GetPosition, self.handle_get_position)
        rospy.loginfo("Position server ready")
        
        #Initialize subscribers 
        self.sub_pet = rospy.Subscriber("pet_position", Point2d, self.callback_pet_position)
        self.sub_user = rospy.Subscriber("user_position", Point2d, self.callback_user_position)

        
        

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

    def callback_pet_position(self, data):
        self.positions["pet"] = [data.x, data.y]
        rospy.loginfo("New pet position: x={} y={}".format(self.positions["pet"][0], self.positions["pet"][1]))

    def callback_user_position(self, data):
        self.positions["user"] = [data.x, data.y]
        rospy.loginfo("New user position: x={} y={}".format(self.positions["user"][0], self.positions["user"][1]))


if __name__ == "__main__":
    rospy.init_node('map')

    #instanciate map_node class
    map_node()

    rospy.spin()
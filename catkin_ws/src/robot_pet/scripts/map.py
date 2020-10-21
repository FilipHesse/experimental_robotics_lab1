#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import GetPosition, GetPositionResponse
from robot_pet.msg import Point2d
import rospy

#define global variables
positions = {
    "user": [0, 1],
    "house": [2, 3],
    "pet": [4, 5]
}

def handle_get_position(req):
    rospy.loginfo("Position of {} requested".format(req.object))
    resp = GetPositionResponse()
    if req.object in positions:
        resp.success = True
        resp.point.x = positions[req.object][0]
        resp.point.y = positions[req.object][1]
        rospy.loginfo("Returning x={} y={}".format(resp.point.x, resp.point.y))
    else:
        rospy.loginfo("Object {} does not exist in dataset! Available objects are: {}".format(req.object, positions.keys()))
        resp.success = False
        resp.point.x = 0
        resp.point.y = 0  
    return resp

def callback_pet_position(data):
    global positions
    positions["pet"] = [data.x, data.y]
    rospy.loginfo("New pet position: x={} y={}".format(positions["pet"][0], positions["pet"][1]))

def callback_user_position(data):
    global positions
    positions["user"] = [data.x, data.y]
    rospy.loginfo("New user position: x={} y={}".format(positions["user"][0], positions["user"][1]))


if __name__ == "__main__":
    rospy.init_node('map')

    #Initialize services
    s = rospy.Service('get_position', GetPosition, handle_get_position)
    rospy.loginfo("Position server ready")

    #Initialize subscribers  
    rospy.Subscriber("pet_position", Point2d, callback_pet_position)
    rospy.Subscriber("user_position", Point2d, callback_user_position)

    rospy.spin()
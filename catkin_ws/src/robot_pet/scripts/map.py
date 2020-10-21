#!/usr/bin/env python3

from __future__ import print_function

from robot_pet.srv import GetPosition, GetPositionResponse
import rospy

#define xlobal variables
positions = {
    "person": [0, 1],
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

def pet_command_server():
    s = rospy.Service('get_position', GetPosition, handle_get_position)
    rospy.loginfo("Position server ready")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('map')
    pet_command_server()
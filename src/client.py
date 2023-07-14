#!/usr/bin/env python

import rospy
import actionlib

from mrs_msgs.msg import Reference
from mrs_visitor.msg import VisitAction, VisitGoal
from geometry_msgs.msg import Point


class visitorClientNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("mrs_visitor", VisitAction)
        self.client.wait_for_server()

    def send_request(self):
        goal = VisitGoal(
            reference=Reference(
                position=Point(10, 10, 5),
                heading=0,
            ),
            dt=0.2,
            safety_distance=2.0,
        )

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        rospy.logwarn(f"Result: {result}")


if __name__ == "__main__":
    rospy.init_node("client")
    rospy.logwarn("client starting")

    client = visitorClientNode()
    client.send_request()

#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Bool, Int8

class MyNode(DTROS):

    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.start_ride_sub = rospy.Subscriber('~rotation', Int8, self.run, queue_size=1)
        self.stop_plan_pub = rospy.Publisher('~end_trajectory', Bool, queue_size=1)
        self.start_lf_pub = rospy.Publisher('~end_trajectory_sub', Bool, queue_size=1)

        self.speed = {
            0: {'v': 0.2,
                'omega': 1.6,
                'count': 200},
            1: {'v': 0.5,
                'omega': 0,
                'count': 200},
            2: {'v': 0.2,
                'omega': -2.2,
                'count': 200}
        }


    def run(self, msg: Int8):

        v = self.speed[msg.data]['v']
        omega = self.speed[msg.data]['omega']
        count = self.speed[msg.data]['count']
        i = 0
        for _ in range(5):
            msg = Twist2DStamped()
            msg.v = 0.3
            msg.omega = 0.0
            self.pub.publish(msg)

        while not rospy.is_shutdown():
            msg = Twist2DStamped()
            msg.v = v
            msg.omega = omega
            rospy.loginfo(f"Publishing message [{i}] -- {msg.omega}")
            self.pub.publish(msg)
            if i == count:
                break
            i += 1
            sys.stdout.flush()
        msg = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0
        self.pub.publish(msg)



        self.stop_plan_pub.publish(Bool(data=True))
        self.start_lf_pub.publish(Bool(data=True))

            
    def on_shutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""
        msg = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0
        self.pub.publish(msg)

        super(MyNode, self).on_shutdown()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='circle_drive_node')
    rospy.spin()

#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from std_srvs.srv import Trigger
import time
from std_msgs.msg import Bool

"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()
    node = rclpy.create_node('set_pose')
    publisher_ = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    publisher_set = node.create_publisher(Bool, '/set_initialpose', 10)
    node.declare_parameter('slave_x', 0.0) 
    slave_x= node.get_parameter('slave_x').get_parameter_value().double_value
    node.declare_parameter('slave_y', 0.0) 
    slave_y= node.get_parameter('slave_y').get_parameter_value().double_value
    time.sleep(5.0)

    # Set our demo's initial pose
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.pose.pose.position.x = slave_x
    initial_pose.pose.pose.position.y = slave_y
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    publisher_.publish(initial_pose)
    
    pubmsg = Bool()
    pubmsg.data = True
    publisher_set.publish(pubmsg)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy, LaserScan
from myagv_multi.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg	
class LaserTracker(Node):

	def __init__(self):
		super().__init__('lasertracker')
		self.lastScan=None
		qos = QoSProfile(depth=10)
		self.winSize = self.declare_parameter('~winSize', 2)
		self.deltaDist =self.declare_parameter('~deltaDist', 0.2)
		self.positionPublisher = self.create_publisher(PositionMsg, 'object_tracker/current_position', qos)
		self.infoPublisher = self.create_publisher(StringMsg, 'object_tracker/info', qos)
		self.scanSubscriber = self.create_subscription(
		    LaserScan,
		    '/scan',
		    self.registerScan,
		    qos)
	def registerScan(self, scan_data):
		ranges = np.array(scan_data.ranges)
		# sort by distance to check from closer to further away points if they might be something real
		sortedIndices = np.argsort(ranges)
		minDistanceID = None
		minDistance   = float('inf')	
		if(not(self.lastScan is None)):
			# if we already have a last scan to compare to:
			for i in sortedIndices:
				# check all distance measurements starting from the closest one
				tempMinDistance   = ranges[i]
				a = i-2
				b = i+3
				c = len(self.lastScan)
				windowIndex = np.clip([a,b],0,c)
				window = self.lastScan[windowIndex[0]:windowIndex[1]]
				with np.errstate(invalid='ignore'):
					if(np.any(abs(window-tempMinDistance)<=0.2)):
						minDistanceID = i
						minDistance = ranges[minDistanceID]
						break # at least one point was equally close
		self.lastScan=ranges

		if(minDistance > scan_data.range_max):
			msg = StringMsg()
			msg.data = 'laser:nothing found'
			self.get_logger().warn('laser no object found')
			self.infoPublisher.publish(msg)
			self.get_logger().info(' waiting again...')
		else:
			msgdata = PositionMsg()
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
			msgdata.angle_x = minDistanceAngle
			msgdata.angle_y = 42.0
			print(minDistanceID)
			msgdata.distance = float(minDistance)
			self.positionPublisher.publish(msgdata)
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    lasertracker = LaserTracker()
    print('seem to do something')
    try:
        rclpy.spin(lasertracker)
    finally:
        lasertracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

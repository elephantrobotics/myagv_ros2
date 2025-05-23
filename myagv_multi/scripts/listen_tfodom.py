#!/usr/bin/env python3
# coding=utf-8
# 从车接收主车坐标以及主车速度，并通过话题multfodom发布出来

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time
import socket
import struct
from std_msgs.msg import Float32MultiArray


def FrameListener():
	rclpy.init()
	node = rclpy.create_node('listen_tfodom')
	qos = QoSProfile(depth=10)
	pub = node.create_publisher(Float32MultiArray,'multfodom', qos)
	soc=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	soc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 30)
	soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)#udp 广播
	soc.bind(('',10000))
	while rclpy.ok():
		data, address = soc.recvfrom(30)
		(posx,posy,posaz,odomvx,odomvy,odomaz) = struct.unpack("ffffff",data)
		print(posx,posy,posaz,odomvx,odomvy,odomaz)
		msg = Float32MultiArray()
		msg.data.append(posx)
		msg.data.append(posy)
		msg.data.append(posaz)
		msg.data.append(odomvx)
		msg.data.append(odomvy)
		msg.data.append(odomaz)
		pub.publish(msg)
		# rclpy.spin_once(node, timeout_sec=0.1)
		# time.sleep(0.05)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	FrameListener()



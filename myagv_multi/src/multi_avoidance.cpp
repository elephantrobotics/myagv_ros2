/**************************************************************************
功能：避障
**************************************************************************/
#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <geometry_msgs/msg/twist.hpp>
#include <string.h>
#include <math.h>
#include <iostream>
#include <myagv_multi/msg/position.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>


using namespace std;
 
geometry_msgs::msg::Twist cmd_vel_msg;    //速度控制信息数据
geometry_msgs::msg::Twist cmd_vel_avoid;    //速度控制信息数据
geometry_msgs::msg::Twist cmd_vel_data;    //速度控制信息数据

float distance1=100.0;    //障碍物距离
float dis_angleX=0.0;    //障碍物方向,前面为0度角，右边为正，左边为负	
double safe_distence;
double danger_distence;
double danger_angular;
double avoidance_kv;
double avoidance_kw;
double max_vel_x;
double min_vel_x;
double max_vel_theta;
double min_vel_theta;

/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const myagv_multi::msg::Position& msg)	
{
	distance1 = msg.distance;
	dis_angleX = msg.angle_x;   
}

/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void cmd_vel_ori_Callback(const geometry_msgs::msg::Twist& msg)
{
	cmd_vel_msg.linear.x = msg.linear.x;
	cmd_vel_msg.angular.z = msg.angular.z;

	cmd_vel_data.linear.x = msg.linear.x;
	cmd_vel_data.angular.z = msg.angular.z;
}


/**************************************************************************
函数功能：主函数
入口参数：无
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_multi wheeltec_slave.launch.py"
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
	rclcpp::init(argc,argv);    //初始化ROS节点
	rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("multi_avoidance");
	auto cmd_vel_Pub = nh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	auto cmdori_sub = nh->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_ori", 5,cmd_vel_ori_Callback);
	auto current_position_sub = nh->create_subscription<myagv_multi::msg::Position>("object_tracker/current_position", 5,current_position_Callback);

	nh->declare_parameter<double>("safe_distence",0.5);
	nh->get_parameter("safe_distence", safe_distence);
	nh->declare_parameter<double>("danger_distence",0.2);
	nh->get_parameter("danger_distence", danger_distence);
	nh->declare_parameter<double>("danger_angular",0.785);
	nh->get_parameter("danger_angular", danger_angular);
	nh->declare_parameter<double>("avoidance_kv",0.2);
	nh->get_parameter("avoidance_kv", avoidance_kv);
	nh->declare_parameter<double>("avoidance_kw",0.3);
	nh->get_parameter("avoidance_kw", avoidance_kw);
	nh->declare_parameter<double>("max_vel_x",1.5);
	nh->get_parameter("max_vel_x", max_vel_x);
	nh->declare_parameter<double>("min_vel_x",0.05);
	nh->get_parameter("min_vel_x", min_vel_x);
	nh->declare_parameter<double>("max_vel_theta",1.5);
	nh->get_parameter("max_vel_theta", max_vel_theta);
	nh->declare_parameter<double>("min_vel_theta",0.05);
	nh->get_parameter("min_vel_theta", min_vel_theta);
  
	rclcpp::Rate loop_rate(10);
	while(rclcpp::ok())
	{
		avoidance_kw = fabs(avoidance_kw);
		if(distance1<safe_distence && distance1>danger_distence)		//障碍物在安全距离和危险距离时，调整速度角度避让障碍物
		{
			//printf("distance1= %f\n",distance1);
			cmd_vel_msg.linear.x = cmd_vel_data.linear.x - fabs(cmd_vel_data.linear.x)*avoidance_kv*cos(dis_angleX)/distance1;//原始速度，减去一个后退的速度
			if(fabs(cmd_vel_msg.linear.x)>fabs(cmd_vel_data.linear.x) && cmd_vel_msg.linear.x*cmd_vel_data.linear.x>0)cmd_vel_msg.linear.x=cmd_vel_data.linear.x;//禁止小车加速
			if(dis_angleX<0)avoidance_kw=-avoidance_kw;											//车左右边的障碍物避障，车头调转方向不一致
			cmd_vel_msg.angular.z = cmd_vel_data.angular.z + avoidance_kw*cos(dis_angleX)/distance1;

		}
		else if(distance1<danger_distence)				//障碍物在危险距离之内时，以远离障碍物为主
		{
			//printf("distance1= %f\n",distance1);
			cmd_vel_msg.linear.x =  - avoidance_kv*cos(dis_angleX);
			if(dis_angleX<0)avoidance_kw=-avoidance_kw;
			cmd_vel_msg.angular.z = avoidance_kw*cos(dis_angleX);
		}
		else										//其他情况直接输出原始速度
		{
			cmd_vel_msg.linear.x = cmd_vel_data.linear.x;
			cmd_vel_msg.angular.z = cmd_vel_data.angular.z;
		}

		//速度限制
		if(cmd_vel_msg.linear.x > max_vel_x)
			cmd_vel_msg.linear.x=max_vel_x;
		else if(cmd_vel_msg.linear.x < -max_vel_x)
			cmd_vel_msg.linear.x=-max_vel_x;
		if(fabs(cmd_vel_msg.linear.x) < min_vel_x)
			cmd_vel_msg.linear.x=0;
		if(cmd_vel_msg.angular.z > max_vel_theta)
			cmd_vel_msg.angular.z=max_vel_theta;
		else if(cmd_vel_msg.angular.z < -max_vel_theta)
			cmd_vel_msg.angular.z=-max_vel_theta;
		if(fabs(cmd_vel_msg.angular.z) < min_vel_theta)
			cmd_vel_msg.angular.z=0;

		cmd_vel_Pub->publish(cmd_vel_msg);
		rclcpp::spin_some(nh);
		loop_rate.sleep();
	} 

	return 0;
}

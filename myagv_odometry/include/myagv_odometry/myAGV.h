#ifndef MYAGV_H
#define MYAGV_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/asio.hpp>
#include <chrono>
#include <memory>
#include <boost/array.hpp>

#define twoKpDef	1.0f				// (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	0.0f				// (2.0f * 0.0f)	// 2 * integral gain
#define TOTAL_RECEIVE_SIZE 43         	// 43 RECEIVE_SIZE(v1.1) //The length of the data sent by the esp32
#define OFFSET_COUNT 	200

class MyAGV : public rclcpp::Node{
public:
    MyAGV(const std::string &node_name = "myagv_odometry_node");
	~MyAGV();
	bool init();
	void execute(double linearX, double linearY, double angularZ);
	void publisherOdom(double dt);
	void publisherImuSensor();
	void Publish_Voltage();

private:
	bool readSpeed();
	void writeSpeed(double movex, double movey, double rot);
	void restore();
	void restoreRun();
	void clearSerialBuffer();

	bool initialized = false;

	double x= 0.0;
	double y= 0.0;
	double theta= 0.0;

	double vx= 0.0;
	double vy= 0.0;
	double vtheta= 0.0;

	double ax;
	double ay;
	double az;

	double wx;
	double wy;
	double wz;
	
	double roll;
	double pitch;
	double yaw;
    
	float Battery_voltage,Backup_Battery_voltage;
	float present_theta = 0.0f;         
	float last_theta = 0.0f;            
	float delta_theta = 0.0f;           
	float accumulated_theta = 0.0f; 

    std::shared_ptr<rclcpp::Node> node_;
	std::string frame_id_of_odometry_;
	std::string child_frame_id_of_odometry_;
	std::string frame_id_of_imu_;
	std::string name_space_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage_backup;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;
    rclcpp::Time currentTime, lastTime;
    sensor_msgs::msg::Imu imu_data;
};

#endif // !MYAGV_H
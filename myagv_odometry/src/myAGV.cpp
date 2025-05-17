#include <vector>
#include <iostream>
#include <iomanip>
#include <time.h>

#include "myagv_odometry/myAGV.h"

const unsigned char header[2] = { 0xfe, 0xfe };

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyS0");

std::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };
std::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };

MyAGV::MyAGV(const std::string &node_name) : Node(node_name)
{

}


MyAGV::~MyAGV()
{
    ;
}

bool MyAGV::init()
{
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));
    clearSerialBuffer();

    lastTime = this->get_clock()->now();
    odomBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    this->declare_parameter<std::string>("odometry.frame_id", "odom");
    this->declare_parameter<std::string>("odometry.child_frame_id", "base_footprint");
    this->declare_parameter<std::string>("imu.frame_id", "imu_link");
    this->declare_parameter<std::string>("namespace", "");
    pub_imu =  create_publisher<sensor_msgs::msg::Imu>("imu", 20);
    pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom", 50); 
    pub_voltage = create_publisher<std_msgs::msg::Float32>("voltage", 10);
    pub_voltage_backup = create_publisher<std_msgs::msg::Float32>("voltage_backup", 10);
    restore(); //first restore,abort current err,don't restore

    this->get_parameter_or<std::string>(
        "odometry.frame_id",
        frame_id_of_odometry_,
        std::string("odom"));
    
    this->get_parameter_or<std::string>(
        "odometry.child_frame_id",
        child_frame_id_of_odometry_,
        std::string("base_footprint"));

    this->get_parameter_or<std::string>(
        "imu.frame_id",
        frame_id_of_imu_,
        std::string("imu_link"));        

    this->get_parameter_or<std::string>(
        "namespace",
        name_space_,
        std::string(""));

    if (name_space_ != "") {
        frame_id_of_odometry_ = name_space_ + "/" + frame_id_of_odometry_;
        child_frame_id_of_odometry_ = name_space_ + "/" + child_frame_id_of_odometry_;
        frame_id_of_imu_ = name_space_ + "/" + frame_id_of_imu_;
    }

    return true;
}

void MyAGV::restore()
{
    // Clear serial port buffer by reading at least 1 byte
    boost::asio::streambuf clear_buffer; 
    boost::asio::read(sp, clear_buffer, boost::asio::transfer_at_least(1));

    // Pause for 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Motor Stall Recovery
    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};

    std::cout << "Sending data: ";
    for (int i = 0; i < 6; ++i) 
    {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)(cmd[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    // Write command data to the serial port
    boost::asio::write(sp, boost::asio::buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return;
}

void MyAGV::restoreRun()
{
    int res = 0;
    std::cout << "if you want restore run,pls input 1,then press enter" << std::endl;
    while(res != 1) {
        std::cin >> res;
        std::cout <<  "press enter" << std::endl;
        std::cout << res;
    }
    restore();
    std::cout <<  "restore finished" << std::endl;
    return;
}

void MyAGV::clearSerialBuffer() 
{
    int fd = sp.native_handle();
    if (tcflush(fd, TCIOFLUSH) < 0) {
        perror("Failed to clear serial buffer");
    } else {
        std::cout << "Serial buffer cleared successfully" << std::endl;
    }					   
    return;
}

bool MyAGV::readSpeed()
{
    int count = 0;
    unsigned char buf_header[1] = {0};
    unsigned char buf[TOTAL_RECEIVE_SIZE] = {0};

    size_t ret;
    boost::system::error_code er2;
    bool header_found = false;
    while (!header_found) {
        ++count;
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) {
            continue;
        }
        if (buf_header[0] != header[0]) {
            continue;
        }
        bool header_2_found = false;
        while (!header_2_found) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            header_2_found = true;
        }
        header_found = true;
    }

    ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);  // ready break
    if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
        int wheel_num = 0;
        for (int i = 0; i < 4; ++i) {
            if (buf[i] == 1) {
                wheel_num = i+1;
                //ROS_ERROR("ERROR %d wheel current > 2000", wheel_num);
                RCLCPP_ERROR(this->get_logger(),"ERROR %d wheel current > 2000", wheel_num);
            }
        }
        restoreRun();
        return false;
    }
    if (ret != TOTAL_RECEIVE_SIZE) {
        //ROS_ERROR("Read error %zu",ret);
        if (ret == 27) RCLCPP_ERROR(this->get_logger(), "Please use myStudio to burn v1.1 firmware");
        RCLCPP_ERROR(this->get_logger(),"Read error %zu",ret);
        return false;
    }

    int index = 0;
    int check = 0;//ilter time older than imu message buffer
    for (int i = 0; i < (TOTAL_RECEIVE_SIZE-1); ++i)
        check += buf[index + i];
    if (check % 256 != buf[index + (TOTAL_RECEIVE_SIZE-1)])
    {
        //ROS_ERROR("Error:Serial port verification failed! check:%d -- %d ",check,buf[index+(TOTAL_RECEIVE_SIZE-1)]);	
        RCLCPP_ERROR(this->get_logger(),"Error:Serial port verification failed! check:%d -- %d ",check,buf[index+(TOTAL_RECEIVE_SIZE-1)]);
        return false;
    }

    vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;
    vy = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01;
    vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01;

    imu_data.linear_acceleration.x = ((buf[index + 3] + buf[index + 4] * 256 ) - 10000) * 0.001;
    imu_data.linear_acceleration.y = ((buf[index + 5] + buf[index + 6] * 256 ) - 10000) * 0.001;
    imu_data.linear_acceleration.z = ((buf[index + 7] + buf[index + 8] * 256 ) - 10000) * 0.001;

    imu_data.angular_velocity.x = ((buf[index + 9] + buf[index + 10] * 256 ) - 10000) * 0.1;
    imu_data.angular_velocity.y = ((buf[index + 11] + buf[index + 12] * 256 ) - 10000) * 0.1;
    imu_data.angular_velocity.z = ((buf[index + 13] + buf[index + 14] * 256 ) - 10000) * 0.1;

    uint8_t highNibble = (buf[index + 15] >> 4) & 0x0F;  

    Battery_voltage = (float)buf[index + 16] / 10.0f;

    bool isBatteryWithBackup = (highNibble == 0x03);
    if (isBatteryWithBackup){
        Backup_Battery_voltage = (float)buf[index + 17] / 10.0f;
    }else{
        Backup_Battery_voltage = 0.0f;
    }
    
    roll  = (int16_t)((buf[index + 26] << 8) | (buf[index + 27] & 0xff)) * 0.01;
    pitch = (int16_t)((buf[index + 28] << 8) | (buf[index + 29] & 0xff)) * 0.01;
    yaw   = (int16_t)((buf[index + 30] << 8) | (buf[index + 31] & 0xff)) * 0.01;

    //ROS_INFO("yaw:%f",yaw);

    if (!initialized)
    {
        present_theta = last_theta = yaw;
        initialized = true;
    }
    present_theta = yaw;
    delta_theta = present_theta - last_theta;
    if(delta_theta< 0.1 && delta_theta > -0.1) delta_theta=0;
    accumulated_theta += delta_theta;
    //ROS_INFO("accumulated_theta:%f",accumulated_theta);
    last_theta = present_theta;

    //std::cout << "Received message is: "  << "|" << vx << "," << vy << "," << vtheta << "|"
                                        //  << imu_data.linear_acceleration.x << "," << imu_data.linear_acceleration.y << "," << imu_data.linear_acceleration.z << "|"
                                    //  << imu_data.angular_velocity.x << "," << imu_data.angular_velocity.y << "," << imu_data.angular_velocity.z << std::endl;
    //std::cout << "current pos is: " << x << "," << y << "," << theta << std::endl;

    return true;
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    if (movex > 1.0) movex = 1.0;
    if (movex < -1.0) movex = -1.0;
    if (movey > 1.0) movey = 1.0;
    if (movey < -1.0) movey = -1.0;
    if (rot > 1.0) rot = 1.0;
    if (rot < -1.0) rot = -1.0;

    unsigned char x_send = static_cast<signed char>(movex * 100) + 128;
    unsigned char y_send = static_cast<signed char>(movey * 100) + 128;
    unsigned char rot_send = static_cast<signed char>(rot * 100) + 128;
    unsigned char check = x_send + y_send + rot_send;

    char buf[8] = { 0 };
    buf[0] = header[0];
    buf[1] = header[1];
    buf[2] = x_send;
    buf[3] = y_send;
    buf[4] = rot_send;
    buf[5] = check;

    boost::asio::write(sp, boost::asio::buffer(buf));
}

void MyAGV::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msg,voltage_backup_msg;
    voltage_msg.data = Battery_voltage;
    pub_voltage->publish(voltage_msg);

    voltage_backup_msg.data = Backup_Battery_voltage;
    pub_voltage_backup->publish(voltage_backup_msg);

}

void MyAGV::publisherImuSensor()
{
    sensor_msgs::msg::Imu ImuSensor;

    ImuSensor.header.stamp = this->get_clock()->now();; 
    ImuSensor.header.frame_id = frame_id_of_imu_;

    tf2::Quaternion qua;
    qua.setRPY(0, 0, yaw * M_PI / 180.0);

    ImuSensor.orientation.x = qua[0]; 
    ImuSensor.orientation.y = qua[1]; 
    ImuSensor.orientation.z = qua[2];
    ImuSensor.orientation.w = qua[3];

    ImuSensor.angular_velocity.x = imu_data.angular_velocity.x;		
    ImuSensor.angular_velocity.y = imu_data.angular_velocity.y;		
    ImuSensor.angular_velocity.z = imu_data.angular_velocity.z;

    ImuSensor.linear_acceleration.x = imu_data.linear_acceleration.x;
    ImuSensor.linear_acceleration.y = imu_data.linear_acceleration.y;
    ImuSensor.linear_acceleration.z = imu_data.linear_acceleration.z;

    ImuSensor.orientation_covariance[0] = 1e6;
    ImuSensor.orientation_covariance[4] = 1e6;
    ImuSensor.orientation_covariance[8] = 1e-6;

    ImuSensor.angular_velocity_covariance[0] = 1e6;
    ImuSensor.angular_velocity_covariance[4] = 1e6;
    ImuSensor.angular_velocity_covariance[8] = 1e-6;

    pub_imu->publish(ImuSensor); 
}

void MyAGV::publisherOdom(double dt)
{   
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = this->get_clock()->now();
    odom_trans.header.frame_id = frame_id_of_odometry_;
    odom_trans.child_frame_id = child_frame_id_of_odometry_;

    geometry_msgs::msg::Quaternion odom_quat;

    theta = accumulated_theta * M_PI / 180.0;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    odom_quat = tf2::toMsg(quat);

    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;

    x += delta_x;
    y += delta_y;

    odom_trans.transform.translation.x = x; 
    odom_trans.transform.translation.y = y; 
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = odom_quat;

    // odomBroadcaster->sendTransform(odom_trans); // Use the Robot Localization ros package instead.

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();;
    odom.header.frame_id = frame_id_of_odometry_;
    odom.child_frame_id = child_frame_id_of_odometry_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = odom_pose_covariance;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vtheta;
    odom.twist.covariance = odom_twist_covariance;

    pub_odom->publish(odom);
}

void MyAGV::execute(double linearX, double linearY, double angularZ)
{   
    currentTime = this->get_clock()->now();   
    double dt = (currentTime - lastTime).seconds();
    if (true ==  readSpeed()) 
    {    
        writeSpeed(linearX, linearY, angularZ);
        publisherOdom(dt);
        //ROS_INFO("dt:%f",dt);
        publisherImuSensor();
        Publish_Voltage();
    } 
    lastTime = currentTime;
}

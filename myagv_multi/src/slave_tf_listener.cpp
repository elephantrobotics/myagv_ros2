/**************************************************************************
作者：lyk
功能：多机编队
**************************************************************************/
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include <tf2/convert.h>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/msg/float32_multi_array.hpp>


int multi_mode = 2;//编队模式选择：目前有模式1：整体自转 和模式2：各个小车原地自转

double e_linear_x = 0;    //从机与目标点x方向的偏差（小车前后方向的偏差）
double e_linear_y = 0;    //从机与目标点y方向的偏差（小车左右方向的偏差）
double e_angular_z = 0;    //从机与目标点角度偏差
double angular_turn = 0;     //修正从机运动方向

double slave_x = 0.8;    //目标点的x坐标
double slave_y = -0.8;    //目标点的y坐标

double max_vel_x=1.0;     //最大速度限制
double min_vel_x=0.05;     //最小速度限制
double max_vel_theta=1.0; //最大角速度限制
double min_vel_theta=0.05;//最小角速度限制
#include <tf2/convert.h>

double k_v=1;   //调节前后方向偏差时，k_v越大，线速度越大
double k_l=1;   //调节左右方向偏差时，k_l越大，角速度越大
double k_a=1;   //调节角度偏差时，k_a越大，角速度越大

bool avoidance;

// tf变换相关参数
std::string base_frame;
std::string base_to_slave;
std::string tf_prefix_;

//主车速度
float odom_linear_x=0;    
float odom_linear_y=0;
float odom_angular_z=0;

//主车速度
float trans_x=0;    
float trans_y=0;
float trans_z=0;

//initial初始化位置成功标志
bool setpose=0;

/**************************************************************************
函数功能：主车信息sub回调函数
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void vel_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  //主车x,y位置，以及方向角
  trans_x = msg->data[0];
  trans_y = msg->data[1];
  trans_z = msg->data[2];
  //主车线速度以及角速度信息
  odom_linear_x = msg->data[3];
  odom_linear_y = msg->data[4];
  odom_angular_z = msg->data[5];
  if(fabs(odom_linear_x)<min_vel_x)odom_linear_x=0;
}
void multi_mode_Callback(const std_msgs::msg::Int32& msg)
{
  multi_mode = msg.data;
}

void setpose_Callback(const std_msgs::msg::Bool& msg)
{
  setpose = msg.data;
}


int main(int argc, char** argv){

  rclcpp::init(argc,argv);    //初始化ROS节点
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("multi_slave");
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_Pub;

  nh->declare_parameter<int>("multi_mode",2);
  nh->get_parameter("multi_mode", multi_mode);
  nh->declare_parameter<double>("slave_x",0.6);
  nh->get_parameter("slave_x", slave_x);
  nh->declare_parameter<double>("slave_y",-0.6);
  nh->get_parameter("slave_y", slave_y);
  nh->declare_parameter<double>("max_vel_x",0.5);
  nh->get_parameter("max_vel_x", max_vel_x);
  nh->declare_parameter<double>("min_vel_x",0.05);
  nh->get_parameter("min_vel_x", min_vel_x);
  nh->declare_parameter<double>("max_vel_theta",0.5);
  nh->get_parameter("max_vel_theta", max_vel_theta);
  nh->declare_parameter<double>("min_vel_theta",0.05);
  nh->get_parameter("min_vel_theta", min_vel_theta);
  nh->declare_parameter<double>("k_v",1);
  nh->get_parameter("k_v", k_v);
  nh->declare_parameter<double>("k_l",1);
  nh->get_parameter("k_l", k_l);
  nh->declare_parameter<double>("k_a",1);
  nh->get_parameter("k_a", k_a);
  nh->declare_parameter<bool>("avoidance",true);
  nh->get_parameter("avoidance", avoidance);

  cmd_vel_Pub = nh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_ori", 10);
  auto vel_sub = nh->create_subscription<std_msgs::msg::Float32MultiArray>("/multfodom", 5,vel_Callback);
  auto multi_mode_sub = nh->create_subscription<std_msgs::msg::Int32>("/multi_mode_topic", 5,multi_mode_Callback);
  auto setpose_sub = nh->create_subscription<std_msgs::msg::Bool>("/set_initialpose", 5,setpose_Callback);

//中间变量
  geometry_msgs::msg::Twist vel_msg;
  slave_x = slave_x+slave_y;
  slave_y = slave_x-slave_y;
  slave_x = -(slave_x-slave_y);//使得期望坐标slave_x slave_y是以主车为坐标原点，右边为x正方向，前面为y正方向的坐标系

  //tf变换中间变量
  geometry_msgs::msg::TransformStamped transformSM1,transformSM2;
  tf2_ros::Buffer buffer1(nh->get_clock());
  tf2_ros::Buffer buffer2(nh->get_clock());
  tf2_ros::TransformListener tfListener1(buffer1);
  tf2_ros::TransformListener tfListener2(buffer2);
  rclcpp::Rate rate(10.0);
  
  //等待小车初始化位置完成
  while (!setpose)
  {
    rclcpp::spin_some(nh);
  }
  rclcpp::Rate(1).sleep();

  while (rclcpp::ok()){

    //根据编队模式不同，选择不同的运动模型
    if(multi_mode==1)
    {
      //读取从车位置坐标

      try
      {
        transformSM1 = buffer1.lookupTransform("map", "base_link",tf2::TimePoint());
        transformSM2 = buffer2.lookupTransform("base_link","map", tf2::TimePoint());
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_INFO(nh->get_logger(), "transform lookup unsuccessful");
        rclcpp::Rate(1).sleep();
        continue;
      }

      //求出_baselinktoslave从车到从车期望坐标的tf变换关系
      tf2::Transform maptobaselink , baselinktoslave , maptoslave , _baselinktoslave;
      maptobaselink.setOrigin(tf2::Vector3(trans_x, trans_y, 0.0)) ;
      tf2::Quaternion qa1;
      qa1.setRPY(0.0, 0.0, trans_z);
      maptobaselink.setRotation(qa1);//主车map到主车base_link的tf变换

      baselinktoslave.setOrigin(tf2::Vector3(slave_y, -slave_x, 0.0));
      tf2::Quaternion qa2;
      qa2.setRPY(0.0, 0.0, 0.0);
      baselinktoslave.setRotation(qa2);//主车base_link到从车期望坐标的tf变换

      maptoslave = maptobaselink*baselinktoslave;//主车map到从车期望坐标slave的tf变换：由maptobaselink ,baselinktoslave变换关系计算可得到
      tf2::Transform _baselinktomap;//从车base_link到主车map的变换
      tf2::fromMsg(transformSM2.transform, _baselinktomap);
      _baselinktoslave = _baselinktomap*maptoslave;//从车base_link到期望坐标slave的tf变换：由_baselinktomap ,maptoslave变换关系计算可得到

      //从车与期望坐标的偏差
      e_linear_x =_baselinktoslave.getOrigin().x();       //x方向的偏差（小车前后方向的偏差）
      e_linear_y =_baselinktoslave.getOrigin().y();       //y方向的偏差（小车左右方向的偏差）
      e_angular_z = tf2::getYaw(_baselinktoslave.getRotation());       //角度偏差
      
      // angular_turn：当主车转弯时，需要修正从车期望坐标朝向的角度值（使得从车期望坐标的朝向为从车运动轨迹圆的切线方向）
      if(fabs(odom_linear_x/odom_angular_z)<5.0)angular_turn = atan2(slave_y,slave_x+odom_linear_x/odom_angular_z);//当odom_linear_x/odom_angular_z)<5.0时，可认为主车在做转弯运动
      else angular_turn = atan2(slave_y,slave_x);//当odom_linear_x/odom_angular_z)>5.0时，可认为主车在做直线运动，这样避免odom_linear_x/odom_angular_z)计算出的值太大，影响angular_turn的结果
      if(fabs(odom_angular_z)>min_vel_theta)//当主车转弯时，修正期望（使得前进方向为从车运动轨迹圆的切线方向），才能跟随主车运动
      {
         if(slave_x>0 && slave_y>=0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z-3.14+angular_turn;
        }
        else if(slave_x<=0 && slave_y>=0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+angular_turn-3.14;
        }
        else if(slave_x<0 && slave_y<0)
        {
          if(odom_angular_z>=0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+3.14+angular_turn;
        }
        else if(slave_x>=0 && slave_y<0)
        {
          if(odom_angular_z>0)e_angular_z=e_angular_z+angular_turn;
          else e_angular_z=e_angular_z+3.14+angular_turn;
        }
        if(e_angular_z>4.71 && e_angular_z<7.85)e_angular_z=e_angular_z-6.28;
        if(e_angular_z<-4.71 && e_angular_z>-7.85)e_angular_z=e_angular_z+6.28;
      }

      printf("Deviation: (e_x = %f e_y = %f e_z =%f)\n",e_linear_x,e_linear_y,e_angular_z);
      //计算得到从机的线速度
      if(fabs(odom_linear_x/odom_angular_z)<5.0)//odom_linear_x/odom_angular_z<5.0可视为小车在做非直线的运动
      {
        double d_r;                             //d_r为主车运动半径与期望跟随点的运动半径之差，
        if(slave_x+odom_linear_x/odom_angular_z>=0)
        {
          d_r = sqrt(pow(slave_x+odom_linear_x/odom_angular_z,2)+pow(slave_y,2))-odom_linear_x/odom_angular_z;
        }
        else 
        {
          d_r = -sqrt(pow(slave_x+odom_linear_x/odom_angular_z,2)+pow(slave_y,2))-odom_linear_x/odom_angular_z;
        }
        vel_msg.linear.x = (odom_linear_x+odom_angular_z*d_r)*fabs(cos(e_angular_z))+k_v*e_linear_x; //根据v/w=r，其中w从机=w主机，所以
                                    //  v从车/w从车=(odom_linear_x+odom_angular_z*d_r) /odom_angular_z=r主车运动半径+d_r = r从车运动半径，符合运动学规律
      }
      else 
      {
        vel_msg.linear.x = odom_linear_x+k_v*e_linear_x; //直线运动时，v从车 = v主车 + 前后方向修正的偏差
      }
      float _k_a=k_a;
      float _k_l=k_l;
      if(vel_msg.linear.x<-min_vel_x)//从车后退时，修正参数的正负值与前进的相反
      {
        if(fabs(odom_angular_z)>min_vel_theta)_k_a=-k_a;//直行运动的_k_a参数为正
        _k_l=-k_l;
      }
      //计算得到从机的角速度
      vel_msg.angular.z = odom_angular_z+_k_l*e_linear_y+_k_a*sin(e_angular_z);//w从车角速度 = v主车角速度 + 左右方向修正的偏差+角度方向修正偏差
    }
    else if(multi_mode==2)
    {
      try
      {
        transformSM2 = buffer1.lookupTransform("map", "base_link",tf2::TimePoint());
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_INFO(nh->get_logger(), "transform lookup unsuccessful");
        rclcpp::Rate(1).sleep();
        continue;
      }

      double angular_z2 ;
      angular_z2 = tf2::getYaw(transformSM2.transform.rotation);//读取从车的方向角angular_z2

      e_linear_x = trans_x-transformSM2.transform.translation.x+slave_y;      //x方向的偏差（小车前后方向的偏差）
      e_linear_y = trans_y-transformSM2.transform.translation.y-slave_x;       //y方向的偏差（小车左右方向的偏差）
      e_angular_z = trans_z-angular_z2;                                                  //角度偏差
      printf("Deviation: (e_x = %f e_y = %f e_z =%f)\n",e_linear_x,e_linear_y,e_angular_z);

      vel_msg.linear.x = odom_linear_x+k_v*e_linear_x*cos(angular_z2)+k_v*e_linear_y*sin(angular_z2); //根据当前从车的方向angular_z2，调整速度修正从车的x y方向偏差（例如从车朝向在x方向0角度时，cos(0)=1,调整小车速度修正x方向偏差）
      float _k_a=k_a;
      float _k_l=k_l;
      if(vel_msg.linear.x<-min_vel_x)//小车后退时，控制车头转动方向与前进时相反（例如从车在期望位置正右位置，前进时应该使车头向左偏修正，后退时应该使车头向右偏修正）
      {
        _k_l=-k_l;
      }
      //根据当前从车当前朝向方向angular_z2，在模仿主车运动的同时，调整车头方向，修正从车的x y ，角度方向偏差
      vel_msg.angular.z = odom_angular_z+0.5*_k_l*e_linear_y*cos(angular_z2)-0.5*_k_l*e_linear_x*sin(angular_z2)+_k_a*sin(e_angular_z);
    }

    //速度限制
    if(vel_msg.linear.x > max_vel_x)
      vel_msg.linear.x=max_vel_x;
    else if(vel_msg.linear.x < -max_vel_x)
      vel_msg.linear.x=-max_vel_x;
    if(fabs(vel_msg.linear.x) < min_vel_x)
      vel_msg.linear.x=0;
    if(vel_msg.angular.z > max_vel_theta)
      vel_msg.angular.z=max_vel_theta;
    else if(vel_msg.angular.z < -max_vel_theta)
      vel_msg.angular.z=-max_vel_theta;
    if(fabs(vel_msg.angular.z) < min_vel_theta)
      vel_msg.angular.z=0;

    cmd_vel_Pub->publish(vel_msg);
    rclcpp::spin_some(nh);
    rate.sleep();
  }

  return 0;
}


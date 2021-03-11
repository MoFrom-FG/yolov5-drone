/*********************************************************************
  > File Name: gimbal_serial.cpp
  >  
  >            
  > Author   : EasonYi
  > Time     : 2021-03-10
  > email    : eason473867143@gmail.com
**********************************************************************/
#include <ros/ros.h>
#include <serial/serial.h>  
#include <std_msgs/String.h> 
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <string.h>
#include <prometheus_msgs/Cloud_platform.h>
  
using namespace std;
prometheus_msgs::Cloud_platform  cloud_platform;

serial::Serial serr; 
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "gimbal_serial"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //发布主题 
    ros::Publisher write_pub = nh.advertise<prometheus_msgs::Cloud_platform>("/write", 10); 
    //指定循环的频率 
    ros::Rate loop_rate(50);

    // 获取云台角度数据的命令
    //3E 3D 00 3D 00 cmd_get_angles
    cloud_platform.get_data_com[0]=0x3e; 
    cloud_platform.get_data_com[1]=0x3d;
    cloud_platform.get_data_com[2]=0x00;
    cloud_platform.get_data_com[3]=0x3d;
    cloud_platform.get_data_com[4]=0x00;
    while(ros::ok()) 
    { 
        write_pub.publish(cloud_platform); 
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 





















/* *****************************************************

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_ros");
  ros::NodeHandle nh;

  // 频率 [20Hz]
  ros::Rate rate(20.0);

  while(ros::ok())
  {
     cout << " creat ros project test !"<<endl;
     //回调
     ros::spinOnce();
     //挂起一段时间(rate为 50HZ)
     rate.sleep();
  }
}

 * ************************************************************************/

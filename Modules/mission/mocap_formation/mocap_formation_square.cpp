/*******************************************************************
 * 文件名:mocap_formation_control.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.11.20
 * 
 * 介绍:该cpp文件主要为动捕集群中四机绕圈相关函数的实现以及程序的运行
 * ****************************************************************/
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mocap_formation_square");
	ros::NodeHandle n;
	ros::Rate r(10);

	//集群五台飞机位置控制数据发布者
  	ros::Publisher uav1_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
  	ros::Publisher uav2_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);
  	ros::Publisher uav3_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
  	ros::Publisher uav4_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav4/mavros/setpoint_raw/local", 10);

	//读取相关参数
	double square_length;
	double uav13_height;
	double uav24_height;
	double hold_time;
	double stage1_time;
	bool sim;
	//1号机仿真位置差值
  	Eigen::Vector3d uav1_gazebo_offset_pose;
	//2号机仿真位置差值
  	Eigen::Vector3d uav2_gazebo_offset_pose;
	//3号机仿真位置差值
  	Eigen::Vector3d uav3_gazebo_offset_pose;
	//4号机仿真位置差值
  	Eigen::Vector3d uav4_gazebo_offset_pose;

	ros::param::param<double>("~square_length", square_length, 4);
	ros::param::param<double>("~13_height", uav13_height, 1);
	ros::param::param<double>("~24_height", uav24_height, 0.5);
	ros::param::param<double>("~hold_time", hold_time, 10);
	ros::param::param<double>("~stage1_time", stage1_time, 5);
	ros::param::param<bool>("~sim", sim, false);
	ros::param::param<double>("~uav1_x", uav1_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("~uav1_y", uav1_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("~uav2_x", uav2_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("~uav2_y", uav2_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("~uav3_x", uav3_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("~uav3_y", uav3_gazebo_offset_pose[1], 0);
  	ros::param::param<double>("~uav4_x", uav4_gazebo_offset_pose[0], 0);
  	ros::param::param<double>("~uav4_y", uav4_gazebo_offset_pose[1], 0);

	mavros_msgs::PositionTarget uav1_local_pose;
	mavros_msgs::PositionTarget uav2_local_pose;
	mavros_msgs::PositionTarget uav3_local_pose;
	mavros_msgs::PositionTarget uav4_local_pose;

	if(!sim)
	{
		uav1_gazebo_offset_pose[0] = 0;
		uav1_gazebo_offset_pose[1] = 0;
		uav2_gazebo_offset_pose[0] = 0;
		uav2_gazebo_offset_pose[1] = 0;
		uav3_gazebo_offset_pose[0] = 0;
		uav3_gazebo_offset_pose[1] = 0;
		uav4_gazebo_offset_pose[0] = 0;
		uav4_gazebo_offset_pose[1] = 0;
	}

	while(ros::ok())
	{
		std::cout << "Please input 1 start square formation" << std::endl;
		int start_flag;
		std::cin >> start_flag;
		if(start_flag == 1)
		{
			//阶段1
			int stage = 1;
			if(stage == 1)
			{
				int count = 0;
				int num = (stage1_time + hold_time) * 10;
				uav1_local_pose.type_mask = 0b100111111000;
				uav1_local_pose.coordinate_frame = 1;
				uav1_local_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_local_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_local_pose.position.z = uav13_height;

				uav2_local_pose.type_mask = 0b100111111000;
				uav2_local_pose.coordinate_frame = 1;
				uav2_local_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_local_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_local_pose.position.z = uav24_height;

				uav3_local_pose.type_mask = 0b100111111000;
				uav3_local_pose.coordinate_frame = 1;
				uav3_local_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_local_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_local_pose.position.z = uav13_height;

				uav4_local_pose.type_mask = 0b100111111000;
				uav4_local_pose.coordinate_frame = 1;
				uav4_local_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_local_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_local_pose.position.z = uav24_height;

				while(ros::ok())
				{
					uav1_local_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_local_pose);
					uav2_local_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_local_pose);
					uav3_local_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_local_pose);
					uav4_local_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_local_pose);
					count++;
					if(count >= num)
					{
						stage = 2;
						break;
					}
					r.sleep();
				}
			}
			
			if(stage == 2)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_local_pose.type_mask = 0b100111111000;
				uav1_local_pose.coordinate_frame = 1;
				uav1_local_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_local_pose.position.y = -square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_local_pose.position.z = uav13_height;

				uav2_local_pose.type_mask = 0b100111111000;
				uav2_local_pose.coordinate_frame = 1;
				uav2_local_pose.position.x = -square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_local_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_local_pose.position.z = uav24_height;

				uav3_local_pose.type_mask = 0b100111111000;
				uav3_local_pose.coordinate_frame = 1;
				uav3_local_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_local_pose.position.y = square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_local_pose.position.z = uav13_height;

				uav4_local_pose.type_mask = 0b100111111000;
				uav4_local_pose.coordinate_frame = 1;
				uav4_local_pose.position.x = square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_local_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_local_pose.position.z = uav24_height;

				while(ros::ok())
				{
					uav1_local_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_local_pose);
					uav2_local_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_local_pose);
					uav3_local_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_local_pose);
					uav4_local_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_local_pose);
					count++;
					if(count >= num)
					{
						stage = 3;
						break;
					}
					r.sleep();
				}
			}

			if(stage == 3)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_local_pose.type_mask = 0b100111111000;
				uav1_local_pose.coordinate_frame = 1;
				uav1_local_pose.position.x = -square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_local_pose.position.y = -square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_local_pose.position.z = uav13_height;

				uav2_local_pose.type_mask = 0b100111111000;
				uav2_local_pose.coordinate_frame = 1;
				uav2_local_pose.position.x = -square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_local_pose.position.y = square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_local_pose.position.z = uav24_height;

				uav3_local_pose.type_mask = 0b100111111000;
				uav3_local_pose.coordinate_frame = 1;
				uav3_local_pose.position.x = square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_local_pose.position.y = square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_local_pose.position.z = uav13_height;

				uav4_local_pose.type_mask = 0b100111111000;
				uav4_local_pose.coordinate_frame = 1;
				uav4_local_pose.position.x = square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_local_pose.position.y = -square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_local_pose.position.z = uav24_height;

				while(ros::ok())
				{
					uav1_local_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_local_pose);
					uav2_local_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_local_pose);
					uav3_local_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_local_pose);
					uav4_local_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_local_pose);
					count++;
					if(count >= num)
					{
						stage = 4;
						break;
					}
					r.sleep();
				}
			}

			if(stage == 4)
			{
				int count = 0;
				int num = hold_time * 10;
				uav1_local_pose.type_mask = 0b100111111000;
				uav1_local_pose.coordinate_frame = 1;
				uav1_local_pose.position.x = -square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_local_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_local_pose.position.z = uav13_height;

				uav2_local_pose.type_mask = 0b100111111000;
				uav2_local_pose.coordinate_frame = 1;
				uav2_local_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_local_pose.position.y = square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_local_pose.position.z = uav24_height;

				uav3_local_pose.type_mask = 0b100111111000;
				uav3_local_pose.coordinate_frame = 1;
				uav3_local_pose.position.x = square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_local_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_local_pose.position.z = uav13_height;

				uav4_local_pose.type_mask = 0b100111111000;
				uav4_local_pose.coordinate_frame = 1;
				uav4_local_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_local_pose.position.y = -square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_local_pose.position.z = uav24_height;

				while(ros::ok())
				{
					uav1_local_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_local_pose);
					uav2_local_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_local_pose);
					uav3_local_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_local_pose);
					uav4_local_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_local_pose);
					count++;
					if(count >= num)
					{
						stage = 1;
						break;
					}
					r.sleep();
				}
			}
			
			if(stage == 1)
			{
				uav1_local_pose.type_mask = 0b100111111000;
				uav1_local_pose.coordinate_frame = 1;
				uav1_local_pose.position.x = square_length/2 - uav1_gazebo_offset_pose[0];
				uav1_local_pose.position.y = square_length/2 - uav1_gazebo_offset_pose[1];
				uav1_local_pose.position.z = uav13_height;

				uav2_local_pose.type_mask = 0b100111111000;
				uav2_local_pose.coordinate_frame = 1;
				uav2_local_pose.position.x = square_length/2 - uav2_gazebo_offset_pose[0];
				uav2_local_pose.position.y = -square_length/2 - uav2_gazebo_offset_pose[1];
				uav2_local_pose.position.z = uav24_height;

				uav3_local_pose.type_mask = 0b100111111000;
				uav3_local_pose.coordinate_frame = 1;
				uav3_local_pose.position.x = -square_length/2 - uav3_gazebo_offset_pose[0];
				uav3_local_pose.position.y = -square_length/2 - uav3_gazebo_offset_pose[1];
				uav3_local_pose.position.z = uav13_height;

				uav4_local_pose.type_mask = 0b100111111000;
				uav4_local_pose.coordinate_frame = 1;
				uav4_local_pose.position.x = -square_length/2 - uav4_gazebo_offset_pose[0];
				uav4_local_pose.position.y = square_length/2 - uav4_gazebo_offset_pose[1];
				uav4_local_pose.position.z = uav24_height;

				while(ros::ok())
				{
					uav1_local_pose.header.stamp = ros::Time::now();
					uav1_local_pub.publish(uav1_local_pose);
					uav2_local_pose.header.stamp = ros::Time::now();
					uav2_local_pub.publish(uav2_local_pose);
					uav3_local_pose.header.stamp = ros::Time::now();
					uav3_local_pub.publish(uav3_local_pose);
					uav4_local_pose.header.stamp = ros::Time::now();
					uav4_local_pub.publish(uav4_local_pose);
					r.sleep();
				}
			}
		}
		else
		{
			ROS_WARN("input error, please input again");
		}
	}
	
}



































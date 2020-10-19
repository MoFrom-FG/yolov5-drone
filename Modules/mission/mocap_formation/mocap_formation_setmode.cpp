/*******************************************************************
 * 文件名:mocap_formation_setmode.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中模式切换相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Mocap_Formation.h"
#include <unistd.h>
#include <iostream>

void mocap_formation::init()
{
    //获取相关参数
    ros::param::param<int>("~OFFBOARD_intervals", offboard_intervals, 3);
    ros::param::param<int>("~LAND_intervals", land_intervals, 3);

    //创建1~5号机的模式切换客户端
    uav1_mode_client = n.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    uav2_mode_client = n.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");
    uav3_mode_client = n.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");
    uav4_mode_client = n.serviceClient<mavros_msgs::SetMode>("/uav4/mavros/set_mode");
    uav5_mode_client = n.serviceClient<mavros_msgs::SetMode>("/uav5/mavros/set_mode");

    uav1_arm_client = n.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    uav2_arm_client = n.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    uav3_arm_client = n.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    uav4_arm_client = n.serviceClient<mavros_msgs::CommandBool>("/uav4/mavros/cmd/arming");
    uav5_arm_client = n.serviceClient<mavros_msgs::CommandBool>("/uav5/mavros/cmd/arming");
}

void mocap_formation::is_wait(int time)
{
    //判断是否需要等待,time变量不为0,则等待
    if(time != 0)
    {
        sleep(time);
    }

}

void mocap_formation::set_formation_offboard()
{
    //设置1~5号机模式变量为offboard,解上锁变量为解锁
    uav1_mode.request.custom_mode = "OFFBOARD";
    uav2_mode.request.custom_mode = "OFFBOARD";
    uav3_mode.request.custom_mode = "OFFBOARD";
    uav4_mode.request.custom_mode = "OFFBOARD";
    uav5_mode.request.custom_mode = "OFFBOARD";

    uav1_arm.request.value = true;
    uav2_arm.request.value = true;
    uav3_arm.request.value = true;
    uav4_arm.request.value = true;
    uav5_arm.request.value = true;

    //集群按照23145的顺序对五台无人机分别解锁并切入offboard模式
    //当有一台无人机解锁或者切入offboard模式失败,该函数返回false
    //五台无人机成功解锁并切入offboard模式,该函数返回true
    if(uav2_arm_client.call(uav2_arm) && uav2_mode_client.call(uav2_mode))
    {
        ROS_INFO("uav2 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav2 armed and set offboard mode failed");
    }

    if(uav3_arm_client.call(uav2_arm) && uav3_mode_client.call(uav3_mode))
    {
        ROS_INFO("uav3 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav3 armed and set offboard mode failed");
    }

    if(uav1_arm_client.call(uav1_arm) && uav1_mode_client.call(uav1_mode))
    {
        ROS_INFO("uav1 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav1 armed and set offboard mode failed");
    }

    if(uav4_arm_client.call(uav4_arm) && uav4_mode_client.call(uav4_mode))
    {
        ROS_INFO("uav4 armed and set offboard mode success");
        is_wait(offboard_intervals);
    }
    else
    {
        ROS_ERROR("uav4 armed and set offboard mode failed");
    }

    if(uav5_arm_client.call(uav5_arm) && uav5_mode_client.call(uav5_mode))
    {
        ROS_INFO("uav5 armed and set offboard mode success");
    }
    else
    {
        ROS_ERROR("uav5 armed and set offboard mode failed");
    }
}

void mocap_formation::set_formation_land()
{
    //设置1~5号机模式变量为land
    uav1_mode.request.custom_mode = "AUTO.LAND";
    uav2_mode.request.custom_mode = "AUTO.LAND";
    uav3_mode.request.custom_mode = "AUTO.LAND";
    uav4_mode.request.custom_mode = "AUTO.LAND";
    uav5_mode.request.custom_mode = "AUTO.LAND";

    //切换为land模式,并对结果进行打印
    if(uav2_mode_client.call(uav2_mode))
    {
        ROS_INFO("uav2 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav2 set land mode failed");
    }

    if(uav3_mode_client.call(uav3_mode))
    {
        ROS_INFO("uav3 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav3 set land mode failed");
    }
    
    if(uav1_mode_client.call(uav1_mode))
    {
        ROS_INFO("uav1 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav1 set land mode failed");
    }

    if(uav4_mode_client.call(uav4_mode))
    {
        ROS_INFO("uav4 set land mode success");
        is_wait(land_intervals);
    }
    else
    {
        ROS_ERROR("uav4 set land mode success");
    }

    if(uav5_mode_client.call(uav5_mode))
    {
        ROS_INFO("uav5 set land mode success");
    }
    else
    {
        ROS_ERROR("uav5 set land mode failed");
    }
}

void mocap_formation::set_formation_disarmed()
{
    //设置1~5号机解上锁变量为false
    uav1_arm.request.value = false;
    uav2_arm.request.value = false;
    uav3_arm.request.value = false;
    uav4_arm.request.value = false;
    uav5_arm.request.value = false;

    //按照23145的顺序调用上锁服务,并根据结果打印相关提示信息
    if(uav2_arm_client.call(uav2_arm))
    {
        ROS_INFO("uav2 disarmed success");
    }
    else
    {
        ROS_ERROR("uav2 disarmed failed");
    }

    if(uav3_arm_client.call(uav3_arm))
    {
        ROS_INFO("uav3 disarmed success");
    }
    else
    {
        ROS_ERROR("uav3 disarmed failed");
    }
    
    if(uav1_arm_client.call(uav1_arm))
    {
        ROS_INFO("uav1 disarmed success");
    }
    else
    {
        ROS_ERROR("uav1 disarmed failed");
    }

    if(uav4_arm_client.call(uav4_arm))
    {
        ROS_INFO("uav4 disarmed success");
    }
    else
    {
        ROS_ERROR("uav4 disarmed failed");
    }

    if(uav5_arm_client.call(uav5_arm))
    {
        ROS_INFO("uav5 disarmed success");
    }
    else
    {
        ROS_ERROR("uav5 disarmed failed");
    }
}

void mocap_formation::set_mode()
{
    //初始化,创建服务调用后客户端以及获取参数
    init();
    while(ros::ok())
    {
        //创建变量用以获取用户输入的值
        int mode;
        //打印提示信息
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< " << std::endl;
        std::cout << "Input the drone state:  0 for armed and offboard, 1 for land, 2 for disarmed" << std::endl;
        //获取用户输入的值
        std::cin >> mode;
        //判断值是否正确,正确则进入正常流程,错误则打印警告信息
        if((mode >= 0) && (mode <= 2))
        {
            formation_mode = mode;
            switch(formation_mode)
            {
                //offboard模式
                case 0:
                    set_formation_offboard();
                    break;
                
                //land模式
                case 1:
                    set_formation_land();
                    break;

                //上锁
                case 2:
                    set_formation_disarmed();
                    break;
            }
        }
        else
        {
            //输入错误
            ROS_WARN("input error,please input again");
        }
        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_set_mode");
    mocap_formation formation;
    formation.set_mode();
    return 0;
}
/*******************************************************************
 * 文件名:mocap_formation_control.cpp
 * 
 * 作者: BOSHEN97
 * 
 * 更新时间: 2020.10.14
 * 
 * 介绍:该cpp文件主要为动捕集群中集群控制相关函数的实现以及程序的运行
 * ****************************************************************/
#include "Mocap_Formation.h"
#include <unistd.h>

void mocap_formation::init()
{
    //控制命令数据订阅者
    cmd_sub = n.subscribe("/prometheus/control_command", 10, &mocap_formation::ControlCallBack, this);
    //集群队形数据订阅者
    formation_type_sub = n.subscribe("/prometheus/formation/change", 10, &mocap_formation::FormationChangeCallBack, this);

    //集群五台飞机状态订阅回调函数
    uav1_state_sub = n.subscribe("/uav1/mavros/state", 10, &mocap_formation::Uav1StateCallBack, this);
    uav2_state_sub = n.subscribe("/uav2/mavros/state", 10, &mocap_formation::Uav2StateCallBack, this);
    uav3_state_sub = n.subscribe("/uav3/mavros/state", 10, &mocap_formation::Uav3StateCallBack, this);
    uav4_state_sub = n.subscribe("/uav4/mavros/state", 10, &mocap_formation::Uav4StateCallBack, this);
    uav5_state_sub = n.subscribe("/uav5/mavros/state", 10, &mocap_formation::Uav5StateCallBack, this);

    uav1_pose_sub = n.subscribe("/uav1/mavros/local_position/pose", 10, &mocap_formation::Uav1PoseCallBack, this);
    uav2_pose_sub = n.subscribe("/uav2/mavros/local_position/pose", 10, &mocap_formation::Uav2PoseCallBack, this);
    uav3_pose_sub = n.subscribe("/uav3/mavros/local_position/pose", 10, &mocap_formation::Uav3PoseCallBack, this);
    uav4_pose_sub = n.subscribe("/uav4/mavros/local_position/pose", 10, &mocap_formation::Uav4PoseCallBack, this);
    uav5_pose_sub = n.subscribe("/uav5/mavros/local_position/pose", 10, &mocap_formation::Uav5PoseCallBack, this);


    //集群五台飞机位置控制数据发布者
    uav1_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
    uav2_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);
    uav3_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);
    uav4_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav4/mavros/setpoint_raw/local", 10);
    uav5_local_pub = n.advertise<mavros_msgs::PositionTarget>("/uav5/mavros/setpoint_raw/local", 10);

    //获取集群X轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_x", formation_distance_x, 1);

    //获取集群Y轴间隔距离参数
    ros::param::param<double>("~FORMATION_DISTANCE_y", formation_distance_y, 2);

    //获取仿真中位置差值
    ros::param::param<double>("~uav1_x", uav1_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav1_y", uav1_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav2_x", uav2_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav2_y", uav2_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav3_x", uav3_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav3_y", uav3_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav4_x", uav4_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav4_y", uav4_gazebo_offset_pose[1], 0);

    ros::param::param<double>("~uav5_x", uav5_gazebo_offset_pose[0], 0);
    ros::param::param<double>("~uav5_y", uav5_gazebo_offset_pose[1], 0);

    //获取是否为仿真参数
    ros::param::param<bool>("~sim",sim,false);

    //初始队形设置为一字形
    formation_data.type = prometheus_msgs::Formation::HORIZONTAL;

    //设置程序初始时间
    begin_time = ros::Time::now();
}

void mocap_formation::is_sim()
{
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

        uav5_gazebo_offset_pose[0] = 0;
        uav5_gazebo_offset_pose[1] = 0;
    }
}

//设置一字队形
void mocap_formation::set_horizontal()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 0 - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = 0 - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;

}

//设置三角队形
void mocap_formation::set_triangle()
{
    //1号机
    uav1_offset_pose[0] = 2 * formation_distance_x - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//设置菱形队形
void mocap_formation::set_diamond()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 2 * formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = 0 - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = -2 * formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = 0 - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//设置菱形队形相关过渡队形
void mocap_formation::set_diamond_stage1()
{
    //1号机
    uav1_offset_pose[0] = 0 - uav1_gazebo_offset_pose[0];
    uav1_offset_pose[1] = 0 - uav1_gazebo_offset_pose[1];
    uav1_offset_pose[2] = 0;

    //2号机
    uav2_offset_pose[0] = 0 - uav2_gazebo_offset_pose[0];
    uav2_offset_pose[1] = 2 * formation_distance_y - uav2_gazebo_offset_pose[1];
    uav2_offset_pose[2] = 0;

    //3号机
    uav3_offset_pose[0] = 2 * formation_distance_x - uav3_gazebo_offset_pose[0];
    uav3_offset_pose[1] = formation_distance_y - uav3_gazebo_offset_pose[1];
    uav3_offset_pose[2] = 0;

    //4号机
    uav4_offset_pose[0] = -2 * formation_distance_x - uav4_gazebo_offset_pose[0];
    uav4_offset_pose[1] = -formation_distance_y - uav4_gazebo_offset_pose[1];
    uav4_offset_pose[2] = 0;

    //5号机
    uav5_offset_pose[0] = 0 - uav5_gazebo_offset_pose[0];
    uav5_offset_pose[1] = -2 * formation_distance_y - uav5_gazebo_offset_pose[1];
    uav5_offset_pose[2] = 0;
}

//集群位置控制发布函数
void mocap_formation::formation_pos_pub()
{
    uav1_local_pub.publish(uav1_desired_pose);
    uav2_local_pub.publish(uav2_desired_pose);
    uav3_local_pub.publish(uav3_desired_pose);
    uav4_local_pub.publish(uav4_desired_pose);
    uav5_local_pub.publish(uav5_desired_pose);
}

//获取单台无人机控制数据
void mocap_formation::get_uav_cmd(Eigen::Vector3d offset_pose, mavros_msgs::PositionTarget& desired_pose)
{
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.type_mask = 0b100111111000;
    desired_pose.coordinate_frame = 1;
    desired_pose.position.x = control_data.Reference_State.position_ref[0] + offset_pose[0];
    desired_pose.position.y = control_data.Reference_State.position_ref[1] + offset_pose[1];
    desired_pose.position.z = control_data.Reference_State.position_ref[2] + offset_pose[2];
    desired_pose.yaw = control_data.Reference_State.yaw_ref;
}

//获取1号机位置数据回调函数
void mocap_formation::Uav1PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav1_current_pose = *pose_msgs;
}

//获取2号机位置数据回调函数
void mocap_formation::Uav2PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav2_current_pose = *pose_msgs;
}

//获取3号机位置数据回调函数
void mocap_formation::Uav3PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav3_current_pose = *pose_msgs;
}

//获取4号机位置数据回调函数
void mocap_formation::Uav4PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav4_current_pose = *pose_msgs;
}

//获取5号机位置数据回调函数
void mocap_formation::Uav5PoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose_msgs)
{
    uav5_current_pose = *pose_msgs;
}

//打印无人机状态函数
void mocap_formation::printf_formation_state()
{
    //固定的浮点显示
    std::cout.setf(std::ios::fixed);
    //设置小数点后精度为2位
    std::cout<<std::setprecision(2);
    //左对齐
    std::cout.setf(std::ios::left);
    //显示小数点
    std::cout.setf(std::ios::showpoint);
    //强制显示符号
    std::cout.setf(std::ios::showpos);
    //打印时间戳
    std::cout << "Time :  " << ros::Time::now().sec - begin_time.sec << std::endl;
    //1号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV1 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav1_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav1_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //1号机当前模式
    std::cout << " [ " << uav1_state.mode << " ] "  << std::endl;
    //1号机当前位置
    std::cout << "Position_uav1 [X Y Z]: " << uav1_current_pose.pose.position.x << "[ m ]" << uav1_current_pose.pose.position.y << "[ m ]" << uav1_current_pose.pose.position.z << "[ m ]" << std::endl;

    //2号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV2 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav2_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav2_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //2号机当前模式
    std::cout << " [ " << uav2_state.mode << " ] "  << std::endl;
    //2号机当前位置
    std::cout << "Position_uav2 [X Y Z]: " << uav2_current_pose.pose.position.x << "[ m ]" << uav2_current_pose.pose.position.y << "[ m ]" << uav2_current_pose.pose.position.z << "[ m ]" << std::endl;

    //3号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV3 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav3_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav3_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //3号机当前模式
    std::cout << " [ " << uav3_state.mode << " ] "  << std::endl;
    //3号机当前位置
    std::cout << "Position_uav3 [X Y Z]: " << uav3_current_pose.pose.position.x << "[ m ]" << uav3_current_pose.pose.position.y << "[ m ]" << uav3_current_pose.pose.position.z << "[ m ]" << std::endl;

    //4号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV4 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav4_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav4_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //4号机当前模式
    std::cout << " [ " << uav4_state.mode << " ] "  << std::endl;
    //4号机当前位置
    std::cout << "Position_uav4 [X Y Z]: " << uav4_current_pose.pose.position.x << "[ m ]" << uav4_current_pose.pose.position.y << "[ m ]" << uav4_current_pose.pose.position.z << "[ m ]" << std::endl;

    //5号机
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV5 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    //是否与飞控连接
    if(uav5_state.connected == true)
    {
        std::cout << "  [Connected]  ";
    }
    else
    {
        std::cout << "  [Unconnected]  ";
    }
    //是否上锁
    if(uav5_state.armed == true)
    {
        std::cout << "  [ Armed ]  ";
    }
    else
    {
        std::cout << "  [ DisArmed ]  ";
    }

    //5号机当前模式
    std::cout << " [ " << uav5_state.mode << " ] "  << std::endl;
    //5号机当前位置
    std::cout << "Position_uav5 [X Y Z]: " << uav5_current_pose.pose.position.x << "[ m ]" << uav5_current_pose.pose.position.y << "[ m ]" << uav5_current_pose.pose.position.z << "[ m ]" << std::endl;
}

//集群控制函数
void mocap_formation::control()
{
    //初始化
    init();
    is_sim();
    while(ros::ok())
    {
        //处理回调函数
        ros::spinOnce();
        //获取集群队形
        switch(formation_data.type)
        {
            //设置为一字队形
            case prometheus_msgs::Formation::HORIZONTAL:
                set_horizontal();
                break;
        
            //设置为三角队形
            case prometheus_msgs::Formation::TRIANGEL:
                set_triangle();
                break;

            //设置为菱形队形过渡队形
            case prometheus_msgs::Formation::DIAMOND_STAGE_1:
                set_diamond_stage1();
                break;

            //设置为菱形队形
            case prometheus_msgs::Formation::DIAMOND:
                set_diamond();
                break;
        }
        //五台无人机获取控制数据
        get_uav_cmd(uav1_offset_pose, uav1_desired_pose);
        get_uav_cmd(uav2_offset_pose, uav2_desired_pose);
        get_uav_cmd(uav3_offset_pose, uav3_desired_pose);
        get_uav_cmd(uav4_offset_pose, uav4_desired_pose);
        get_uav_cmd(uav5_offset_pose, uav5_desired_pose);
        //发布集群控制
        formation_pos_pub();
        //打印无人机相关信息
        printf_formation_state();
        //等待0.1秒
        usleep(100000);
    }
    

}

//获取无人机控制指令
void mocap_formation::ControlCallBack(const prometheus_msgs::ControlCommandConstPtr& control_msgs)
{
    control_data = *control_msgs;
}

//获取无人机集群队形指令
void mocap_formation::FormationChangeCallBack(const prometheus_msgs::FormationConstPtr& change_msgs)
{
    formation_data.type = change_msgs->type;
}


//集群五台无人机状态读取

//1号机
void mocap_formation::Uav1StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav1_state = *state_msgs;
}

//2号机
void mocap_formation::Uav2StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav2_state = *state_msgs;
}

//3号机
void mocap_formation::Uav3StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav3_state = *state_msgs;
}

//4号机
void mocap_formation::Uav4StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav4_state = *state_msgs;
}

//5号机
void mocap_formation::Uav5StateCallBack(const mavros_msgs::StateConstPtr &state_msgs)
{
    uav5_state = *state_msgs;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_control");
    mocap_formation formation;
    formation.control();
    return 0;
}
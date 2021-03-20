/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <waterplus_map_tools/Arrived.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    //初始化ROS节点，参数列表来自命令行
    ros::init(argc, argv, "wp_nav_test");

    //创建节点时，会自动调用ros::start()
    ros::NodeHandle nh;
    //创建3个客户端，
    ros::ServiceClient cliGetNum = nh.serviceClient<waterplus_map_tools::GetNumOfWaypoints>("/waterplus/get_num_waypoint");
    ros::ServiceClient cliGetWPIndex = nh.serviceClient<waterplus_map_tools::GetWaypointByIndex>("/waterplus/get_waypoint_index");
    ros::ServiceClient cliGetWPName = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");


    //新添加
    ros::ServiceClient cliArrived = nh.serviceClient<waterplus_map_tools::Arrived>("arrived");
    //实例化一个服务的请求与响应的对象
    waterplus_map_tools::Arrived srvArrived;
    //填入请求的信息
    int nowPoint = 1;
    



    //写一个消息节点，发布到达目标点后的消息(收到消息发1，否则一直发0)
    //一旦红外收到消息，反馈回来，导航发布坐标的节点就中断掉,不进入while循环导航点
    //然后红外就正常的运行下去。

    ///////////////////////////////////////////////////////////////////////////////////
    //打印：获取目标点的数量。
    waterplus_map_tools::GetNumOfWaypoints srvNum;
    if (cliGetNum.call(srvNum))
    {
        ROS_INFO("Num_wp = %d", (int)srvNum.response.num);
    }
    else
    {
        ROS_ERROR("Failed to call service get_num_waypoints");
    }
    //打印：目标点的各个x、y坐标。
    waterplus_map_tools::GetWaypointByIndex srvI;
    for(int i=0;i<srvNum.response.num;i++)
    {
        srvI.request.index = i;
        if (cliGetWPIndex.call(srvI))
        {
            std::string name = srvI.response.name;
            float x = srvI.response.pose.position.x;
            float y = srvI.response.pose.position.y;
            ROS_INFO("Get_wp_index: name = %s (%.2f,%.2f)", name.c_str(),x,y);
        }
        else
        {
            ROS_ERROR("Failed to call service get_wp_index");
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////
    // 打印：根据目标点的名字来反馈要去的目标点
    // waterplus_map_tools::GetWaypointByName srvN;
    // for(int i=0;i<10;i++)
    // {
    //     std::ostringstream stringStream;
    //     stringStream << i;
    //     std::string wp_index = stringStream.str();
    //     srvN.request.name = wp_index;
    //     if (cliGetWPName.call(srvN))
    //     {
    //         std::string name = srvN.response.name;
    //         float x = srvN.response.pose.position.x;
    //         float y = srvN.response.pose.position.y;
    //         ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", wp_index.c_str(),x,y);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to call service get_waypoint_name");
    //     }
    // }
    ////////////////////////////////////////////////////////////////////////////////////

    //获取move_base的开启情况，要是没有开启，进入循环，阻隔运行巡航目标点。
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        //这个操作能关闭这个节点，直接退出这个程序的所有运行
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int nWPIndex = 0;//记录当前要去或者已经到了的目标点的代表。
    int nNumOfWaypoints = 0;//记录所有的目标点。
    move_base_msgs::MoveBaseGoal goal;//发布目标点的结构体。
    

    //一旦ros::ok() 返回false，节点就已经关闭。反过来，代表节点一直在运行。
    while(ros::ok())
    {
        srvArrived.request.index = nowPoint;
        if (cliArrived.call(srvArrived))
        {
            ROS_INFO("client connect success.");
            if (srvArrived.response.result)
            {
                ROS_INFO("True!!!");
            }
            else {
                ROS_INFO("False!!!");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service arrived");
            break;
        }

        //获取标定的目标点的数量，并打印出来。
        waterplus_map_tools::GetNumOfWaypoints srvNum;
        if (cliGetNum.call(srvNum))
        {
            ROS_INFO("Num_wp = %ld", (long int)srvNum.response.num);
            nNumOfWaypoints = (int)srvNum.response.num;
        }
        else
        {
            ROS_ERROR("Failed to call service get_num_waypoint");
            break;
        }

        //如果遍历了所有的目标点，计数清零，重来。
        if(nWPIndex >= nNumOfWaypoints)
        {
            nWPIndex = 0;
            continue;
        }

        //打印出具体的目标点的x、y的坐标
        waterplus_map_tools::GetWaypointByIndex srvI;
        srvI.request.index = nWPIndex;

        if (cliGetWPIndex.call(srvI))
        {
            std::string name = srvI.response.name;
            float x = srvI.response.pose.position.x;
            float y = srvI.response.pose.position.y;
            ROS_INFO("Get_wp_index: name = %s (%.2f,%.2f)", name.c_str(),x,y);
        }
        else
        {
            ROS_ERROR("Failed to call service get_wp_index");
        }
        //发布动作，去往目标点
        ROS_INFO("Go to the WayPoint[%d]",nWPIndex);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = srvI.response.pose;
        ac.sendGoal(goal);
        ac.waitForResult();

        //到达目标点后输出打印信息，反馈到达的情况。
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arrived at WayPoint[%d] !",nWPIndex);
            nWPIndex ++;
        }
        else
            ROS_INFO("Failed to get to WayPoint[%d] ...",nWPIndex );

        
    }

    return 0;
}
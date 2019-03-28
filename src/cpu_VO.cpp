/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include <unistd.h>
#include "std_msgs/MultiArrayDimension.h"
#include "vonetvlad/my_image.h"
#include "vonetvlad/my_data.h"
#include "vonetvlad/Param.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <array>

#define H (240)
#define W (640)
#define C (3)
#define rH (14)
#define rW (14)
#define rC (512)

class cpu_VO{
public:
    cpu_VO(){
        cpu_vo_sub = n.subscribe("dpu_VO", 5, &cpu_VO::cpuVOPub, this);
        cpu_vo_pub = n.advertise<std_msgs::String>("cpu_VO", 3);
    }
    void cpuVOPub(const vonetvlad::my_data::ConstPtr& msg);

private: 
    ros::NodeHandle n;
    ros::Subscriber cpu_vo_sub;
    ros::Publisher cpu_vo_pub;
};


void cpu_VO::cpuVOPub(const vonetvlad::my_data::ConstPtr& msg)
{
    ROS_INFO("CPU VO heard: [%s]", msg->ID.c_str());
    int tmp;

    ros::NodeHandle n;
    ros::ServiceClient client1, client2;
    client1 = n.serviceClient<vonetvlad::Param>("ParamManager");
    vonetvlad::Param srv;
    srv.request.name = "cpu_VO_begin";
    // Debug
    ROS_INFO("############# %f ##################", msg->data.at(10));
    
    // n.param<int>("running_cpu_VO",tmp,0);
    // 修改参数
    client1.call(srv);
    // doing computation
    // n.setParam("running_cpu_VO", 1);
    // starting time
    ros::Time start = ros::Time::now();
    usleep(10*1000);
    // finishing time
    ROS_INFO_STREAM("image ID: " <<msg->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());
    std::stringstream ss;
    std_msgs::String msg_pub;
    ss << "  CPU VO result: " << "Not Completed";
    msg_pub.data = ss.str();
    cpu_vo_pub.publish(msg_pub);
    client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
    vonetvlad::Param srv2;
    srv2.request.name = "cpu_VO_end";
    // 修改参数
    client2.call(srv2);
    //n.setParam("running_cpu_VO", 0);

}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "cpu_VO");
    cpu_VO cpu_VO_inst;
    ros::Rate loop_rate(50);

    while (ros::ok()){
        // while()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

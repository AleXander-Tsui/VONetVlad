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


class dpu_NetVLAD{
public:
    dpu_NetVLAD(){
        dpu_netvald_sub = n.subscribe("frame", 1, &dpu_NetVLAD::callbackThread,this);
        dpu_netvald_pub = n.advertise<vonetvlad::my_data>("dpu_NetVLAD", 3);
        recent_doing = 0;
        count_since_last = 0;
    }
    void callbackThread(const vonetvlad::my_image::ConstPtr& msg);

private: 
    ros::NodeHandle n;
    ros::Subscriber dpu_netvald_sub;
    ros::Publisher dpu_netvald_pub;
    int recent_doing;
    int count_since_last;
};

void dpu_NetVLAD::callbackThread(const vonetvlad::my_image::ConstPtr& msg)
{   
    
    int tmp;    
    // n.param<int>("running_dpu_NetVLAD",tmp,0);

    int running_dpu_VO;
    int depth_dpu_VO;
    int depth_cpu_NetVLAD;

    n.param<int>("running_dpu_VO", running_dpu_VO, 0);
    n.param<int>("depth_dpu_VO", depth_dpu_VO, 0);
    n.param<int>("depth_cpu_NetVLAD", depth_cpu_NetVLAD, 0);
    ROS_INFO_STREAM("depth_cpu_NetVLAD: " << depth_cpu_NetVLAD << "; depth_dpu_VO: " << depth_dpu_VO << "; running_dpu_VO: " << running_dpu_VO << "; recent_doing: " << recent_doing);
       
    if ( (running_dpu_VO > 0) ||  depth_dpu_VO > 1 || depth_cpu_NetVLAD > 1 || recent_doing>0){
        if( recent_doing){
            count_since_last += 1;
            if ( depth_cpu_NetVLAD ==0 || count_since_last > 5){
                recent_doing = 0;
            }
        }
        return;
    }
    else{
        recent_doing = 1;
        count_since_last = 0;
        n.setParam("running_dpu_NetVLAD", 1);
        ROS_INFO("DPU NetVLAD heard: [%s]", msg->ID.c_str());
        // Debug
        ROS_INFO("############# %d ##################", msg->data.at(10));

        // starting time
        ros::Time start = ros::Time::now();
        // doing computation
        usleep(66*1000);
        // finishing time
        ROS_INFO_STREAM("image ID: " << msg->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());
        
        // 将计算结果转换为自定义消息"my_data"发布
        // 定义自定义消息
        vonetvlad::my_data dat;
        std::stringstream ss;
        ss << msg->ID;
        dat.ID = ss.str();
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim[0].label = "height";
        dat.layout.dim[1].label = "width";
        dat.layout.dim[2].label = "channal";
        dat.layout.dim[0].size = rH;
        dat.layout.dim[1].size = rW;
        dat.layout.dim[2].size = rC;
        dat.layout.dim[0].stride = rH*rW*rC;
        dat.layout.dim[1].stride = rW*rC;
        dat.layout.dim[2].stride = rC;
        dat.layout.data_offset = 0;
        std::vector<float> vec(rW*rH*rC, 0);
        for (int i=0; i<rH; i++)
            for (int j=0; j<rW; j++)
                for (int z=0; z<rC; z++)
                    vec[i*rW*rC + j*rC + z] = rand() % 10 + 1 ;
        dat.data = vec;
        /*std::stringstream ss;
        std_msgs::String msg_pub;
        ss << " DPU NetVLAD result: " << msg->data.c_str();
        msg_pub.data = ss.str();*/
        dpu_netvald_pub.publish(dat);
        n.setParam("running_dpu_NetVLAD", 0);
    }
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "dpu_NetVLAD");
    dpu_NetVLAD dpu_NetVLAD_inst;
    ros::Rate loop_rate(50);
    
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

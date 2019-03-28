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


class cpu_NetVLAD{
public:
    cpu_NetVLAD(){
        cpu_netvlad_sub = n.subscribe("dpu_NetVLAD", 3, &cpu_NetVLAD::push2queue, this);
    }
    static void cpuNetVLADPub(void* __this);
    void push2queue(const vonetvlad::my_data::ConstPtr& msg);
    std::thread cpu_thread;
    void startcputhread(){
        cpu_thread = std::thread(cpuNetVLADPub, this);
    }
    std::queue <vonetvlad::my_data::ConstPtr> dpu_netvlad_queue;

private: 
    ros::NodeHandle n;
    ros::Subscriber cpu_netvlad_sub;
};

void cpu_NetVLAD::push2queue(const vonetvlad::my_data::ConstPtr& msg){
    // ROS_INFO("cpu_NetVLAD heard: [%s]", msg->data.c_str());   // 塞入队列
    dpu_netvlad_queue.push(msg);
}

void cpu_NetVLAD::cpuNetVLADPub(void* __this)
{
    cpu_NetVLAD* _this = (cpu_NetVLAD*)  __this;
    ros::NodeHandle n;
    ros::Publisher cpu_netvlad_pub = n.advertise<std_msgs::String>("cpu_NetVLAD", 3);;
    ros::Rate loop_rate2(50);
    int tmp;
    // n.param<int>("running_cpu_NetVLAD",tmp,0);
    // n.param<int>("depth_cpu_NetVLAD",tmp,0);
    ros::ServiceClient client1, client2, client3;
    while (n.ok()){
        if ( (! _this->dpu_netvlad_queue.empty() ) ){
            client1 = n.serviceClient<vonetvlad::Param>("ParamManager");
            vonetvlad::Param srv1;
            srv1.request.name = "cpu_NetVLAD_begin";
            srv1.request.size = int(_this->dpu_netvlad_queue.size() );
            // 修改参数
            client1.call(srv1);
            /*
            n.setParam("running_cpu_NetVLAD", 1);
            n.setParam("depth_cpu_NetVLAD", int(_this->dpu_netvlad_queue.size() ) );*/
            ROS_INFO("CPU NetVLAD heard: [%s]", _this->dpu_netvlad_queue.front()->ID.c_str() );
            // Debug
            ROS_INFO("############# %f ##################", _this->dpu_netvlad_queue.front()->data.at(10));

            // starting time
            ros::Time start = ros::Time::now();
            // doing computation
            usleep(360*1000);
            // finishing time
            ROS_INFO_STREAM("image ID: " <<_this->dpu_netvlad_queue.front()->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());
            std::stringstream ss;
            std_msgs::String msg_pub;
            ss << "  CPU NetVLAD result: " << "Not Completed";
            msg_pub.data = ss.str();
            cpu_netvlad_pub.publish(msg_pub);
            ros::spinOnce();
            _this->dpu_netvlad_queue.pop();
            client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
            vonetvlad::Param srv2;
            srv2.request.name = "cpu_NetVLAD_end";
            srv2.request.size = int(_this->dpu_netvlad_queue.size() );
            // 修改参数
            client2.call(srv2);
            /*n.setParam("running_cpu_NetVLAD", 0);
            n.setParam("depth_cpu_NetVLAD", int(_this->dpu_netvlad_queue.size() ) );*/
        }
        else{
            client3 = n.serviceClient<vonetvlad::Param>("ParamManager");
            vonetvlad::Param srv3;
            srv3.request.name = "cpu_NetVLAD_freeze";
            // 修改参数
            client3.call(srv3);
            /*n.setParam("running_cpu_NetVLAD", 0);  
            n.setParam("depth_cpu_NetVLAD", 0);*/           
            ROS_INFO_STREAM("Queue empty");
            loop_rate2.sleep();
        }
    }


}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "cpu_NetVLAD");
    cpu_NetVLAD cpu_NetVLAD_inst;
    ros::Rate loop_rate(50);
    cpu_NetVLAD_inst.startcputhread();

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    cpu_NetVLAD_inst.cpu_thread.join();
    return 0;
}

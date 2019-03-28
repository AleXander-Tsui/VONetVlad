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

class dpu_VO{
public:
    dpu_VO(){
        sub_frame = n.subscribe("frame", 1, &dpu_VO::push2queue,this);
    }
    void push2queue(const vonetvlad::my_image::ConstPtr& msg);
    static void callbackThread(void* __this);
    void startdputhread(){
        dpu_thread = std::thread(callbackThread, this);
    }
    std::thread dpu_thread;
    std::queue <vonetvlad::my_image::ConstPtr> frame_queue;

private: 
    ros::NodeHandle n;
    ros::Subscriber sub_frame;
};

void dpu_VO::push2queue(const vonetvlad::my_image::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->ID.c_str());
    frame_queue.push(msg);
}

void dpu_VO::callbackThread(void* __this)
{
  dpu_VO * _this =(dpu_VO *)__this;
  ROS_INFO_STREAM("Callback thread id=" << std::this_thread::get_id());

  ros::NodeHandle n;
  int tmp;
  ros::Rate loop_rate2(50);
  // 发布DPU计算后的结果消息
  ros::Publisher dpu_vo_pub = n.advertise<vonetvlad::my_data>("dpu_VO", 3);
  /*int running_dpu_NetVLAD;
  int running_cpu_NetVLAD;
  int depth_cpu_NetVLAD;*/
    // n.param<int>("running_dpu_VO",tmp,0);
    // n.param<int>("depth_dpu_VO",tmp,0);
  ros::ServiceClient client1;
  ros::ServiceClient client2;
  while (n.ok())
  {
    /*n.param<int>("running_dpu_NetVLAD", running_dpu_NetVLAD, 0);
    n.param<int>("running_cpu_NetVLAD", running_cpu_NetVLAD, 0);
    n.param<int>("depth_cpu_NetVLAD",depth_cpu_NetVLAD,0);*/

    client1 = n.serviceClient<vonetvlad::Param>("ParamManager");
    vonetvlad::Param srv;
    srv.request.name = "dpu_VO_begin";
    srv.request.size = _this->frame_queue.size();
    if (client1.call(srv))
    {
        if (srv.response.cond == 0)
        {
            ROS_INFO_STREAM("Queue empty");
            loop_rate2.sleep();
        }
        if (srv.response.cond == 1)
        {
            ROS_INFO_STREAM("Queue depth: " << _this->frame_queue.size() << "; WORD: " << _this->frame_queue.front()->ID.c_str() );
            // starting time
            ros::Time start = ros::Time::now();
            // doing computation
            usleep(3*1000);
            // finishing time
            ROS_INFO_STREAM("image ID: " << _this->frame_queue.front()->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());

            // 将计算结果转换为自定义消息"my_data"发布
            // 定义自定义消息
            vonetvlad::my_data dat;
            std::stringstream ss;
            ss << _this->frame_queue.front()->ID;
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
            std::vector<float> vec2(rW*rH*rC, 0);
            for (int i=0; i<rH; i++)
                for (int j=0; j<rW; j++)
                    for (int z=0; z<rC; z++)
                        vec2[i*rW*rC + j*rC + z] = rand() % 10 + 1 ;
            dat.data = vec2;
            dpu_vo_pub.publish(dat);
            ros::spinOnce();
            _this->frame_queue.pop();
            // Try
            client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
            vonetvlad::Param srv2;
            srv2.request.name = "dpu_VO_end";
            srv2.request.size = _this->frame_queue.size();
            client2.call(srv2);
            ROS_INFO("#############################################");
        }
        if (srv.response.cond == 2)
        {
            loop_rate2.sleep();
        }
    }
    /*if ( (! _this->frame_queue.empty())){
        ROS_INFO_STREAM("running_dpu_NetVLAD: " << running_dpu_NetVLAD << "; depth_cpu_NetVLAD: " << depth_cpu_NetVLAD << "; running_cpu_NetVLAD: " << running_cpu_NetVLAD);
        if ( (! running_dpu_NetVLAD) && ( (depth_cpu_NetVLAD <= 1 && running_cpu_NetVLAD) || (depth_cpu_NetVLAD < 1 && ! running_cpu_NetVLAD) ) ){
            n.setParam("running_dpu_VO", 1);
            n.setParam("depth_dpu_VO", int(_this->frame_queue.size() ) );
            ROS_INFO_STREAM("Queue depth: " << _this->frame_queue.size() << "; WORD: " << _this->frame_queue.front()->ID.c_str() );

            // Debug: check msg
            // 将消息转换为计算输入
            // std::vector<uint8_t> vec1(H*W*C, 0);
            // vec1 = _this->frame_queue.front()->data;
            ROS_INFO("############# %d ##################", _this->frame_queue.front()->data.at(10));
            // starting time
            ros::Time start = ros::Time::now();
            // doing computation
            usleep(3*1000);
            // finishing time
            ROS_INFO_STREAM("image ID: " << _this->frame_queue.front()->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());

            // 将计算结果转换为自定义消息"my_data"发布
            // 定义自定义消息
            vonetvlad::my_data dat;
            std::stringstream ss;
            ss << _this->frame_queue.front()->ID;
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
            std::vector<float> vec2(rW*rH*rC, 0);
            for (int i=0; i<rH; i++)
                for (int j=0; j<rW; j++)
                    for (int z=0; z<rC; z++)
                        vec2[i*rW*rC + j*rC + z] = rand() % 10 + 1 ;
            dat.data = vec2;
            dpu_vo_pub.publish(dat);
            ros::spinOnce();
            _this->frame_queue.pop();
            n.setParam("running_dpu_VO", 0);
            n.setParam("depth_dpu_VO", int(_this->frame_queue.size() ) );
            // loop_rate2.sleep();
        }
        else{
            ROS_INFO_STREAM("Queue not empty, but not start");
            n.setParam("depth_dpu_VO", int(_this->frame_queue.size() ) );
            loop_rate2.sleep();
        }
    }
    else {
        n.setParam("running_dpu_VO", 0);
        n.setParam("depth_dpu_VO", 0 );
        ROS_INFO_STREAM("Queue empty");
        loop_rate2.sleep();
    }
  }*/
  }
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "dpu_VO");
    dpu_VO dpu_VO_inst;
    ros::Rate loop_rate(50);
    dpu_VO_inst.startdputhread();
    
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    dpu_VO_inst.dpu_thread.join();
    return 0;
}

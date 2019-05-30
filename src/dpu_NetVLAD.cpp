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

#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <cv_bridge/cv_bridge.h> 

#include <dnndk/dnndk.h>

#define H (240)
#define W (640)
#define C (3)
#define rH (24)
#define rW (24)
#define rC (512)
#define INPUT_NODE "ConvNdBackward1"
#define OUTPUT_NODE "ConvNdBackward29"
#define length 4096

using namespace std;
//using namespace std::chrono;
using namespace cv;

float* dpuNetVLAD(Mat image){
    DPUKernel *kernel = dpuLoadKernel("netvlad");
    assert(kernel);
    DPUTask *task = dpuCreateTask(kernel, 0);
    resize(image, image, Size(384,384), (0, 0), (0, 0), INTER_LINEAR);
    float mean[3] = {0, 0, 0};
    dpuSetInputImage(task, INPUT_NODE, image, mean);
    cout << "\nRun netvlad ..." << endl;
	dpuRunTask(task);
    long long timeProf = dpuGetTaskProfile(task);
    cout << "  DPU CONV Execution time: " << (timeProf * 1.0f) << "us\n";

    int num = dpuGetOutputTensorSize(task, OUTPUT_NODE);
    float *f_result = new float[294912];
    int8_t *result = new int8_t[num];
    dpuGetOutputTensorInCHWInt8(task, OUTPUT_NODE, result, num);
    for (int i = 0; i < num; i++)
    {
        f_result[i] = (+result[i]) * 2;
    }
    dpuDestroyTask(task);
    dpuDestroyKernel(kernel);
    delete[] result;
    return f_result;
} 


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
    ros::NodeHandle n;
    ros::ServiceClient client1, client2, client3;

    client1 = n.serviceClient<vonetvlad::Param>("ParamManager");
    vonetvlad::Param srv;
    srv.request.name = "dpu_NetVLAD_init";
    if (client1.call(srv))
    {
        running_dpu_VO = srv.response.running_dpu_VO;
        depth_dpu_VO = srv.response.depth_dpu_VO;
        depth_cpu_NetVLAD = srv.response.depth_cpu_NetVLAD;
    }
    else
    {
        ROS_INFO("Error!");
        return;
    }

    ROS_INFO_STREAM("depth_cpu_NetVLAD: " << depth_cpu_NetVLAD << "; depth_dpu_VO: " << depth_dpu_VO << "; running_dpu_VO: " << running_dpu_VO << "; recent_doing: " << recent_doing);
    
    if ( (running_dpu_VO > 0) ||  depth_dpu_VO > 1 || depth_cpu_NetVLAD > 1 || recent_doing>0){
        if( recent_doing){
            count_since_last += 1;
            if ( depth_cpu_NetVLAD ==0 || count_since_last > 6){
                recent_doing = 0;
            }
        }
        return;
    }
    else{
        recent_doing = 1;
        count_since_last = 0;
        client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
        vonetvlad::Param srv2;
        srv2.request.name = "dpu_NetVLAD_begin";
        // 修改参数
        client2.call(srv2);
        // n.setParam("running_dpu_NetVLAD", 1);
        if (srv2.response.running_dpu_VO == 1)
        {
            ROS_INFO("Time lines overlape.");
        }
        else{
            ROS_INFO("DPU NetVLAD heard: [%s]", msg->ID.c_str());
            // Debug
            // ROS_INFO("############# %d ##################", msg->data.at(10));
            cv::Mat frame = cv_bridge::toCvCopy(msg->frame, "bgr8")->image;
            // starting time
            ros::Time start = ros::Time::now();
            // doing computation
            // usleep(66*1000);
            float* DPU_result = dpuNetVLAD(frame);
            // finishing time
            ROS_INFO_STREAM("image ID: " << msg->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());
            
            // 将计算结果转换为自定义消息"my_data"发布
            // 定义自定义消息
            vonetvlad::my_data dat;
            std::stringstream ss;
            ss << msg->ID;
            dat.ID = ss.str();
            std::vector<float> vec(DPU_result, DPU_result + rH*rW*rC);
            dat.data = vec;
            /*std::stringstream ss;
            std_msgs::String msg_pub;
            ss << " DPU NetVLAD result: " << msg->data.c_str();
            msg_pub.data = ss.str();*/
            dpu_netvald_pub.publish(dat);
        }
        client3 = n.serviceClient<vonetvlad::Param>("ParamManager");
        vonetvlad::Param srv3;
        srv3.request.name = "dpu_NetVLAD_end";
        // 修改参数
        client3.call(srv3);
        // n.setParam("running_dpu_NetVLAD", 0);
    }
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "dpu_NetVLAD");
    // 开启DPU
    dpuOpen();
    dpu_NetVLAD dpu_NetVLAD_inst;
    ros::Rate loop_rate(50);
    
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    dpuClose();
    return 0;
}

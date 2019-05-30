/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include <string.h>
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
#define rH (3)
#define rW (10)
#define rC (256)
#define INPUT_NODE "conv_0_pose"
#define OUTPUT_NODE "conv_5_pose"
#define length 4096

using namespace cv;
using namespace std;
cv::Mat pre_frame;
std::string pre_ID = "0";

void out_file(DPUTask* task) {
    int num = dpuGetOutputTensorSize(task, OUTPUT_NODE);
    float* result = new float[num];
    dpuGetOutputTensorInHWCFP32(task, OUTPUT_NODE, result, num);
    //result = dpuGetOutputTensorAddress(task, OUTPUT_NODE);
    ofstream outfile("result_HWC.txt", ios::out);
    if(!outfile) {
            cerr<<"open outfile erro"<<endl;
            exit(1);
    }
    for(int i=0; i<num; i++) {
            outfile<<result[i]<<" ";
    }
    outfile.close();

    dpuGetOutputTensorInCHWFP32(task, OUTPUT_NODE, result, num);
    //result = dpuGetOutputTensorAddress(task, OUTPUT_NODE);
    ofstream outfile1("result_CHW.txt", ios::out);
    if(!outfile1) {
            cerr<<"open outfile erro"<<endl;
            exit(1);
    }
    for(int i=0; i<num; i++) {
            outfile1<<result[i]<<" ";
    }
    outfile1.close();
    delete[] result;
}

float* dpuVO(Mat image1, Mat image2) {
    // Attach to DPU driver and prepare for running
    // Load DPU Kernel for DenseBox neural network
    ROS_INFO("ready to open kernel");
    DPUKernel *kernel = dpuLoadKernel("deployVO");
    ROS_INFO("complete open kernel");
    assert(kernel);
    DPUTask *task = dpuCreateTask(kernel, 0);
    // device_setup(kernel, task);
	DPUTensor* Result;
	resize(image1, image1, Size(608,160), (0, 0), (0, 0), INTER_LINEAR);
    resize(image2, image2, Size(608,160), (0, 0), (0, 0), INTER_LINEAR);
	// mean
    // cout << float(image1.at<Vec3b>(0, 0)[0]) << endl;
    // cout << float(image2.at<Vec3b>(0, 0)[0]) << endl;
	/*for (int row = 0; row < height; row++) 
    {
		for (int col = 0; col < width; col++) 
        {
            int b = image.at<Vec3b>(row, col)[0];
            int g = image.at<Vec3b>(row, col)[1];
            int r = image.at<Vec3b>(row, col)[2];
            image.at<Vec3b>(row, col)[0] = b - 104;
            image.at<Vec3b>(row, col)[1] = g - 117;
            image.at<Vec3b>(row, col)[2] = r - 123;
        }
    }*/
    //cout << float(image.at<Vec3b>(0,0)[0]) << endl;
    //cout << float(image.at<Vec3b>(0,0)[0]) - 104 << endl;
    // 3 -> 6
    uchar* test = image1.data;
    cout << float(test[0]) << ' ' << float(test[1]) << ' ' << float(test[2])<< ' ' << float(test[3])<< ' ' << float(test[4])<< endl;

    float* data = new float [583680];
    memset((void*)data, 0, 583680*4);
    for (int c = 0; c < 6; c++)
    {
        for (int h = 0; h < 160; h++)
        {
            for (int w = 0; w < 608; w++)
            {
                if (c == 0)
                data[c*160*608 + h*608 + w] = float(image2.at<Vec3b>(h, w)[0]) - 104;
                if (c == 1)
                data[c*160*608 + h*608 + w] = float(image2.at<Vec3b>(h, w)[1]) - 117;
                if (c == 2)
                data[c*160*608 + h*608 + w] = float(image2.at<Vec3b>(h, w)[2]) - 123;
                if (c == 3)
                data[c*160*608 + h*608 + w] = float(image1.at<Vec3b>(h, w)[0]) - 104;
                if (c == 4)
                data[c*160*608 + h*608 + w] = float(image1.at<Vec3b>(h, w)[1]) - 117;
                if (c == 5)
                data[c*160*608 + h*608 + w] = float(image1.at<Vec3b>(h, w)[2]) - 123;
            }
        }
    }
    cout << data[0] << ' ' << data[97280] << ' ' << data[194560] << endl;
    //cout << data[160*608] << ' ' << data[160*608+1] << endl;
    //cout << data[608] << ' ' << data[609] << endl;
	dpuSetInputTensorInCHWFP32(task, INPUT_NODE, data, 583680);
	dpuRunTask(task);
    ROS_INFO("1");
	int num = dpuGetOutputTensorSize(task, OUTPUT_NODE);
    float* result = new float[num];
    dpuGetOutputTensorInCHWFP32(task, OUTPUT_NODE, result, num);
    // out_file(task);
    dpuDestroyTask(task);
    dpuDestroyKernel(kernel);
    ROS_INFO("2");
    // Dettach from DPU driver & release resources
    return result;
}

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
            if (strcmp(pre_ID.c_str(), "0") == 0)
            {
                ROS_INFO("**************First frame****************");
                ROS_INFO_STREAM("Queue depth: " << _this->frame_queue.size() << "; WORD: " << _this->frame_queue.front()->ID.c_str() );
                pre_ID = _this->frame_queue.front()->ID.c_str();
                pre_frame = cv_bridge::toCvCopy( _this->frame_queue.front()->frame, "bgr8")->image;
                ROS_INFO_STREAM(pre_frame.cols << ' ' << pre_frame.rows << ' ' << int(pre_frame.ptr<uchar>(100)[100*3]) << ' ' << int(pre_frame.ptr<uchar>(200)[400*3]));
                _this->frame_queue.pop();
                client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
                vonetvlad::Param srv2;
                srv2.request.name = "dpu_VO_end";
                srv2.request.size = _this->frame_queue.size();
                //ROS_INFO("####");
                client2.call(srv2);
                ROS_INFO("#############################################");
            }
            else{
                ROS_INFO_STREAM("Queue depth: " << _this->frame_queue.size() << "; WORD: " << _this->frame_queue.front()->ID.c_str() );
                cv::Mat now_frame = cv_bridge::toCvCopy( _this->frame_queue.front()->frame, "bgr8")->image;
                // starting time
                ros::Time start = ros::Time::now();
                // doing computation
                // usleep(3*1000);
                float* DPU_result = dpuVO(pre_frame, now_frame);
                // finishing time
                ROS_INFO_STREAM("previous image ID: " << pre_ID);
                ROS_INFO_STREAM("image ID: " << _this->frame_queue.front()->ID << "; starting time: " << start << "; finishing time: " << ros::Time::now());

                // test opencv image
                cv::Mat cv_frame = cv_bridge::toCvCopy( _this->frame_queue.front()->frame, "bgr8")->image;
                ROS_INFO_STREAM(cv_frame.cols << ' ' << cv_frame.rows << ' ' << int(cv_frame.ptr<uchar>(100)[100*3]) << ' ' << int(cv_frame.ptr<uchar>(200)[400*3]));
                // 将计算结果转换为自定义消息"my_data"发布
                // 定义自定义消息
                vonetvlad::my_data dat;
                std::stringstream ss;
                ss << _this->frame_queue.front()->ID;
                dat.ID = ss.str();
                std::vector<float> vec(DPU_result, DPU_result + rH*rW*rC);
                dat.data = vec;
                dpu_vo_pub.publish(dat);
                ros::spinOnce();
                // change pre_frame
                pre_ID = _this->frame_queue.front()->ID.c_str();
                pre_frame = cv_bridge::toCvCopy( _this->frame_queue.front()->frame, "bgr8")->image;
                
                _this->frame_queue.pop();
                // Try
                client2 = n.serviceClient<vonetvlad::Param>("ParamManager");
                vonetvlad::Param srv2;
                srv2.request.name = "dpu_VO_end";
                srv2.request.size = _this->frame_queue.size();
                //ROS_INFO("####");
                client2.call(srv2);
                ROS_INFO("#############################################");
            }
        }
        if (srv.response.cond == 2)
        {
            loop_rate2.sleep();
        }
    }
  }
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "dpu_VO");
    // 开启DPU
    dpuOpen();
    dpu_VO dpu_VO_inst;
    ros::Rate loop_rate(50);
    dpu_VO_inst.startdputhread();
    
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    dpu_VO_inst.dpu_thread.join();
    dpuClose();
    return 0;
}

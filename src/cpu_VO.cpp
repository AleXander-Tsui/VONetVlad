/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include<fstream>
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
#include <chrono>

#define H (240)
#define W (640)
#define C (3)
#define rH (3)
#define rW (10)
#define rC (256)

using namespace std;

const string baseImagePath = "/home/linaro/xuyf/catkin_ws/src/vonetvlad/weights";
const string tmpLsPath = "/tmp/tmpLsPath";

vector<std::string> files;
vector<float*> weights;

int ListImages(std::string const &path, std::string const &tmppath, std::vector< std::string> &images) {
    images.clear();
    system( ("ls "+ path + " > " + tmppath ).c_str() );
    std::ifstream fin(tmppath, std::ios::in);
    int count = 0;
    char line[1024];
    while ( fin.getline(line,1000) ) {
        std::string name = line;
        std::string ext = name.substr(name.find_last_of(".") + 1);
        if ((ext == "bin") || (ext == "bit")) {
            images.push_back(name);
            count ++;
        }
    }
    return count;
}

float* getfilefrombin(std::string const &filename, float* &result){
    std::ifstream fin;
    fin.open(filename.c_str(), std::ios::binary);
    //获得文件的大小
	fin.seekg(0, std::ios::end);
	long fsize = fin.tellg();
	std::cout << "file len: " << fsize << " num len: "  <<  fsize/(sizeof(float)) << std::endl;
    fin.seekg(0, std::ios::beg);
    std::cout << "1" << std::endl;
    result = new float[fsize/(sizeof(float))];
    std::cout << "2" << std::endl;
    fin.read(( char * ) result, fsize*sizeof(char));
    std::cout << "3" << std::endl;
        std::cout << result << std::endl;
        std::cout << result[2] << std::endl;
    fin.close();
    return result;
}

int cpuVO(float* result){
    // read data
    int datalen=0;
    float* inputdata = result;
    int layerinsize[3] = {7680,512,512};
    int layeroutsize[3] = {512,512,6};
    float* layerout[3];
    for (int layer=0; layer < 3; layer++){
        layerout[layer] = new float[layeroutsize[layer]];
        float* output_layer = layerout[layer];
        float* input_layer = layer ? layerout[layer - 1] : inputdata;
        float* weight_layer = weights[layer*2];
        float* bias_layer = weights[layer*2 + 1];
        int m_layer = layerinsize[layer];
        int n_layer = layeroutsize[layer];
        std::cout<< "m_layer: " << m_layer << " ;n_layer: " << n_layer <<  std::endl;
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        for(int s=0;s<n_layer;s++){  
            output_layer[s] = 0;
            for(int m=0;m<m_layer;m++){
                output_layer[s] += input_layer[m] * weight_layer[s*m_layer + m];
            }
        } 
        for(int s=0;s<n_layer;s++){  
            output_layer[s] += bias_layer[s];
        }
        if (layer != 2)
        {
            for(int s=0;s<n_layer;s++)
            {  
                if(output_layer[s] < 0)
                    output_layer[s] = 0;
            }
        }
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout<<" time cost= "<< (time_used.count() * 1000) <<" ms."<<std::endl;
    }
    
    ofstream outfile1("/home/linaro/xuyf/catkin_ws/src/vonetvlad/fc_0_result.txt", ios::out);
    ofstream outfile2("/home/linaro/xuyf/catkin_ws/src/vonetvlad/fc_1_result.txt", ios::out);
    ofstream outfile3("/home/linaro/xuyf/catkin_ws/src/vonetvlad/fc_2_result.txt", ios::out);
    if(!outfile1) {
        cerr<<"open outfile erro"<<endl;
        exit(1);
    }
    for(int i=0; i<layeroutsize[0]; i++) {
        outfile1<<layerout[0][i]<<" ";
    }
    outfile1.close();
    if(!outfile3) {
        cerr<<"open outfile erro"<<endl;
        exit(1);
    }
    for(int i=0; i<layeroutsize[2]; i++) {
        outfile3<<layerout[2][i]<<" ";
    }
    outfile3.close();
    if(!outfile2) {
        cerr<<"open outfile erro"<<endl;
        exit(1);
    }
    for(int i=0; i<layeroutsize[1]; i++) {
        outfile2<<layerout[1][i]<<" ";
    }
    outfile2.close();
    
    delete[] inputdata;
    /*while (!weights.empty())
    {
        float* weights_rel = weights.back();
        delete[] weights_rel;
        weights.pop_back();
    }*/
    return 0;
} 

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
    ROS_INFO_STREAM(msg->data.at(10) << msg->data.at(100) << msg->data.at(200));
    
    // n.param<int>("running_cpu_VO",tmp,0);
    // 修改参数
    client1.call(srv);
    // doing computation
    // n.setParam("running_cpu_VO", 1);
    // starting time
    float *DPU_result = new float[rW*rH*rC];
    for (int i=0; i<msg->data.size(); i++){
        DPU_result[i] = msg->data[i];
    }
    ros::Time start = ros::Time::now();
    ROS_INFO("Ready to cal.");
    // usleep(10*1000);
    cpuVO(DPU_result);
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
    ListImages(baseImagePath,tmpLsPath, files);
    for(auto fileN : files){
        std::cout << fileN << std::endl;
        float* weights_temp;
        getfilefrombin("/home/linaro/xuyf/catkin_ws/src/vonetvlad/weights/"+fileN, weights_temp);
        std::cout << "4" << std::endl;
        weights.push_back(weights_temp);
    }
    while (ros::ok()){
        // while()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include <fstream>
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
#define rH (24)
#define rW (24)
#define rC (512)

float conv_weight[32768];
float cent[32768];
float WPCA_w[4194304];
float WPCA_b[128];

float f_result[294912];
float after_softmax[36864];
float vlad[32768];
float fin[128];

void normalize(float *data)
{
	for (int i = 0; i < 576; i++)
	{
		float sum = 0;
		for (int j = 0; j < 512; j++)
		{
			sum = sum + data[j * 576 + i] * data[j * 576 + i];
		}
		sum = sqrt(sum);
		if (sum < 1e-12)
			sum = 1e-12;
		for (int j = 0; j < 512; j++)
		{
			data[j * 576 + i] = data[j * 576 + i] / sum;
		}
	}
}

void conv(float *data, float *a, float *w)
{
	for (int i = 0; i < 36864; i++)
	{
		a[i] = 0;
	}
	for (int i = 0; i < 64; i++)
	{
		for (int j = 0; j < 576; j++)
		{
			for (int k = 0; k < 512; k++)
			{
				a[i * 576 + j] = a[i * 576 + j] + data[k * 576 + j] * w[512 * i + k];
			}
		}
	}
	for (int i = 0; i < 576; i++)
	{
		float sum = 0;
		for (int j = 0; j < 64; j++)
		{
			a[j * 576 + i] = a[j * 576 + i] - 360;
			sum = sum + exp(a[j * 576 + i]);
		}
		for (int j = 0; j < 64; j++)
		{
			a[j * 576 + i] = exp(a[j * 576 + i]) / sum;
		}
	}
}

void vlad_core(float *data, float *a, float *v, float *c)
{
	float *temp = new float[64 * 512 * 576];
	for (int i = 0; i < 64; i++)
	{
		for (int j = 0; j < 512; j++)
		{
			for (int k = 0; k < 576; k++)
			{
				temp[i * 576 * 512 + j * 576 + k] = data[j * 576 + k] + c[i * 512 + j];
				temp[i * 576 * 512 + j * 576 + k] *= a[576 * i + k];
			}
		}
	}
	for (int i = 0; i < 64; i++)
	{
		for (int j = 0; j < 512; j++)
		{
			for (int k = 0; k < 576; k++)
			{
				v[i * 512 + j] += temp[i * 512 * 576 + j * 576 + k];
			}
		}
	}
}

void normalize2(float *vlad)
{
	float *temp = new float[32768];
	for (int i = 0; i < 64; i++)
	{
		float sum = 0;
		for (int j = 0; j < 512; j++)
		{
			sum = sum + vlad[j + i * 512] * vlad[j + i * 512];
		}
		sum = sqrt(sum);
		if (sum < 1e-12)
			sum = 1e-12;
		for (int j = 0; j < 512; j++)
		{
			temp[j * 64 + i] = vlad[j + i * 512] / sum;
		}
	}
	float sum = 0;
	for (int i = 0; i < 32768; i++)
		sum += (temp[i] * temp[i]);
	sum = sqrt(sum);
	for (int i = 0; i < 32768; i++)
		vlad[i] = (temp[i] / sum);
}

void fc(float *v, float *f, float *w, float *b)
{
	for (int i = 0; i < 128; i++)
		f[i] = 0;
	for (int i = 0; i < 128; i++)
	{
		for (int j = 0; j < 32768; j++)
		{
			f[i] = f[i] + v[j] * w[i * 32768 + j];
		}
		f[i] = f[i] + b[i];
	}
}

void normalize3(float *f)
{
	float sum = 0;
	for (int i = 0; i < 128; i++)
	{
		sum += f[i] * f[i];
	}
	sum = sqrt(sum);
	for (int i = 0; i < 128; i++)
	{
		f[i] = f[i] / sum;
	}
}


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
            // ROS_INFO("############# %f ##################", _this->dpu_netvlad_queue.front()->data.at(10));
            float *f_result = new float[rW*rH*rC];
            for (int i=0; i<_this->dpu_netvlad_queue.front()->data.size(); i++){
                f_result[i] = _this->dpu_netvlad_queue.front()->data[i];
            }
            // starting time
            ros::Time start = ros::Time::now();
            // doing computation
            // usleep(360*1000);
            normalize(f_result);
            conv(f_result, after_softmax, conv_weight);
            for (int i = 0; i < 32768; i++)
                vlad[i] = 0;
            vlad_core(f_result, after_softmax, vlad, cent);

            normalize2(vlad);

            fc(vlad, fin, WPCA_w, WPCA_b);
            normalize3(fin);

            std::cout<< fin[0] << " " << fin[1] << " " << fin[2] << std::endl;
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

    // 载入网络参数
    std::fstream in("/home/linaro/xuyf/catkin_ws/src/vonetvlad/weights/vlad_weight", std::ios::in | std::ios::binary);
	in.read((char *)conv_weight, 32768 * sizeof(float));
	in.read((char *)cent, 32768 * sizeof(float));
	in.read((char *)WPCA_w, 4194304 * sizeof(float));
	in.read((char *)WPCA_b, 128 * sizeof(float));
	in.close();
    cpu_NetVLAD_inst.startcputhread();

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    cpu_NetVLAD_inst.cpu_thread.join();
    return 0;
}

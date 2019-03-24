/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayDimension.h"
#include "vonetvlad/my_image.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <array>
#include <time.h>

#define H (240)
#define W (640)
#define C (3)

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "frame_gen");
    
    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
    ros::Publisher chatter_pub = n.advertise<vonetvlad::my_image>("frame", 1);

    // 设置循环的频率
    ros::Rate loop_rate(20);

    int count = 0;
    while (ros::ok())
    {   
        // 定义自定义消息
        vonetvlad::my_image dat;
        std::stringstream ss;
        ss << count;
        dat.ID = ss.str();
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
        dat.layout.dim[0].label = "height";
        dat.layout.dim[1].label = "width";
        dat.layout.dim[2].label = "channal";
        dat.layout.dim[0].size = H;
        dat.layout.dim[1].size = W;
        dat.layout.dim[2].size = C;
        dat.layout.dim[0].stride = H*W*C;
        dat.layout.dim[1].stride = W*C;
        dat.layout.dim[2].stride = C;
        dat.layout.data_offset = 0;
        std::vector<uint8_t> vec(W*H*C, 0);

        for (int i=0; i<H; i++)
            for (int j=0; j<W; j++)
                for (int z=0; z<C; z++)
                    vec[i*W*C + j*C + z] = rand() % 10 + 1 ;
        dat.data = vec;

        // 发布消息
        ROS_INFO_STREAM("Image ID: " << dat.ID.c_str());
        chatter_pub.publish(dat);

        // 循环等待回调函数
        ros::spinOnce();

        // 按照循环频率延时
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

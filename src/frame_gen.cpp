/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayDimension.h"
#include "sensor_msgs/Image.h"
#include "vonetvlad/my_image.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <array>
#include <time.h>

#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <cv_bridge/cv_bridge.h> 

#include <dnndk/dnndk.h>


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
        std::string image = "/home/linaro/xuyf/catkin_ws/src/vonetvlad/images/0000001101.png";
        cv::Mat cv_frame = cv::imread(image);
        // test opencv image
        ROS_INFO_STREAM(cv_frame.cols << ' ' << cv_frame.rows << ' ' << int(cv_frame.ptr<uchar>(100)[100*3]) << ' ' << int(cv_frame.ptr<uchar>(200)[400*3]));
        vonetvlad::my_image imsg;
        sensor_msgs::Image::Ptr ros_frame;
        std_msgs::Header head;
        ros_frame = cv_bridge::CvImage(head, "bgr8", cv_frame).toImageMsg();
        imsg.frame = *ros_frame;
        std::stringstream ss;
        ss << count;
        imsg.ID = ss.str();

        // 发布消息
        ROS_INFO_STREAM("Image ID: " << imsg.ID.c_str());
        chatter_pub.publish(imsg);

        // 循环等待回调函数
        ros::spinOnce();

        // 按照循环频率延时
        loop_rate.sleep();
        ++count;
    }
    return 0;
}

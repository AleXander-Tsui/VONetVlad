#include "ros/ros.h"
#include "vonetvlad/Param.h"
#include <string.h>
/*
class my_server{
public:
    my_server(){
        service = n.advertiseService("ParamManager", &my_server::manage_param, this);
    }
    bool manage_param(vonetvlad::Param::Request  &req,
                      vonetvlad::Param::Response &res);

private: 
    ros::NodeHandle n;
    ros::ServiceServer service;
};
*/

int running_dpu_VO = 0;
int running_dpu_NetVLAD = 0;
int running_cpu_VO = 0;
int running_cpu_NetVLAD = 0;
int depth_dpu_VO = 0;
int depth_cpu_NetVLAD = 0;

bool manage_param(vonetvlad::Param::Request  &req,
                  vonetvlad::Param::Response &res)
{
    ROS_INFO("server begin");
    ros::NodeHandle n;
    // 初始化参数
    /*n.param<int>("running_dpu_VO", running_dpu_VO, 0);
    n.param<int>("running_dpu_NetVLAD", running_dpu_NetVLAD, 0);
    n.param<int>("running_cpu_VO", running_cpu_VO, 0);
    n.param<int>("running_cpu_NetVLAD", running_cpu_NetVLAD, 0);
    n.param<int>("depth_dpu_VO", depth_dpu_VO, 0);
    n.param<int>("depth_cpu_NetVLAD", depth_cpu_NetVLAD, 0);*/
    //usleep(5*1000);
    // 根据不同情况以及参数确定是否允许运行、改变参数
    if (strcmp(req.name.c_str(), "dpu_VO_begin") == 0)
    {
        //ros::Rate loop_rate2(50);
        // 空队列
        if (req.size == 0)
        {
            //n.setParam("running_dpu_VO", 0);
            //n.setParam("depth_dpu_VO", 0 );
            running_dpu_VO = 0;
            depth_dpu_VO = 0;
            res.cond = 0;
            return true;
        }
        else
        {
            if((! running_dpu_NetVLAD) && ( (depth_cpu_NetVLAD <= 1 && running_cpu_NetVLAD) || (depth_cpu_NetVLAD < 1 && ! running_cpu_NetVLAD) ))
            {
                //n.setParam("running_dpu_VO", 1);
                //n.setParam("depth_dpu_VO", int(req.size));
                running_dpu_VO = 1;
                depth_dpu_VO = int(req.size);
                res.cond = 1;
                return true;
            }
            else
            {
                ROS_INFO_STREAM("Queue not empty, but not start");
                //n.setParam("depth_dpu_VO", int(req.size));
                depth_dpu_VO = int(req.size);
                res.cond = 2;
                return true;
            }
        }  
    }
    if (strcmp(req.name.c_str(), "dpu_VO_end") == 0)
    {
	    ROS_INFO("dpu vo end");
        //n.setParam("running_dpu_VO", 0);
        //n.setParam("depth_dpu_VO", int(req.size));
        running_dpu_VO = 0;
        depth_dpu_VO = int(req.size);
        res.cond = 3;
        return true;
    }
    if (strcmp(req.name.c_str(), "dpu_NetVLAD_init") == 0)
    {
        //n.getParam("running_dpu_VO", running_dpu_VO);
        //n.getParam("depth_dpu_VO", depth_dpu_VO);
        //n.getParam("depth_cpu_NetVLAD", depth_cpu_NetVLAD);
        res.running_dpu_VO = running_dpu_VO;
        res.depth_dpu_VO = depth_dpu_VO;
        res.depth_cpu_NetVLAD = depth_cpu_NetVLAD;
        return true;
    }
    if (strcmp(req.name.c_str(), "dpu_NetVLAD_begin") == 0)
    {
        //n.setParam("running_dpu_NetVLAD", 1);
        res.running_dpu_VO = running_dpu_VO;
	    running_dpu_NetVLAD = 1;
        return true;
    }
    if (strcmp(req.name.c_str(), "dpu_NetVLAD_end") == 0)
    {
        //n.setParam("running_dpu_NetVLAD", 0);
	    running_dpu_NetVLAD = 0;
        return true;
    }    
    if (strcmp(req.name.c_str(), "cpu_VO_begin") == 0)
    {
        //n.setParam("running_cpu_VO", 1);
	    running_cpu_VO = 1;
        return true;
    }    
    if (strcmp(req.name.c_str(), "cpu_VO_end") == 0)
    {
        //n.setParam("running_cpu_VO", 0);
	    running_cpu_VO = 0;
        return true;
    }
    if (strcmp(req.name.c_str(), "cpu_NetVLAD_begin") == 0)
    {
        //n.setParam("running_cpu_NetVLAD", 1);
        //n.setParam("depth_cpu_NetVLAD", req.size);
	    running_cpu_NetVLAD = 1;
	    depth_cpu_NetVLAD = int(req.size);
        return true;
    }
    if (strcmp(req.name.c_str(), "cpu_NetVLAD_end") == 0)
    {
        //n.setParam("running_cpu_NetVLAD", 0);
        //n.setParam("depth_cpu_NetVLAD", req.size);
        running_cpu_NetVLAD = 0;
        depth_cpu_NetVLAD = int(req.size);
        return true;
    }
    if (strcmp(req.name.c_str(), "cpu_NetVLAD_freeze") == 0)
    {
        //n.setParam("running_cpu_NetVLAD", 0);  
        //n.setParam("depth_cpu_NetVLAD", 0);
	    running_cpu_NetVLAD = 0;
        depth_cpu_NetVLAD = 0;	
        return true;
    }
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("ParamManager", manage_param);

    ROS_INFO("Ready to work.");
    ros::spin();
    return 0;
}

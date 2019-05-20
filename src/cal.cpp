#include <assert.h>
#include "ros/ros.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <chrono>

/* header file OpenCV for image processing */
#include <opencv2/opencv.hpp>

/* header file for DNNDK APIs */
#include <dnndk/dnndk.h>

#define INPUT_NODE "conv_0_pose"
#define OUTPUT_NODE "conv_5_pose"
#define length 4096

using namespace std;
//using namespace std::chrono;
using namespace cv;

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
    //cout << data[0] << ' ' << data[1] << endl;
    //cout << data[160*608] << ' ' << data[160*608+1] << endl;
    //cout << data[608] << ' ' << data[609] << endl;
	dpuSetInputTensorInCHWFP32(task, INPUT_NODE, data, 583680);
	dpuRunTask(task);
	int num = dpuGetOutputTensorSize(task, OUTPUT_NODE);
    float* result = new float[num];
    dpuGetOutputTensorInCHWFP32(task, OUTPUT_NODE, result, num);
    //out_file(task);
    dpuDestroyTask(task);
    dpuDestroyKernel(kernel);

    // Dettach from DPU driver & release resources
    return result;
}
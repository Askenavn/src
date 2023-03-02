#include <iostream>
#include <array>
#include <vector>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


class VideoInput{
public:
    VideoInput(ros::NodeHandle nh);
    ~VideoInput();
    void getFrame(cv::Mat& frame);
    ros::NodeHandle nh;
    cv::VideoCapture videoCapture;
};

#include <iostream>
#include <array>
#include <vector>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <fiducial_positioning_system/VideoInputConfig.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


class VideoInput{
public:
    VideoInput(ros::NodeHandle nh);

    ~VideoInput();
    void getFrame(cv::Mat& frame);

private:
    void setupVideoCapture(fid_ps::VideoInputConfig& config, uint32_t level);

    ros::NodeHandle nh;
    dynamic_reconfigure::Server<fid_ps::VideoInputConfig> drServer;
    cv::VideoCapture videoCapture;
};

#include "FiducialPositioningSystem.h"


VideoInput::VideoInput(ros::NodeHandle nh):
    nh(nh),
    drServer(this->nh){

    videoCapture.open(0, cv::CAP_V4L2 );
    videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    drServer.setCallback(boost::bind(&VideoInput::setupVideoCapture, this, _1, _2));


    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    videoCapture.set(cv::CAP_PROP_FPS, 30);
}

VideoInput::~VideoInput(){
    videoCapture.release();
}

void VideoInput::getFrame(cv::Mat& frame) {
    videoCapture.read(frame);
}

void VideoInput::setupVideoCapture(fid_ps::VideoInputConfig& config, uint32_t level) {
    ROS_INFO("Received reconfiguration request");
    ROS_INFO("int_param: %d", config.int_param);
    videoCapture.set(cv::CAP_PROP_BRIGHTNESS, config.int_param);
}



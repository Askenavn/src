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

    videoCapture.set(cv::CAP_PROP_FPS, config.fps);
    ROS_INFO("FPS: %d", (int)videoCapture.get(cv::CAP_PROP_FPS));

    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    ROS_INFO("resolution: %dx%d", (int)videoCapture.get(cv::CAP_PROP_FRAME_WIDTH),(int)videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));

    videoCapture.set(cv::CAP_PROP_BRIGHTNESS, config.brightness);
    ROS_INFO("brightness: %d", (int)videoCapture.get(cv::CAP_PROP_BRIGHTNESS));

    videoCapture.set(cv::CAP_PROP_CONTRAST, config.contrast);
    ROS_INFO("contrast: %d", (int)videoCapture.get(cv::CAP_PROP_CONTRAST));

    videoCapture.set(cv::CAP_PROP_SATURATION, config.saturation);
    ROS_INFO("saturation: %d", (int)videoCapture.get(cv::CAP_PROP_SATURATION));

}



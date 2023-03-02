#include "FiducialPositioningSystem.h"



VideoInput::VideoInput(ros::NodeHandle nh):
        nh(nh){
    videoCapture.open(0, cv::CAP_V4L2 );
    videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, 1900);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    videoCapture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    videoCapture.set(cv::CAP_PROP_FPS, 30);

}

VideoInput::~VideoInput(){
    videoCapture.release();
}

void VideoInput::getFrame(cv::Mat& frame) {
    videoCapture.read(frame);
}


#include "FiducialPositioningSystem.h"
#include <ros/ros.h>




#include <ros/ros.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh("hi");
    cv::Mat frame;
    VideoInput vi(nh);
    ROS_INFO("my_node is running");
    while (1){
        std::string sss;

        vi.getFrame(frame);
        if(frame.empty()) continue;
        cv::imshow("Camera", frame);
        if (cv::waitKey(1) == 27) break;
        ros::spinOnce();
    }

    return 0;
}
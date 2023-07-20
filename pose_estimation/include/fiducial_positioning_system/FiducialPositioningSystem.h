#include <iostream>
#include <array>
#include <vector>
#include <chrono>
#include <map>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <fiducial_positioning_system/VideoInputConfig.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>


class VideoInput{
    friend class VideoProcess;
public:
    VideoInput(ros::NodeHandle nh, int camId);

    ~VideoInput();
    void getFrame(cv::Mat& frame);
    void showParams();
    bool isWorking();

private:
    void setupVideoCapture(fid_ps::VideoInputConfig& config, uint32_t level);

    ros::NodeHandle nh;
    dynamic_reconfigure::Server<fid_ps::VideoInputConfig> drServer;
    cv::VideoCapture videoCapture;

};

class VideoProcess{
public:

    void convertGray(cv::Mat& rgb, cv::Mat& gray);
    void binarize(cv::Mat& gray, cv::Mat& wb, double thresh);
    void addTrackbar(const char* winname, int& threshold);
private:

};

class Marks{
public:

    void detectMarks(cv::Mat frame);
    cv::Mat drawMarks(cv::Mat frame);
    void getVectors(double sidelen, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    cv::Mat drawAxis(cv::Mat frame, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    std::vector<cv::Point3d> getCameraPose();
    std::vector<int> getIds();
    void sendPose(std::vector<cv::Point3d> poses);
    bool sendShapes(std::vector<cv::Point3d> poses, std::map<int, std::array<double, 3>> dict, ros::NodeHandle& n);
    void setIds(std::vector<int> ids);
    std::vector<cv::Point3d> getReferencePoses(std::vector<cv::Point3d> poses);
    void getReferenceVecs();
    std::vector<std::array<cv::Point3d, 4>> shapesCorners(std::map<int, std::array<double, 3>> dict);
private:

   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> corners;
   std::vector<cv::Vec3d> rvecs;
   std::vector<cv::Vec3d> tvecs;

};

std::vector<cv::Point3d> filter( std::vector<cv::Point3d> prePoses, std::vector<cv::Point3d> curPoses);

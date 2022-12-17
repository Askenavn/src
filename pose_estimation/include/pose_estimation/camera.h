#include <iostream>
#include <array>
#include <vector>
 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

struct CameraParams{
    int camId;
    int FPS;
    std::array<int, 2> resolution;

    CameraParams(int id, int width, int height): 
        camId(id), resolution({width, height}){
    }
};

class Camera{
public:
    std::string winname;
    cv::VideoCapture cap;
    bool isWorking;

    Camera(CameraParams params);

    cv::Mat getFrame();

    CameraParams getCameraInfo();
};

struct Marks{
   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> corners;
   std::vector<cv::Vec3d> rvecs;
   std::vector<cv::Vec3d> tvecs;

   void detectMarks(cv::Mat frame);

   cv::Mat drawMarks(cv::Mat frame);

   void getVectors(cv::Mat cameraMatrix, cv::Mat distCoeffs);

   cv::Mat drawAxis(cv::Mat frame, cv::Mat cameraMatrix, cv::Mat distCoeffs);
   
   std::vector<cv::Point3d> getCameraPose();
};
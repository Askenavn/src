#include "camera.h"

Camera::Camera(CameraParams params):
      cap(params.camId), isWorking(false){
   cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

   if (cap.isOpened() == false) {
      std::cout << "[LOG ]: Cannot open the video camera" << std::endl;
      return;
   }

   isWorking = true;

   cap.set(cv::CAP_PROP_FRAME_WIDTH, params.resolution[0]);
   cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.resolution[1]);
}

cv::Mat Camera::getFrame(){
   cv::Mat frame;
   isWorking = cap.read(frame); 

   if (isWorking == false) {
      std::cout << "[LOG ]: Video camera is disconnected" << std::endl;
   }

   return frame;
}

CameraParams Camera::getCameraInfo(){
   CameraParams par(0, 0, 0);
   par.resolution[0] = cap.get(cv::CAP_PROP_FRAME_WIDTH);
   par.resolution[1] = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
   par.FPS = cap.get(cv::CAP_PROP_FPS);

   return par; 
}


void Marks::detectMarks(cv::Mat frame){
   std::vector<int> markerIds;
   std::vector<std::vector<cv::Point2f>> markerCorners;

   cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

   cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

   ids = markerIds;
   corners = markerCorners;
}

cv::Mat Marks::drawMarks(cv::Mat frame){
   cv::aruco::drawDetectedMarkers(frame, corners, ids);

   return frame;      
}

void Marks::getVectors(cv::Mat cameraMatrix, cv::Mat distCoeffs){
   cv::aruco::estimatePoseSingleMarkers(corners, 0.22, cameraMatrix, distCoeffs, rvecs, tvecs);
}

cv::Mat Marks::drawAxis(cv::Mat frame, cv::Mat cameraMatrix, cv::Mat distCoeffs){
   cv::Mat img = frame.clone();
   for (int i=0; i < rvecs.size(); i++){
      cv::drawFrameAxes(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.11);
   }
   return img;
}

std::vector<cv::Point3d> Marks::getCameraPose(){
   std::vector<cv::Point3d> poses(ids.size());

   for(int k=0; k<ids.size();k++){   
   cv::Mat rotMat;
   cv::Mat transMat = cv::Mat::zeros(4, 4, CV_64F);
   cv::Rodrigues(rvecs[k], rotMat);

   for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
         transMat.at<double>(i,j)=rotMat.at<double>(i,j);
      }
   } 

   transMat.at<double>(0,3) = tvecs[k](0);
   transMat.at<double>(1,3) = tvecs[k](1);
   transMat.at<double>(2,3) = tvecs[k](2);
   transMat.at<double>(3,3) = 1.;
   
   std::cout<<tvecs[k]<<std::endl;
   std::cout<<transMat.inv()<<std::endl;

   cv::Mat invMat = transMat.inv();
   poses[k] = cv::Point3d(invMat.at<double>(0,3), invMat.at<double>(1,3), invMat.at<double>(2,3));
   }   
   
   return poses;
}

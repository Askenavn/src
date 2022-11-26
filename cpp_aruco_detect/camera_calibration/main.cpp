#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

 

void calibrate(std::string a){
   int ChessBoard[2]{6,9};

   std::vector<std::vector<cv::Point3f> > objpoints;
   std::vector<std::vector<cv::Point2f> > imgpoints;
   std::vector<cv::Point3f> objp;

   for(int i{0}; i<ChessBoard[1]; i++){
      for(int j{0}; j<ChessBoard[0]; j++){
         objp.push_back(cv::Point3f(j,i,0));
      }   
   }

   std::vector<cv::String> images;

   std::string path = a + "*.jpg";
   cv::glob(path, images);

   cv::Mat frame, gray;

   std::vector<cv::Point2f> corners;

   
   for(int i{0}; i<images.size(); i++)
   {
      frame = cv::imread(images[i]);
      cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
   
      if(bool success = cv::findChessboardCorners(gray, cv::Size(ChessBoard[0], ChessBoard[1]), 
      corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE)){
         cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
         
         cv::cornerSubPix(gray,corners,cv::Size(11,11), cv::Size(-1,-1),criteria);
         
         cv::drawChessboardCorners(frame, cv::Size(ChessBoard[0], ChessBoard[1]), corners, success);
         
         objpoints.push_back(objp);
         imgpoints.push_back(corners);
      }
      cv::imshow("Image",frame);
      cv::waitKey(1);
   }
   
   cv::destroyAllWindows();
   
   cv::Mat cameraMatrix,distCoeffs,rvec,tvec;
   

   cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, rvec, tvec);
   
   std::cout << "cameraMatrix : "<< std::endl << cameraMatrix << std::endl << cameraMatrix.type()<< std::endl;
   std::cout << "distCoeffs : " << std::endl << distCoeffs << std::endl << distCoeffs.type()<< std::endl;

}


int main(int argc, char* argv[]){
   
   calibrate("/home/nanzat/images/");

   return 0;
}

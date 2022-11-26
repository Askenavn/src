#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

 

struct CameraParams{
   int camId;
   std::array<int, 2> resolution;
   int FPS;

   CameraParams(int id, int width, int height): 
      camId(id), resolution({width, height}){
   }
};

void printInfo(const CameraParams& par){
   std::cout << "[LOG ]:" 
   << " Cam: " << par.camId 
   << " Resolution of the video: " << par.resolution[0] << " x " << par.resolution[1] 
   << " FPS : " << par.FPS << std::endl; 

}

class Camera{
public:
   std::string winname;
   cv::VideoCapture cap;
   bool isWorking;

   Camera(CameraParams params):
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

   cv::Mat getFrame(){
      cv::Mat frame;
      isWorking = cap.read(frame); 

      if (isWorking == false) {
         std::cout << "[LOG ]: Video camera is disconnected" << std::endl;
      }

      return frame;
   }

   CameraParams getCameraInfo(){
      CameraParams par(0, 0, 0);
      par.resolution[0] = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      par.resolution[1] = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      par.FPS = cap.get(cv::CAP_PROP_FPS);

      return par; 
   }

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


      bool success;
      
      for(int i{0}; i<images.size(); i++)
      {
         frame = cv::imread(images[i]);
         cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
      
         success = cv::findChessboardCorners(gray, cv::Size(ChessBoard[0], ChessBoard[1]), 
         corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
         
         if(success)
         {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            cv::cornerSubPix(gray,corners,cv::Size(11,11), cv::Size(-1,-1),criteria);
            
            cv::drawChessboardCorners(frame, cv::Size(ChessBoard[0], ChessBoard[1]), corners, success);
            
            objpoints.push_back(objp);
            imgpoints.push_back(corners);
         }
         std::cout<<images[i]<<std::endl;
         cv::imshow("Image",frame);
         cv::waitKey(0);
      }
      
      cv::destroyAllWindows();
      
      cv::Mat cameraMatrix,distCoeffs,rvec,tvec;
      

      cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, rvec, tvec);
      
      std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
      std::cout << "distCoeffs : " << distCoeffs << std::endl;

         }
};



int main(int argc, char* argv[]){
   CameraParams par(2, 1024, 768);
   Camera camera(par);

   printInfo(par);
   
   while (camera.isWorking){
      cv::Mat frame = camera.getFrame();

      par = camera.getCameraInfo();

      printInfo(par);

      cv::imshow(camera.winname, frame);

      if (cv::waitKey(10) == 27){
         std::cout << "[LOG ]: Esc key is pressed by user. Stopping the video" << std::endl;
         break;
      }
   }
   
   camera.calibrate("/home/nanzat/images/");

   return 0;
}

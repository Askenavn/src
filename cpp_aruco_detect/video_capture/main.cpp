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

   cv::Mat setDist(std::array<double, 5> d){
      cv::Mat dist(cv::Size(5, 1), 6);

      for (int i=0; i<5;i++){
         dist.at<double>(i) = d[i];
      }

      return dist;
   }

      cv::Mat setCamMat(std::array<std::array<double, 3>, 3> mtx){
      cv::Mat camMatrix(cv::Size(3, 3), 6);

      for (int i=0; i<3;i++){
         cv::Mat buff; 
            for(int j=0; j<3; j++){
               camMatrix.at<double>(i,j) = mtx[i][j];
            }
      }

      return camMatrix;
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

   return 0;
}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>


class Camera{
public:
   std::array<int, 2> resolution;
   std::string winname;
   cv::VideoCapture cap;
   bool success;

   Camera(int cam_id):
      cap(cam_id), success(false){
      cap.set(cv::CAP_PROP_FOURCC, ('M','P','E','G'));

      if (cap.isOpened() == false) {
         std::cout << "[LOG ]: Cannot open the video camera" << std::endl;
         return;
      }

      success = true;

      double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
      double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); 
      std::cout << "[LOG ]: Resolution of the video : " << dWidth << " x " << dHeight << std::endl; 
   }

   cv::Mat getFrame(){
      cv::Mat frame;
      success = cap.read(frame); 

      if (success == false) {
         std::cout << "[LOG ]: Video camera is disconnected" << std::endl;
      }

      return frame;
   }

   bool playVideo(cv::Mat frame){
      cv::imshow(winname, frame);

      if (cv::waitKey(10) == 27){
         std::cout << "[LOG ]: Esc key is pressed by user. Stopping the video" << std::endl;
         return false;
      }
      
      return true;
   }
};



int main(int argc, char* argv[]){
   Camera camera(0);
   while (camera.success){
      cv::Mat frame = camera.getFrame();

      if(!camera.playVideo(frame)){
         break;
      }
   }
   return 0;
}
   
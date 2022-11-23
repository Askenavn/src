#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>


class Camera{
public:
   std::array<int, 2> resolution;
   std::string winname;
   cv::VideoCapture cap;

   Camera(int cam_id):
      cap(cam_id){
      cap.set(cv::CAP_PROP_FOURCC, ('M','P','E','G'));

      if (cap.isOpened() == false) {
         std::cout << "[LOG ]: Cannot open the video camera" << std::endl;
         return;
      }

      double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
      double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); 
      std::cout << "[LOG ]: Resolution of the video : " << dWidth << " x " << dHeight << std::endl; 
   }

   cv::Mat getFrame(){
      cv::Mat frame;
      bool bSuccess = cap.read(frame); 

      if (bSuccess == false) {
         std::cout << "[LOG ]: Video camera is disconnected" << std::endl;
      }

      return frame;
   }

};



int main(int argc, char* argv[]){
   Camera camera(0);
   while (true){
      cv::Mat frame = camera.getFrame();

      cv::imshow(camera.winname, frame);

         if (cv::waitKey(10) == 27){
      std::cout << "[LOG ]: Esc key is pressed by user. Stoppig the video" << std::endl;
      break;
      }
   }
   return 0;
}
   
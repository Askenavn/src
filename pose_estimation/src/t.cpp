#include "camera.h"

int main(int argc, char* argv[]){
   CameraParams par(0, 1024, 768);
   Camera camera(par);

   // printInfo1(par);
   
   while (camera.isWorking){
      cv::Mat frame = camera.getFrame();
      cv::imshow(camera.winname, frame);

      if (cv::waitKey(10) == 27){
         std::cout << "[LOG ]: Esc key is pressed by user. Stopping the video" << std::endl;
         break;
      }
   }
   return 0;
}
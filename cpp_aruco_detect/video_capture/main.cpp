#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>

 //новую структуру

struct CameraParams{
   int camId;
   std::array<int, 2> resolution;
   int FPS;

   CameraParams(int id, int width, int height): 
      camId(id), resolution({width, height}){
   }
};

void printResolution(const CameraParams& par){
   std::cout << "[LOG ]:" 
   << " Cam: " << par.camId 
   << " Resolution of the video : " << par.resolution[0] << " x " << par.resolution[1] 
   << " FPS :" << std::endl; 
}

class Camera{
public:
   std::string winname;
   cv::VideoCapture cap;
   bool isWorking;

   Camera(CameraParams params):
      cap(params.camId), isWorking(false){
      cap.set(cv::CAP_PROP_FOURCC, ('M','P','E','G'));

      if (cap.isOpened() == false) {
         std::cout << "[LOG ]: Cannot open the video camera" << std::endl;
         return;
      }

      isWorking = true;

      cap.set(cv::CAP_PROP_FRAME_WIDTH, params.resolution[0]);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.resolution[1]);

      double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //удалить
      double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); 
      std::cout << "[LOG ]: Resolution of the video : " << dWidth << " x " << dHeight << std::endl; 
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
      CameraParams a;
      a.resolution[0] = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      a.resolution[1] = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      a.FPS = cap.get(cv::CAP_PROP_FPS);

      return a; 
   }
};



int main(int argc, char* argv[]){
   CameraParams par(2, 800, 600);
   Camera camera(par);

   camera.printResolution(par);
   
   while (camera.isWorking){
      cv::Mat frame = camera.getFrame();

      camera.getCameraInfo(par);

      camera.printFPS();
     

      cv::imshow(camera.winname, frame);

      if (cv::waitKey(10) == 27){
         std::cout << "[LOG ]: Esc key is pressed by user. Stopping the video" << std::endl;
         break;
      }
   }

   return 0;
}
   

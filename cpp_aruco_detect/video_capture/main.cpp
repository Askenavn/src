#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>

 

struct CameraParams{
   int camId;
   std::array<int, 2> resolution;
   int FPS;

   CameraParams(int id, int width, int height): 
      camId(id), resolution({width, height}){
   }
};

<<<<<<< HEAD

void printInfo(const CameraParams& par){
   std::cout << "[LOG ]:" 
   << " Cam: " << par.camId 
   << " Resolution of the video: " << par.resolution[0] << " x " << par.resolution[1] 
   << " FPS : " << par.FPS << std::endl; 
=======
void printResolution(const CameraParams& par){
   std::cout << "[LOG ]:" 
   << " Cam: " << par.camId 
   << " Resolution of the video : " << par.resolution[0] << " x " << par.resolution[1] 
   << " FPS :" << std::endl; 
>>>>>>> b61d0da6ba472c88f106020d8cf41926231523a1
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
<<<<<<< HEAD
      CameraParams par(0, 0, 0);
      par.resolution[0] = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      par.resolution[1] = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      par.FPS = cap.get(cv::CAP_PROP_FPS);

      return par; 
   }

=======
      CameraParams a;
      a.resolution[0] = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      a.resolution[1] = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      a.FPS = cap.get(cv::CAP_PROP_FPS);

      return a; 
   }
>>>>>>> b61d0da6ba472c88f106020d8cf41926231523a1
};



int main(int argc, char* argv[]){
   CameraParams par(2, 800, 600);
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
   

#include "FiducialPositioningSystem.h"

int main(){
    VideoInput vi;
    cv::Mat frame;


    std::cout << "start" << std::endl;
    while (1){
        vi.getFrame(frame);
        if(frame.empty()) continue;

        cv::imshow("Camera", frame); // отображаем текущий кадр на экране
        if (cv::waitKey(1) == 27) break; // выход из цикла при нажатии клавиши Esc
    }

}
#include "CameraCalibration.h"
#include <iostream>

void writeParams(cv::Mat img, cv::Mat cameraMatrix, cv::Mat distCoeffs){

    std::ofstream file("/home/nanzat/catkin_ws/src/camera_params.txt");

    std::string imgSize(std::to_string(img.rows)+"\n"+std::to_string(img.cols)+"\n");
    std::string mtx;

    for(int i=0;i<cameraMatrix.rows;i++){
        for(int j=0;j<cameraMatrix.cols;j++){
            mtx += std::to_string(cameraMatrix.at<double>(i,j));
            mtx +="\n";
        }
    }

    std::string dist;

    for(int i=0;i<distCoeffs.cols;i++){
        dist += std::to_string(distCoeffs.at<double>(0,i));
        dist +="\n";
    }

    std::string s;

    s = imgSize + mtx + dist;

    file << s;
    file.close();

    return ;
}

bool readParams( cv::Mat cameraMatrix, cv::Mat distCoeffs){

    std::ifstream txt("/home/nanzat/catkin_ws/src/camera_params.txt");
    std::string resolution, buff;

    getline(txt,buff);
    resolution += buff + "x";
    getline(txt,buff);
    resolution += buff;

    getline(txt,buff);

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            getline(txt, buff);
            cameraMatrix.at<double>(i,j)=std::stod(buff);
        }
    }

    getline(txt, buff);

    for(int i=0; i<5; i++){
        getline(txt, buff);
        distCoeffs.at<double>(i)=std::stod(buff);
    }
      
    txt.close();
}
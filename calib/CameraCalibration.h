#include <iostream>
#include <array>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

void writeParams(cv::Mat img, cv::Mat cameraMatrix, cv::Mat distCoeffs);

void readParams(cv::Mat cameraMatrix, cv::Mat distCoeffs);
#include "FiducialPositioningSystem.h"
#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_node");
    ros::NodeHandle nh("hi");

    ROS_INFO("calib_node is running");
    ROS_INFO("\nChoose the source of the images for calibration: \n\t 1 --- videostream; \n\t 0 --- path to folder;");
    bool mode;
    std::cin>>mode;

    std::string folder("src/pose_estimation/images/");

    if(mode){

        ROS_INFO("the videostream mode was chosen");
        ROS_INFO("every frame will be saved in folder: src/pose_estimation/images/, every second");

        cv::Mat frame;

        VideoInput vi(nh);

        cv::namedWindow("Camera", 1);

        int i=0;

        while (1){
            
            if(!vi.isWorking()) break;

            vi.getFrame(frame);
            if(frame.empty()) continue;

            cv::imshow("Camera", frame);

            if(!cv::imwrite(folder + std::to_string(i)+".jpg", frame)){
                ROS_INFO("Error, image wasn't saved");
            }
            i++;
            
            if (cv::waitKey(1000) == 27) break;
            ros::spinOnce();
        }
    }
    else{

        ROS_INFO("the mode by folder was chosen");
        ROS_INFO("type the folder where the images are in");

        std::cin>>folder;
    }

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

    std::string path = folder + "*.jpg";
    cv::glob(path, images);

    cv::Mat frame, gray;

    std::vector<cv::Point2f> corners;


    for(int i{0}; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        if(bool success = cv::findChessboardCorners(gray, cv::Size(ChessBoard[0], ChessBoard[1]), 
        corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE)){
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            cv::cornerSubPix(gray,corners,cv::Size(11,11), cv::Size(-1,-1),criteria);
            
            cv::drawChessboardCorners(frame, cv::Size(ChessBoard[0], ChessBoard[1]), corners, success);
            
            objpoints.push_back(objp);
            imgpoints.push_back(corners);
        }
        cv::imshow("Image",frame);
        cv::waitKey(1);
    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix,distCoeffs,rvec,tvec;


    double repError = cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), 
                                            cameraMatrix, distCoeffs, rvec, tvec);

    std::ofstream file("/home/nanzat/catkin_ws/src/camera_params.txt");

    std::string imgSize(std::to_string(gray.rows)+"\n"+std::to_string(gray.cols)+"\n");
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

    return 0;
}

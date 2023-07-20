#include "FiducialPositioningSystem.h"
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    static ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("marker_pose", 1);
    geometry_msgs::PointStamped mark_pose;

    //***********************
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker\>("visualization_marker\", 1);
    // uint32_t shape = visualization_msgs::Marker::CUBE;
    //***********************

    cv::Mat frame;
    cv::Mat gray;

    VideoInput vi(nh, 2);
    VideoProcess process;

    // ROS_INFO("my_node is running");

    cv::namedWindow("Camera", 1);
    int threshold = 125;
    process.addTrackbar("Camera", threshold);
       ROS_INFO("my_node is running");

//****************************
    
    cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrix.at<double>(0,0)= 1457.1543771212064;
    cameraMatrix.at<double>(0,1)= 0.;
    cameraMatrix.at<double>(0,2)= 475.35928500917271;
    cameraMatrix.at<double>(1,0)= 0.;
    cameraMatrix.at<double>(1,1)= 1462.0138672847527;
    cameraMatrix.at<double>(1,2)= 411.09174792599219;
    cameraMatrix.at<double>(2,0)= 0.;
    cameraMatrix.at<double>(2,1)= 0.;
    cameraMatrix.at<double>(2,2)= 1.;

    distCoeffs.at<double>(0,0)= -0.42335861220347393;
    distCoeffs.at<double>(0,1)= 0.35641500875717114;
    distCoeffs.at<double>(0,2)= -0.0023359806213360639;
    distCoeffs.at<double>(0,3)= 0.0076403906656301356;
    distCoeffs.at<double>(0,4)= -0.34337436357687584;
    double sidelen = 0.175;
    double sidelenRobots = 0;

    std::vector<cv::Point3d> buffPose = {cv::Point3d(0.,0.,0.), cv::Point3d(0.,0.,0.), cv::Point3d(0.,0.,0.)};
    // std::string path = "/home/nanzat/aruco_images/(115;75;220).jpg";
    std::string path = "/home/nanzat/aruco_images/2.jpg";
    // std::array<double, 4> blue = {0.0f, 0.0f, 1.0f, 1.0};
    // std::array<double, 4> green = {0.0f, 1.0f, 0.0f, 1.0};
    // std::array<double, 4> red = {1.0f, 0.0f, 0.0f, 1.0};
    std::map<int, std::array<double, 3>> dict{{0,{0.016, 1, 0.3}},{1,{0.016, 1, 0.3}},{2,{0.016, 1, 0.3}},{3,{0.016, 1, 0.3}},
                                            {4,{0.12, 0.6, 0.3}},{5,{0.12, 0.6, 0.3}},{6,{0.3, 0.6, 0.12}},{7,{0.3, 0.6, 0.12}},
                                            {8,{0.3, 0.4, 0.12}},{9,{0.3, 0.4, 0.12}},{10,{0.12, 0.4, 0.3}},{11,{0.12, 0.4, 0.3}},};
//*******************************
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;
    
    /***************/
    cv::Point3d first;
    /******************/
    while (1){
        
        if(!vi.isWorking()){
            break;
        }

        begin = std::chrono::steady_clock::now();

        vi.getFrame(frame);
        // frame = cv::imread(path);

        if(frame.empty()) continue;


        Marks detected;

        process.convertGray(frame, gray);
        process.binarize(gray, gray, (double)threshold);

        detected.detectMarks(gray);
        cv::Mat detectedImage = detected.drawMarks(gray);
        detected.getVectors(sidelen, cameraMatrix, distCoeffs);  
        cv::imshow("Camera", detected.drawAxis(detectedImage, cameraMatrix, distCoeffs));
        std::vector<cv::Point3d> poses = detected.getCameraPose();

        std::vector<cv::Point3d> filtered = filter(buffPose, poses);
        buffPose = filtered;
        for(int i=0;i<filtered.size();i++){
            std::cout<<detected.getIds()[i]<<std::endl<<filtered[i].x<<std::endl
            <<filtered[i].y<<std::endl<<filtered[i].z<<std::endl<<std::endl;

            std::vector<cv::Point3d> refposes = detected.getReferencePoses(filtered);
            // std::stringstream ss;
            // ss<<detected.getIds()[i]<<'\n'<<refposes[i].x<<'\n'<<refposes[i].y<<'\n'<<refposes[i].z<<'\n';
            // mark_pose.data = ss.str();
            // pub.publish(mark_pose);
            mark_pose.header.frame_id = detected.getIds()[i];
            mark_pose.point.x = refposes[i].x;
            mark_pose.point.y = refposes[i].y;
            mark_pose.point.z = refposes[i].z;
            pub.publish(mark_pose);
            }
        
        detected.sendPose(filtered);

        Marks detectedRobot;



        
        // bool work = detected.sendShapes(filtered,dict,nh);

        // if (!work){
        //     return 0;
        // }



        // std::vector<cv::Point3d> refposes = detected.getReferencePoses(filtered);


        // Marks new_detected(detected);

        // new_detected.getReferenceVecs();

        // new_detected.sendPose(refposes);




        // bool work = new_detected.sendShapes(refposes,dict,nh);

        // if (!work){
        //     return 0;
        // }

        // std::vector<std::array<cv::Point3d, 4>> shapeCorners = detected.shapesCorners(dict);

        // int width=2000, height=2000;
        // cv::Size size(width, height);
        // cv::Mat map(size, CV_64FC3, cv::Scalar(255,255,255));


        // for(int i=0; i<shapeCorners.size();i++){
        //     cv::line(map, cv::Point2d((width/10*9)-shapeCorners[i][0].x*(width/5), (height/2)+shapeCorners[i][0].y*(height/5)), cv::Point2d((width/10*9)-shapeCorners[i][1].x*(width/5), (height/2)+shapeCorners[i][1].y*(height/5)), cv::Scalar(0,0,0), 2);
        //     cv::line(map, cv::Point2d((width/10*9)-shapeCorners[i][1].x*(width/5), (height/2)+shapeCorners[i][1].y*(height/5)), cv::Point2d((width/10*9)-shapeCorners[i][2].x*(width/5), (height/2)+shapeCorners[i][2].y*(height/5)), cv::Scalar(0,0,0), 2);
        //     cv::line(map, cv::Point2d((width/10*9)-shapeCorners[i][2].x*(width/5), (height/2)+shapeCorners[i][2].y*(height/5)), cv::Point2d((width/10*9)-shapeCorners[i][3].x*(width/5), (height/2)+shapeCorners[i][3].y*(height/5)), cv::Scalar(0,0,0), 2);
        //     cv::line(map, cv::Point2d((width/10*9)-shapeCorners[i][3].x*(width/5), (height/2)+shapeCorners[i][3].y*(height/5)), cv::Point2d((width/10*9)-shapeCorners[i][0].x*(width/5), (height/2)+shapeCorners[i][0].y*(height/5)), cv::Scalar(0,0,0), 2);
        //     cv::putText(map, std::to_string(detected.getIds()[i]), cv::Point2d((width/10*9)-shapeCorners[i][1].x*(width/5), (height/2)+shapeCorners[i][0].y*(height/5)), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 3);
        // }

        // cv::namedWindow("rectangles", cv::WINDOW_NORMAL);
        // cv::imshow("rectangles", map);


 

        end = std::chrono::steady_clock::now();
        std::cout << "Processing time(ms)= " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;

        if (cv::waitKey(1) == 27) break;
        ros::spinOnce();
    }





 
    return 0;
}

// 141
// 0.89994
// 1.43528
// 1.4887

// 140
// 0.865645
// 2.60084
// 1.29088

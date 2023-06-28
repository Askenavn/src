#include "FiducialPositioningSystem.h"


VideoInput::VideoInput(ros::NodeHandle nh, int camId):
    nh(nh),
    drServer(this->nh){

    videoCapture.open(camId, cv::CAP_V4L2 );
    videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    drServer.setCallback(boost::bind(&VideoInput::setupVideoCapture, this, _1, _2));


    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    videoCapture.set(cv::CAP_PROP_FPS, 30);
}

VideoInput::~VideoInput(){
    videoCapture.release();
}

void VideoInput::getFrame(cv::Mat& frame) {
    videoCapture.read(frame);
}

bool VideoInput::isWorking(){
    return videoCapture.isOpened();
}

void VideoInput::showParams(){
    ROS_INFO("FPS: %d", (int)videoCapture.get(cv::CAP_PROP_FPS));
    ROS_INFO("resolution: %dx%d", (int)videoCapture.get(cv::CAP_PROP_FRAME_WIDTH),(int)videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("brightness: %d", (int)videoCapture.get(cv::CAP_PROP_BRIGHTNESS));
    ROS_INFO("contrast: %d", (int)videoCapture.get(cv::CAP_PROP_CONTRAST));
    ROS_INFO("saturation: %d", (int)videoCapture.get(cv::CAP_PROP_SATURATION));
}

void VideoInput::setupVideoCapture(fid_ps::VideoInputConfig& config, uint32_t level) {
    ROS_INFO("Received reconfiguration request");

    videoCapture.set(cv::CAP_PROP_FPS, config.fps);
    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    videoCapture.set(cv::CAP_PROP_BRIGHTNESS, config.brightness);
    videoCapture.set(cv::CAP_PROP_CONTRAST, config.contrast);
    videoCapture.set(cv::CAP_PROP_SATURATION, config.saturation);

    if(this->isWorking()) this->showParams();
}


void VideoProcess::convertGray(cv::Mat& rgb, cv::Mat& gray){
    cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
}

void VideoProcess::binarize(cv::Mat& gray, cv::Mat& wb, double threshold){
    cv::threshold(gray, wb, threshold, 255, cv::THRESH_BINARY);
}

void VideoProcess::addTrackbar(const char* winname, int& threshold){
    cv::createTrackbar("Threshold", winname, &threshold, 255);
}


std::vector<int> Marks::getIds(){
    return ids;
}


void Marks::detectMarks(cv::Mat frame){
   std::vector<int> markerIds;
   std::vector<std::vector<cv::Point2f>> markerCorners;

   cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
   cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

   cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

   this->ids = markerIds;
   this->corners = markerCorners;
}

cv::Mat Marks::drawMarks(cv::Mat frame){
   cv::aruco::drawDetectedMarkers(frame, this->corners, this->ids);

   return frame;      
}

void Marks::getVectors(cv::Mat cameraMatrix, cv::Mat distCoeffs){
   cv::aruco::estimatePoseSingleMarkers(this->corners, 0.175, cameraMatrix, distCoeffs, this->rvecs, this->tvecs);
}

cv::Mat Marks::drawAxis(cv::Mat frame, cv::Mat cameraMatrix, cv::Mat distCoeffs){
   cv::Mat img = frame.clone();
   for (int i=0; i < this->rvecs.size(); i++){
      cv::drawFrameAxes(img, cameraMatrix, distCoeffs, this->rvecs[i],this->tvecs[i], 0.11);
   }
   return img;
}

void Marks::sendPose(std::vector<cv::Point3d> poses){
   static tf2_ros::StaticTransformBroadcaster br;
   for(int i=0; i<poses.size();i++){

      geometry_msgs::TransformStamped pose;

      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.child_frame_id = std::to_string(ids[i]);
      pose.transform.translation.x = poses[i].x;
      pose.transform.translation.y = poses[i].y;
      pose.transform.translation.z = poses[i].z;
      tf2::Quaternion q;
      q.setRotation(tf2::Vector3((double)rvecs[i](0), (double)rvecs[i](1), (double)rvecs[i](2)), 
               sqrt(rvecs[i](0)*rvecs[i](0) + rvecs[i](1)*rvecs[i](1) + rvecs[i](2)*rvecs[i](2)));
      pose.transform.rotation.x = q.x();
      pose.transform.rotation.y = q.y();
      pose.transform.rotation.z = q.z();
      pose.transform.rotation.w = q.w();

      if(((double)rvecs[i](0)==0)&((double)rvecs[i](1)==0)){
         pose.transform.rotation.x = 0.;
         pose.transform.rotation.y = 0.;
         pose.transform.rotation.z = 0.;
         pose.transform.rotation.w = 1.;
      }

      // pose.transform.rotation.x = 0;
      // pose.transform.rotation.y = 0;
      // pose.transform.rotation.z = 0;
      // pose.transform.rotation.w = 1;


      br.sendTransform(pose);
   }
}


bool Marks::sendShapes(std::vector<cv::Point3d> poses, std::map<int, std::array<double, 3>> dict, ros::NodeHandle& n){
   
   static ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

   unsigned int shape = visualization_msgs::Marker::CUBE;
   int count = poses.size();
   visualization_msgs::MarkerArray markerArray;

   
   for(int i=0; i<poses.size();i++){

      visualization_msgs::Marker marker;

      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "map";
      
      marker.id = ids[i];
      marker.ns = "basic_shapes";
      marker.type = shape;
      
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = poses[i].x;
      marker.pose.position.y = poses[i].y;
      // marker.pose.position.z = poses[i].z-(dict[ids[i]][2]/2);
      marker.pose.position.z = 0+(dict[ids[i]][2]/2);
      
      tf2::Quaternion q;
      q.setRotation(tf2::Vector3((double)rvecs[i](0), (double)rvecs[i](1), (double)rvecs[i](2)), 
               sqrt(rvecs[i](0)*rvecs[i](0) + rvecs[i](1)*rvecs[i](1) + rvecs[i](2)*rvecs[i](2)));
      

      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      if(((double)rvecs[i](0)==0)&((double)rvecs[i](1)==0)){
         marker.pose.orientation.x = 0.;
         marker.pose.orientation.y = 0.;
         marker.pose.orientation.z = 0.;
         marker.pose.orientation.w = 1.;
      }

      marker.scale.x = dict[ids[i]][0];
      marker.scale.y = dict[ids[i]][1];
      marker.scale.z = dict[ids[i]][2];

      double r = 255, g = 140, b = 0;

      marker.color.r = (r/256);
      marker.color.g = (g/256);
      marker.color.b = (b/256);
      marker.color.a = 1.0;


      marker.lifetime = ros::Duration();

      markerArray.markers.push_back(marker);
   }
   

   while (markers_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
         return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
   }
   markers_pub.publish(markerArray);


   return true;
}


std::vector<cv::Point3d> Marks::getCameraPose(){
   std::vector<cv::Point3d> poses(ids.size());

   for(int k=0; k<ids.size();k++){   
   cv::Mat rotMat;
   cv::Mat transMat = cv::Mat::zeros(4, 4, CV_64F);
   cv::Rodrigues(this->rvecs[k], rotMat);

   for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
         transMat.at<double>(i,j)=rotMat.at<double>(i,j);
      }
   } 

   transMat.at<double>(0,3) = this->tvecs[k](0);
   transMat.at<double>(1,3) = this->tvecs[k](1);
   transMat.at<double>(2,3) = this->tvecs[k](2);
   transMat.at<double>(3,3) = 1.;
   
   cv::Mat invMat = transMat.inv();
   // poses[k] = cv::Point3d(invMat.at<double>(0,3), invMat.at<double>(1,3), invMat.at<double>(2,3));
   poses[k] = cv::Point3d(transMat.at<double>(0,3), transMat.at<double>(1,3), transMat.at<double>(2,3));
   }
   
   return poses;
}


std::vector<cv::Point3d> Marks::getReferencePoses(std::vector<cv::Point3d> poses){
   std::vector<cv::Point3d> refposes(ids.size());

   cv::Mat rotMat;
   cv::Mat transMat = cv::Mat::zeros(4, 4, CV_64F);
   cv::Rodrigues(this->rvecs[0], rotMat);

   for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
         transMat.at<double>(i,j)=rotMat.at<double>(i,j);
      }
   }

   transMat.at<double>(0,3) = this->tvecs[0](0);
   transMat.at<double>(1,3) = this->tvecs[0](1);
   transMat.at<double>(2,3) = this->tvecs[0](2);
   transMat.at<double>(3,3) = 1.;
   
   cv::Mat invMat = transMat.inv();

   for(int k=0; k<this->ids.size();k++){

      cv::Vec4d curr(poses[k].x, poses[k].y, poses[k].z, 1.);
      cv::Mat coord = invMat*cv::Mat(curr);

      refposes[k] = cv::Point3d(coord.at<double>(0), coord.at<double>(1), coord.at<double>(2));
   }
   
   return refposes;
}

void Marks::getReferenceVecs(){
   std::vector<cv::Vec3d> refrvecs=this->rvecs;

   cv::Mat rotMat;
   cv::Mat transMat = cv::Mat::zeros(4, 4, CV_64F);
   cv::Mat absMat;

   cv::Rodrigues(this->rvecs[0], absMat);

   this->rvecs[0](0)=0,0000000001;
   this->rvecs[0](1)=0,0000000001;
   this->rvecs[0](2)=0,0000000001;
 
   // std::cout<<this->ids[0]<<std::endl;
   // std::cout<<rvecs[0]<<std::endl;
   // std::cout<<std::endl;

   for(int k=1; k<rvecs.size();k++){
      cv::Rodrigues(this->rvecs[k], rotMat);

      cv::Mat newRotMat = rotMat.inv()*absMat;
      cv::Rodrigues(newRotMat, this->rvecs[k]); 
      // std::cout<<this->ids[k]<<std::endl;
      // std::cout<<rvecs[k]<<std::endl;
      // std::cout<<std::endl;
   }



}

std::vector<std::array<cv::Point3d, 4>> Marks::shapesCorners(std::map<int, std::array<double, 3>> dict){
   cv::Mat absMat;
   cv::Mat absTrMat = cv::Mat::zeros(4, 4, CV_64F);
   cv::Rodrigues(this->rvecs[0], absMat);

   for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
         absTrMat.at<double>(i,j)=absMat.at<double>(i,j);
      }
   }

   absTrMat.at<double>(0,3) = this->tvecs[0](0);
   absTrMat.at<double>(1,3) = this->tvecs[0](1);
   absTrMat.at<double>(2,3) = this->tvecs[0](2);
   absTrMat.at<double>(3,3) = 1.;

   cv::Mat invMat = absTrMat.inv();
   
   std::vector<std::array<cv::Point3d, 4>> corners;
   cv::Mat corner;
   std::array<cv::Point3d, 4> arr;

   corner = cv::Mat(cv::Vec4d(dict[ids[0]][0]/2, dict[ids[0]][1]/2, 0, 1));
   arr[0] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

   std::cout<<corner<<std::endl;

   corner = cv::Mat(cv::Vec4d(-dict[ids[0]][0]/2, dict[ids[0]][1]/2, 0, 1));
   arr[1] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

   std::cout<<corner<<std::endl;

   corner = cv::Mat(cv::Vec4d(-dict[ids[0]][0]/2, -dict[ids[0]][1]/2, 0, 1));
   arr[2] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

   std::cout<<corner<<std::endl;

   corner = cv::Mat(cv::Vec4d(dict[ids[0]][0]/2, -dict[ids[0]][1]/2, 0, 1));
   arr[3] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

   std::cout<<corner<<std::endl;

   corners.push_back(arr);


   for(int k=1; k<this->ids.size(); k++){
      cv::Mat rotMat;
      cv::Mat transMat = cv::Mat::zeros(4, 4, CV_64F);
      cv::Rodrigues(this->rvecs[k], rotMat);

      for(int i=0;i<3;i++){
         for(int j=0;j<3;j++){
            transMat.at<double>(i,j)=rotMat.at<double>(i,j);
         }
      }

      transMat.at<double>(0,3) = this->tvecs[k](0);
      transMat.at<double>(1,3) = this->tvecs[k](1);
      transMat.at<double>(2,3) = this->tvecs[k](2);
      transMat.at<double>(3,3) = 1.;

     

      corner = invMat*transMat*cv::Mat(cv::Vec4d(dict[ids[k]][0]/2, dict[ids[k]][1]/2, 0, 1));
      arr[0] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

      std::cout<<corner<<std::endl;

      corner = invMat*transMat*cv::Mat(cv::Vec4d(-dict[ids[k]][0]/2, dict[ids[k]][1]/2, 0, 1));
      arr[1] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

      std::cout<<corner<<std::endl;

      corner = invMat*transMat*cv::Mat(cv::Vec4d(-dict[ids[k]][0]/2, -dict[ids[k]][1]/2, 0, 1));
      arr[2] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

      std::cout<<corner<<std::endl;

      corner = invMat*transMat*cv::Mat(cv::Vec4d(dict[ids[k]][0]/2, -dict[ids[k]][1]/2, 0, 1));
      arr[3] = cv::Point3d(corner.at<double>(0), corner.at<double>(1), corner.at<double>(2));

      std::cout<<corner<<std::endl;

      corners.push_back(arr);
   }
   return corners;
}



std::vector<cv::Point3d> filter( std::vector<cv::Point3d> prePoses, std::vector<cv::Point3d> curPoses){
   double coeff = 0.1;
   std::vector<cv::Point3d> filtered = curPoses;

   for(int i=0; i<curPoses.size(); i++){
      if(i>=prePoses.size()){
         filtered[i].x = coeff*curPoses[i].x + (1-coeff)*curPoses[i].x;
         filtered[i].y = coeff*curPoses[i].y + (1-coeff)*curPoses[i].y;
         filtered[i].z = coeff*curPoses[i].z + (1-coeff)*curPoses[i].z;
      }
      else{
         filtered[i].x = coeff*curPoses[i].x + (1-coeff)*prePoses[i].x;
         filtered[i].y = coeff*curPoses[i].y + (1-coeff)*prePoses[i].y;
         filtered[i].z = coeff*curPoses[i].z + (1-coeff)*prePoses[i].z;
      }
   }
   return filtered;
}

void Marks::setIds(std::vector<int> nums){
   this->ids=nums;
}

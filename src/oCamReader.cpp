//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 --rs -d=10 -dp="detector_params.yml" -c="outCameraCalib78.txt"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d.hpp> //add Rodrigues transformation
//#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>

//OCAM
#include "opencv2/opencv.hpp"
#include "withrobot_camera.hpp"	/* withrobot camera API */
//END OCAM

#include <string> 
#include <math.h>
#include <fstream>

using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
}

//// CAMERA
bool arucoProcess(Mat srcImg) {
	
   	//OCAM 
	Mat image, imageCopy;
	image = srcImg;
	image.copyTo(imageCopy);

	if (imageCopy.empty()){		
		return 0;
	} 	
	
	imshow("out", imageCopy);	
	//END OCAM 
		
    return true;
}

//////// ROS
cv::Mat srcImg;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{	 
 try
  {	
	cv::waitKey(5);
	//GRAB IMAGE from publisher node	
	srcImg = cv_bridge::toCvShare(msg, "bgr8")->image;
	arucoProcess(srcImg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//MAIN
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "image_listener");
  	ros::NodeHandle nh;		

	cv::namedWindow("view");
	cv::startWindowThread();
	
  	image_transport::ImageTransport it(nh);
	
	image_transport::Subscriber sub = it.subscribe("left/image_raw", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}

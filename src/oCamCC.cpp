//"CHANGE HERE" is for hardcoded parameters that should be changed

//#include "opencv2/aruco.hpp"
//#include "opencv2/aruco/dictionary.hpp"


//NOTES on 360, use calib file based on chosen resolution !!!
//cd ~/openCVmine/testROS/ocams/devel/lib/ocams
//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalibA3.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0  -usePOZYX=0 -useLIDAR=0
// ./mosse
//roscore
//cd ~/catkin_ws/
//catkin_make
//ifconfig
//rosrun beginner_tutorials talker.py
//ping 192.168.10.104
//ping 192.168.10.106
//sudo systemctl start isc-dhcp-server.service
//sudo systemctl restart isc-dhcp-server.service
//sudo systemctl enable isc-dhcp-server.service
//dhcpd -t
//ps -C dhcpd
//sudo service isc-dhcp-server restart
//service isc-dhcp-server status
//sudo /etc/init.d/networking restart
//sudo /etc/init.d/isc-dhcp-server restart

// cd ~/openCVmine/testROS/ocams/
// make all

//START LIDAR
// roslaunch velodyne_pointcloud VLP16_points.launch

//with LIDAR
// ./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib99.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0 --rs -usePOZYX=0 -useLIDAR=1 -PCDcloud="test.pcd"

/////// 360 CAMERA
// cd catkin_ws
// ~/catkin_ws/devel/setup.bash
// rosrun beginner_tutorials talker.py

// cd ~/openCVmine/testROS/ocams/devel/lib/ocams
// ./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalibA3.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0  -usePOZYX=0 -useLIDAR=0
// ./ocams -c1 _exposure=60

//#include "GPU360Rectifier.h"


//ADD LiDAR - https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/

#include <ros/ros.h>
#include <boost/foreach.hpp>
 
int camWidth = 2560; //1920;//3840; //  - 
int camHeight = 1440; //1080;//2160;

//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 --rs -d=10 -dp="detector_params.yml" -c="outCameraCalib78.txt"
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

//#include <vector>
//#include <iostream>

//OCAM
//#include "withrobot_camera.hpp"	/* withrobot camera API */
//END OCAM

#include <string> 
#include <math.h>
#include <fstream>
#include <time.h>
time_t start = time(0);//clock();//

//ARDUINO COM READ
#include <stdlib.h>
#include <stdio.h>
//#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
//#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#define DEBUG 1
#include <thread>


//#include <opencv2/highgui/highgui.hpp>



//using namespace cv::aruco;
#include <cv_bridge/cv_bridge.h>

//#include <opencv2/aruco.hpp>
//#include <opencv2/aruco/dictionary.hpp>

#include <opencv2/highgui.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp> //add Rodrigues transformation
using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =	
	"{HorAngle |       | Measured horizontal angle }"
	"{VerAngle |       | Measured vertical angle }"
	"{RealDist |       | Measured distance from solid center }"
	"{af       |       | 0 for no preview, 1 for preview window }"
	"{shape    |       | 0 for 21edron, 1 for Cube }"
	"{p        |       | 0 for no preview, 1 for preview window }"
	"{usePOZYX |       | 0 no POZYX, 1 enable POZYX}"
	"{useLIDAR |       | 0 no LIDAR, 1 enable LIDAR}"
	"{PCDcloud |       | Point cloud for reference 3D object}"
	"{g        |       | 0 for color, 1 for grey }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

//// CAMERA

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    //fs["cornerRefinementMethod"] >> params->cornerRefinementMethod; //OCAM not supported in ROS openCV ?
//fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;    
    return true;
}

//MATRIX FUNCTIONS 3 x 1
Vec3d matrix31_abs(Vec3d inputmatrix){
	inputmatrix(0) = abs(inputmatrix(0));
	inputmatrix(1) = abs(inputmatrix(1));
	inputmatrix(2) = abs(inputmatrix(2));
	return inputmatrix;
}
void zero31_matrix(Vec3d &rotResultC){
	rotResultC(0) = 0;
	rotResultC(1) = 0;
	rotResultC(2) = 0;	
}
void matrix32_assign_from_to(Vec3d assignFrom, Vec3d &assignTo){
	assignTo(0) = assignFrom(0);
	assignTo(1) = assignFrom(1);
	assignTo(2) = assignFrom(2);	
}
void add_to_31_matrix(Vec3d &rotResultCTMP, Vec3d add_this){
	rotResultCTMP(0) = rotResultCTMP(0) + add_this(0);
	rotResultCTMP(1) = rotResultCTMP(1) + add_this(1);
	rotResultCTMP(2) = rotResultCTMP(2) + add_this(2);		
}

void scale31_matrix(Vec3d &rotResultCTMP, float divider)
{
	rotResultCTMP(0) = rotResultCTMP(0) / divider;
	rotResultCTMP(1) = rotResultCTMP(1) / divider;
	rotResultCTMP(2) = rotResultCTMP(2) / divider;	
}

float matrix31_distance(Vec3d centerRotationsTMP, Vec3d centerRotationsTMP2){
	float center_diffX1 = centerRotationsTMP(0) - centerRotationsTMP2(0);		
	float center_diffX2 = centerRotationsTMP(1) - centerRotationsTMP2(1);		
	float center_diffX3 = centerRotationsTMP(2) - centerRotationsTMP2(2);	
	float Center_to_Prev_Center_dist = sqrt(pow(center_diffX1,2) + pow(center_diffX2,2) + pow(center_diffX3,2));
	return Center_to_Prev_Center_dist;
}

//MATRIX FUNCTIONS 3 x 3
void zero33_matrix(Mat &rotResultC){
	rotResultC.at<double>(0,0) = 0;
	rotResultC.at<double>(0,1) = 0;
	rotResultC.at<double>(0,2) = 0;
	rotResultC.at<double>(1,0) = 0;
	rotResultC.at<double>(1,1) = 0;
	rotResultC.at<double>(1,2) = 0;
	rotResultC.at<double>(2,0) = 0;
	rotResultC.at<double>(2,1) = 0;
	rotResultC.at<double>(2,2) = 0;
}

void add_to_33_matrix(Mat &rotResultCTMP, Mat add_this){
	rotResultCTMP.at<double>(0,0) = rotResultCTMP.at<double>(0,0) + add_this.at<double>(0,0);
	rotResultCTMP.at<double>(0,1) = rotResultCTMP.at<double>(0,1) + add_this.at<double>(0,1);
	rotResultCTMP.at<double>(0,2) = rotResultCTMP.at<double>(0,2) + add_this.at<double>(0,2);
	rotResultCTMP.at<double>(1,0) = rotResultCTMP.at<double>(1,0) + add_this.at<double>(1,0);
	rotResultCTMP.at<double>(1,1) = rotResultCTMP.at<double>(1,1) + add_this.at<double>(1,1);
	rotResultCTMP.at<double>(1,2) = rotResultCTMP.at<double>(1,2) + add_this.at<double>(1,2);
	rotResultCTMP.at<double>(2,0) = rotResultCTMP.at<double>(2,0) + add_this.at<double>(2,0);
	rotResultCTMP.at<double>(2,1) = rotResultCTMP.at<double>(2,1) + add_this.at<double>(2,1);
	rotResultCTMP.at<double>(2,2) = rotResultCTMP.at<double>(2,2) + add_this.at<double>(2,2);
	//return rotResultCTMP;
}

void scale33_matrix(Mat &rotResultCTMP, float divider)
{
	rotResultCTMP.at<double>(0,0) = rotResultCTMP.at<double>(0,0) / divider;
	rotResultCTMP.at<double>(0,1) = rotResultCTMP.at<double>(0,1) / divider;
	rotResultCTMP.at<double>(0,2) = rotResultCTMP.at<double>(0,2) / divider;
	rotResultCTMP.at<double>(1,0) = rotResultCTMP.at<double>(1,0) / divider;
	rotResultCTMP.at<double>(1,1) = rotResultCTMP.at<double>(1,1) / divider;
	rotResultCTMP.at<double>(1,2) = rotResultCTMP.at<double>(1,2) / divider;
	rotResultCTMP.at<double>(2,0) = rotResultCTMP.at<double>(2,0) / divider;
	rotResultCTMP.at<double>(2,1) = rotResultCTMP.at<double>(2,1) / divider;
	rotResultCTMP.at<double>(2,2) = rotResultCTMP.at<double>(2,2) / divider;
}

float matrix33_distance(Mat centerRotationsTMP, Mat centerRotationsTMP2){
	float center_diffX1 = centerRotationsTMP.at<double>(0,0) - centerRotationsTMP2.at<double>(0,0);		
	float center_diffX2 = centerRotationsTMP.at<double>(0,1) - centerRotationsTMP2.at<double>(0,1);		
	float center_diffX3 = centerRotationsTMP.at<double>(0,2) - centerRotationsTMP2.at<double>(0,2);		
	float center_diffY1 = centerRotationsTMP.at<double>(1,0) - centerRotationsTMP2.at<double>(1,0);		
	float center_diffY2 = centerRotationsTMP.at<double>(1,1) - centerRotationsTMP2.at<double>(1,1);		
	float center_diffY3 = centerRotationsTMP.at<double>(1,2) - centerRotationsTMP2.at<double>(1,2);	
	float center_diffZ1 = centerRotationsTMP.at<double>(2,0) - centerRotationsTMP2.at<double>(2,0);		
	float center_diffZ2 = centerRotationsTMP.at<double>(2,1) - centerRotationsTMP2.at<double>(2,1);		
	float center_diffZ3 = centerRotationsTMP.at<double>(2,2) - centerRotationsTMP2.at<double>(2,2);
	float Center_to_Prev_Center_dist = sqrt(pow(center_diffX1,2) + pow(center_diffX2,2) + pow(center_diffX3,2) 
	+ pow(center_diffY1,2) + pow(center_diffY2,2) + pow(center_diffY3,2) 
	+ pow(center_diffZ1,2) + pow(center_diffZ2,2) + pow(center_diffZ3,2));
	return Center_to_Prev_Center_dist;
}
Mat rotation_matrix_Z_rad(float rad){
 	return (Mat_<double>(4,4) << 
	cos(rad),-sin(rad),0,0, 
	sin(rad),cos(rad),0,
	0,0,0,1,0, 
	0,0,0,1);
}
Mat rotation_matrix_Y_rad(float rad){
 	return (Mat_<double>(4,4) << 
	cos(rad),0,sin(rad),0,   
	0,1,0,0,   
	-sin(rad),0,cos(rad),0,    
	0,0,0,1);
}
Mat rotation_matrix_X_rad(float rad){
 	return (Mat_<double>(4,4) << 
	1,0,0,0,  
	0,cos(rad),-sin(rad),0,   
	0,sin(rad),cos(rad),0, 
	0,0,0,1);
}

//float PI = 3.14159265f;
float PI   = 3.141592653589793238462643383279502884;//19716939937510582097494459230781640628f;
//http://www.cplusplus.com/forum/beginner/44774/
//Float	4 1.8E-38	3.4E+38
//Double	8 2.2E-308	1.8E+308
//Long Double	8 2.2E-308	1.8E+308

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{ 
    //assert(isRotationMatrix(R));     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) ); 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);     
}


//TIMER
int totalIterationsTimer=0;
struct timeval beginT, endT;
double REAL_seconds_since_start=0;
double REAL_PREV_seconds_since_start=0;
int REAL_PREV_POZYX_samples = 0;

//// COMBINATIONS
//METHOD 1 //find combinations and choose the group with least members (as it will not contain the offending outlier)		
vector<int> outlierComboIDs;
int comboIteration;
vector<int> foundIDs;
vector<int> combination;
vector<vector<int>> combinations;
int errorID = -1;
int comboDepthLevel=0;

void generateCobminations(int offset, int k) {
	if (k == 0) {			    
		combinations.push_back(combination);
		return;
	}
	for (int i = offset; i <= foundIDs.size() - k; ++i) {
		combination.push_back(foundIDs[i]);
		generateCobminations(i+1, k-1);
		combination.pop_back();
	}
}

void discoverOutliersMinMax( vector<cv::Vec<double, 3>> &centerPointsTMP, vector<int> &centerPointsIDsTMP, std::vector<cv::Mat> &centerRotationsTMP ){

				comboIteration += 0.01; //go to 1 in first round

				comboDepthLevel++; // increase depth				

				int outofThresCount = 0; //count how many out of threshold

				//find combinations and choose the group with least members (as it will not contain the offending outlier)				
				float distThreshold = 0.04f; //0.001f;//0.015 - comboIteration; //0.014f; //5cm threshold to determine clusters //CHANGE HERE
				int nCombinations = centerPointsIDsTMP.size(); //ids.size(); //centerPointsIDs , centerPoints
				int kCombinations = nCombinations-1; //start with max, reduce up to 2

				foundIDs.clear();//reset ids with new ones
				combinations.clear();
				combination.clear();

				int biggestDistID = 0;
				int smallerDistID = 0;
				float currentBiggestDist = 0.0f; //initialize with a threshold
				float currentSmallerDist = 100000; ////CHANGE HERE
				for (int i = 0; i < nCombinations; ++i) 
				{ 
					foundIDs.push_back(i); //foundIDs.push_back(ids[i]); 
				}
				generateCobminations(0, kCombinations);

				//cout << "ids = " << foundIDs.size() << endl; 
				//cout << " Combinations = " << combinations.size() << endl; 

				//ROTATIONS				
				//cout << "ROTATIONS SIZE = " << centerRotationsTMP.size() << endl; 

				if(centerRotationsTMP.size() < 3){ return; }							

				for (int i = 0; i < combinations.size(); i++) { //for (int i = 0; i < combinations.size()-1; i++) {
					//find center average
					Vec3d center1;
					Vec3d center1ROT;
					for (int i1 = 0; i1 < combinations[i].size(); i1++) {
						add_to_31_matrix(center1,centerPointsTMP[combinations[i][i1]]);						
						
						Vec3d toEuler = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[i][i1]])*(180.0f/PI);	
						//cv::Mat R;
						//cv::Rodrigues(centerRotationsTMP[combinations[i][i1]],R);

						add_to_31_matrix(center1ROT, matrix31_abs(toEuler));						
						//cout << "toEuler Angles = " << toEuler << endl; 
					}
					scale31_matrix(center1, combinations[i].size());					
					scale31_matrix(center1ROT, combinations[i].size());					
					//cout << "toEuler Angles CENTER = " << center1ROT << endl; 
				
					float currentDist = 100000000;	//find distance to all others
					for (int j = 0; j < combinations.size(); j++) { //for (int j = i+1; j < combinations.size(); j++) {
						//find center average
						Vec3d center2;
						Vec3d center2ROT;
						for (int j1 = 0; j1 < combinations[j].size(); j1++) {
							add_to_31_matrix(center2,centerPointsTMP[combinations[j][j1]]);
							Vec3d toEuler1 = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[j][j1]])*(180.0f/PI);
							add_to_31_matrix(center2ROT, matrix31_abs(toEuler1));								
						}
						scale31_matrix(center2, combinations[j].size());						
						scale31_matrix(center2ROT, combinations[j].size());						

						float Center_to_Cam_distA = matrix31_distance(center1ROT,center2ROT);							
						
						//cout << "Center_to_Cam_distA AAAAAA= " << Center_to_Cam_distA << endl; 
						
						//MIN MAX
						//currentDist = currentDist + Center_to_Cam_distA;
						if(Center_to_Cam_distA < currentDist && Center_to_Cam_distA != 0){ //REMOVE CASE WHERE WE CHECK SAME DATA
							currentDist = Center_to_Cam_distA;
						}						
							
					}//end inner for loop (check with all other combinations centers)

					if(currentDist > currentBiggestDist){
						currentBiggestDist = currentDist;
						biggestDistID = i;					
					}
					if(currentDist < currentSmallerDist){
						currentSmallerDist = currentDist;
						smallerDistID = i;					
					}
					if(currentBiggestDist/(combinations.size()-1) > distThreshold){
						outofThresCount++;					
					}				
				}//end for loop	


				//CHECK SECOND BIGGEST DISTANCE TO BIGGEST, if close assume more than one errorneous
				int found =0;
				for (int i = 0; i < combinations.size(); i++) {
					//find center average
					Vec3d center1;
					Vec3d center1ROT;
					for (int i1 = 0; i1 < combinations[i].size(); i1++) {
						add_to_31_matrix(center1,centerPointsTMP[combinations[i][i1]]);						
						
						Vec3d toEuler = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[i][i1]])*(180.0f/PI);	
						//cv::Mat R;
						//cv::Rodrigues(centerRotationsTMP[combinations[i][i1]],R);
						add_to_31_matrix(center1ROT, matrix31_abs(toEuler));						
						//cout << "toEuler Angles = " << toEuler << endl; 
					}
					scale31_matrix(center1, combinations[i].size());					
					scale31_matrix(center1ROT, combinations[i].size());					
					//cout << "toEuler Angles CENTER = " << center1ROT << endl; 
				
					float currentDist = 0;	//find distance to all others
					for (int j = 0; j < combinations.size(); j++) { //for (int j = i+1; j < combinations.size(); j++) {
						//find center average
						Vec3d center2;
						Vec3d center2ROT;
						for (int j1 = 0; j1 < combinations[j].size(); j1++) {
							add_to_31_matrix(center2,centerPointsTMP[combinations[j][j1]]);
							Vec3d toEuler1 = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[j][j1]])*(180.0f/PI);
							add_to_31_matrix(center2ROT, matrix31_abs(toEuler1));								
						}
						scale31_matrix(center2, combinations[j].size());						
						scale31_matrix(center2ROT, combinations[j].size());						

						float Center_to_Cam_distA = matrix31_distance(center1ROT,center2ROT);						
						
						//MIN MAX
						//currentDist = currentDist + Center_to_Cam_distA;
						if(Center_to_Cam_distA < currentDist){
							currentDist = Center_to_Cam_distA;
						}						
							
					}//end inner for loop (check with all other combinations centers)
					
					//CHANGE HERE
					//if distance above threshold look to see how far biggest distances are from the biggest found above
					if(abs(currentDist - currentBiggestDist) < 45.04f){ 
						found++;					
					}
				}	

				//if max distance average above a threshold, assume we have one error, 
				//otherwise assume no erroneous marker measurement or move to find 2-3-4 etc erroneous combos
				//if(currentBiggestDist/ (combinations.size()) > 0.17f){ //  if(currentBiggestDist/ (combinations.size()-1) > 0.05f){
				//if(outofThresCount > 0 && currentBiggestDist > distThreshold){
				//if(outofThresCount > 0 && currentBiggestDist/(combinations.size()-1) > distThreshold){
				//if(currentBiggestDist/(combinations.size()-1) > distThreshold){		
					//find erroneous marker
					int errorIter = -1;
					for (int i = 0; i < centerPointsIDsTMP.size(); i++) {//search the pool of all marker IDs found for the erroneous
						bool found1 = false;
						for (int j = 0; j < combinations[biggestDistID].size(); j++) {
							if(combinations[biggestDistID][j] == centerPointsIDsTMP[i]){
								found1 = true;
							}
						}
						if(!found1){ //if centerPointsID not in the max distance group, it is the erroneous one
							errorID = centerPointsIDsTMP[i];
							errorIter = i;
							break;
						}					
					}

					//Centers comparisson with and without the outlier
					//MIN MAX - if center with outlier VS center without distance lower than threshold, stop

					vector<int> centerPointsIDsORIG = centerPointsIDsTMP;
					vector<cv::Vec<double, 3>>  centerPointsORIG = centerPointsTMP;
					//std::vector<cv::Mat> centerRotationsORIG = centerRotationsTMP;
					centerPointsIDsORIG.erase(centerPointsIDsORIG.begin() + errorIter);
					centerPointsORIG.erase(centerPointsORIG.begin() + errorIter);

					cv::Vec<double, 3> centerMean;					
					for (int i = 0; i < centerPointsIDsORIG.size(); i++) {
						add_to_31_matrix(centerMean, centerPointsORIG[i]);					
					}
					scale31_matrix(centerMean,centerPointsORIG.size());
					
					if(centerPointsIDsORIG.size() == 1){
						matrix32_assign_from_to(centerPointsORIG[0],centerMean);
					}

					cv::Vec<double, 3> centerMean1;					
					for (int i = 0; i < centerPointsIDsTMP.size(); i++) {
						add_to_31_matrix(centerMean1, centerPointsTMP[i]);							
					}
					scale31_matrix(centerMean1, centerPointsIDsTMP.size());
					
					if(centerPointsIDsTMP.size() == 1){
						matrix32_assign_from_to(centerPointsTMP[0],centerMean1);			
					}					
					
					float centersWithAndWithoutDistance = matrix31_distance(centerMean,centerMean1);
					
					//if(nCombinations < 3){  //if(centersWithAndWithoutDistance > distThreshold){
					//if(centersWithAndWithoutDistance > distThreshold){
					//if(nCombinations > 3 && centersWithAndWithoutDistance > distThreshold){
					//if(nCombinations > 3){ //if(nCombinations > 2 && centersWithAndWithoutDistance > distThreshold){

					// if min to max distance vs threshold small, do no action
					if (abs(currentBiggestDist - currentSmallerDist) < 2.04f){ //CHANGE HERE
						// NO ACTION
						//if(preview == 1){
							//cout << "currentBiggestDist = " << currentBiggestDist 
							//<< " currentSmallerDist = " << currentSmallerDist << endl; 
						//}
					}else if (found == 1){
						//cout << "LAST ERROR ID = " << errorID << " MAX DISTANCE = " 
						//<< currentBiggestDist << " DISTANT COUNT = " << outofThresCount << endl;
 
						outlierComboIDs.push_back(errorID);

						//reconstruct the matrices without the found erroneous marker
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + errorIter);
						centerPointsTMP.erase(centerPointsTMP.begin() + errorIter);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + errorIter);
					}else if(1==1){
						biggestDistID = 0;
						float currentMaxCenterDist = 0;
						//CHOOSE HIGEST DIST POINT - ROTATIONS
						for (int i = 0; i < centerRotationsTMP.size(); ++i) 
						{ 
							//CHOOSE HIGEST DIST POINT - ROTATIONS
							float toAllDist = 0;
							for (int j = 0; j < centerRotationsTMP.size(); ++j) 
							{
							   float Center_to_Prev_Center_dist = matrix33_distance(centerRotationsTMP[j], centerRotationsTMP[i]);
							   toAllDist = toAllDist + Center_to_Prev_Center_dist;								
							}

							//distROT = centerRotationsTMP[i].at<double>(0,0);
							if(toAllDist > currentMaxCenterDist){
							   currentMaxCenterDist = toAllDist;
							   biggestDistID = i;					
							}
						}

						//WE ASSUME AL LEAST TWO ERRONEUS HERE, choose how to eliminate one so we iterate the rest
						//cout << "ASSUMED ERROR ID = " << centerPointsIDsTMP[biggestDistID] << endl; 
						outlierComboIDs.push_back(centerPointsIDsTMP[biggestDistID]);

						//reconstruct the matrices without the found erroneous marker
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + biggestDistID);
						centerPointsTMP.erase(centerPointsTMP.begin() + biggestDistID);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + biggestDistID);

						//cout << "ASSUMED ERROR ID A = " << centerPointsIDsTMP[biggestDistID] << endl; 
						discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP );
						//cout << "ASSUMED ERROR ID B = " << comboDepthLevel << " _____" << centerPointsIDsTMP[biggestDistID] << endl; 
					}


					// if min to max distance vs threshold large and if only one distant, assing one erroneuous and stop

					// if min to max distance vs threshold large and if more than one distant, search to remove one marker and loop
					
				//}
}//END FUNCTION 2 MIN MAX


//// ARUCO PARAMETERS
int markersX;
int markersY ;
float markerLength ;
float markerSeparation ;
int dictionaryId ;
bool showRejected ;
bool refindStrategy ;
int camId ;
int colorMode ;
int preview;
int autoFocus;
int usePOZYX;
int useLIDAR;
std::string PCDcloud;
int shapeID;
float HorAngle;
float VerAngle;
float RealDist;
Mat camMatrix, distCoeffs;
Ptr<aruco::DetectorParameters> detectorParams ;
String video;
Ptr<aruco::Dictionary> dictionary;
VideoCapture inputVideo;

int totalIterations = 0;



// Checks if a matrix is a valid rotation matrix.
	bool isRotationMatrixA(Mat &R)
	{
	    Mat Rt;
	    transpose(R, Rt);
	    Mat shouldBeIdentity = Rt * R;
	    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
	    return  norm(I, shouldBeIdentity) < 1e-6;
	}
	// Calculates rotation matrix to euler angles
	// The result is the same as MATLAB except the order
	// of the euler angles ( x and z are swapped ).
	Vec3f rotationMatrixToEulerAnglesA(Mat &R)
	{
	 
	    assert(isRotationMatrixA(R));
	    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
	    bool singular = sy < 1e-6; // If
	    float x, y, z;

	    if (!singular)
	    {
	        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
	        y = atan2(-R.at<double>(2,0), sy);
	        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
	    }
	    else
	    {
	        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
	        y = atan2(-R.at<double>(2,0), sy);
	        z = 0;
	    }
	    return Vec3f(x, y, z);

	}
// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta) //rotationMatrixToEulerAnglesA(Mat &R)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
    
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
    
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
    
    
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    
    return R;

}


//FILE OUTPUT
std::ofstream outfile;

//BRIGHTNESS - CONTRAST
double alpha; /**< Simple contrast control */
int beta;  /**< Simple brightness control */

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;     
}


bool arucoProcessInit() {	

	//Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	//Ptr<aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(56,5);
	dictionary = cv::aruco::Dictionary::create (102,5,0); 	
        //dictionary = cv::aruco::generateCustomDictionary(102,5);

	//read back and confirm
	cv::FileStorage fsr("dic_save.yml", cv::FileStorage::READ);
	int mSize, mCBits;
	cv::Mat bits;
	fsr["MarkerSize"] >> mSize;
	fsr["MaxCorrectionBits"] >> mCBits;
	fsr["ByteList"] >> bits;
	fsr.release();
	//Ptr<aruco::Dictionary> dictionary;
	cv::aruco::Dictionary dic = cv::aruco::Dictionary(bits, mSize, mCBits);
	*dictionary = dic;
	

	float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
		                       markerSeparation);

	// create board object
	//gridboard = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);

	//board = gridboard.staticCast<aruco::Board>();
 
}//END ARUCO INIT FUNCTION


//KEEP MEMORY OF MEASUEMENTS
int toalFrameCountwithMarkers;
int idsCountPrevFrame;


//////////////// ARUCO MARKER ID /////////////

vector<cv::Vec<double, 3>> centerPoints;
vector<int> centerPointsIDs;
std::vector<cv::Mat> centerRotations;


Vec3f toEuler; //toEuler = rotationMatrixToEulerAngles(rotResultC)*(180.0f/PI);
int counted = 0;
Vec3d rvecsBaseC, tvecsBaseC;
Mat rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
//separate inliers and outliers
vector< int > inlierIDs;
vector< int > outlierIDs;
std::string individual_identified_tranforms_OUTPUT;
//int errorID = -1;
Mat transResultC = (Mat_<double>(1,3) << 0,0,0);
float distSideToSolidCenter;
float actualRectSideSize;
//summaries
int identifiedTriangleCount=0;
int identifiedRectangleCount=0;

vector< int > ids;
float Center_to_Cam_dist = -1;

vector< Point2f > imagePoints;	
void resetVTempVariables3(){
	//cout << "centerPoints " << centerPoints.size() << "centerPointsIDs " << centerPointsIDs.size() << "centerRotations " << centerRotations.size() << "counted " << counted << endl;		

	centerPoints.clear();
	centerPointsIDs.clear();
	centerRotations.clear();

	counted = 0;

	inlierIDs.clear();
	outlierIDs.clear();
	identifiedTriangleCount=0;
	identifiedRectangleCount=0;	
	Center_to_Cam_dist = -1;
	imagePoints.clear();


	outlierComboIDs.clear();
	comboIteration=0;
	foundIDs.clear();
	combination.clear();
	combinations.clear();
	errorID = -1;
	comboDepthLevel=0;

	rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
	transResultC = (Mat_<double>(1,3) << 0,0,0);
}





// matrix to rotate from (0,0,1) to specified direction
cv::Mat directionMatrix(cv::Vec<double, 3> direction) {


    cv::Vec<double, 3>  normal = cv::Vec<double, 3> (0.0,0.0,1.0);
    cv::Vec<double, 3>  VA = cv::normalize(  normal.cross(direction));//    cross(normal, direction));
    Point3f V;
    V.x = VA(0);
    V.y = VA(1);
    V.z = VA(2);
    float phi = acos(normal.dot(direction));//   dot(normal,direction));

    float rcos = cos(phi);
    float rsin = sin(phi);

    Mat rotMatrix = (Mat_<double>(3,3) << 
						rcos + V.x * V.x * (1.0 - rcos), 	 -V.z * rsin + V.x * V.y * (1.0 - rcos), V.y * rsin + V.x * V.z * (1.0 - rcos), 
						V.z * rsin + V.y * V.x * (1.0 - rcos),   rcos + V.y * V.y * (1.0 - rcos),	 -V.x * rsin + V.y * V.z * (1.0 - rcos), 
						 -V.y * rsin + V.z * V.x * (1.0 - rcos), -V.x * rsin + V.z * V.y * (1.0 - rcos), rcos + V.z * V.z * (1.0 - rcos)
						);
    return rotMatrix;
}


int checkAxisFlip(Mat image, vector<Point2f> imagePointsAA, Mat imageCopy, int markerID){
	int correctFlip = 1;
	if( image.rows > 0 && image.cols > 0 && imagePointsAA.size() > 0){				
		//TRACK FEATURES and compare to PROJECTED					
		int topPointX = imagePointsAA[0].x - 250*1.2;
		int topPointY = imagePointsAA[0].y - 250*1.2;
		if(topPointX < 0){
			topPointX = 0;
		}
		if(topPointX > camWidth){
			topPointX = camWidth;
		}
		if(topPointY < 0){
			topPointY = 0;
		}
		if(topPointY > camHeight){
			topPointY = camHeight;
		}
		int roiWidth = 360*1.2;
		int roiHeight = 360*1.2;
		if(roiWidth <= 0){
			roiWidth = 10;
		}
		if(roiHeight <= 0){
			roiHeight = 10;
		}
											
		cv::Mat imageROI(image, cv::Rect(topPointX, topPointY, roiWidth, roiHeight));
		Mat cur_grey;
		cvtColor(imageROI, cur_grey, COLOR_BGR2GRAY);
		//https://stackoverflow.com/questions/58831690/how-to-measure-the-saturation-of-an-image
		Mat saturationImage;
		cvtColor(imageROI, saturationImage, COLOR_BGR2HSV);	
		int cornersMax = 160;
		float minFeatureDistance = 1;
		float cornerQuality = 0.05;
		vector <Point2f> prev_corner, cur_corner;
		//imshow("imageROI",cur_grey);						
		//cv::goodFeaturesToTrack(prev_grey.getUMat(ACCESS_RW), upoints, cornersMax, cornerQuality, minFeatureDistance, noArray(),4 , false, 0.04); 
		cv::goodFeaturesToTrack(cur_grey.getUMat(ACCESS_RW), prev_corner, cornersMax, cornerQuality, minFeatureDistance, noArray(),4 , false, 0.04); 
		Point2f centroid = Point2f(0,0);
		int countPixels = 0;
		for(size_t i=0; i < prev_corner.size(); i++) {
			Vec3b color = saturationImage.at<Vec3b>(prev_corner[i]);
			float saturation = color(1);													
			if(saturation < 65){
				circle(imageCopy, prev_corner[i] + Point2f((float)topPointX, (float)topPointY), 2.5,  CV_RGB(242/1, 2/112, 220/121), -1);		
				centroid += prev_corner[i] + Point2f((float)topPointX, (float)topPointY);
				countPixels++;		
			}
		}
		centroid.x = centroid.x / countPixels;
		centroid.y = centroid.y / countPixels;
		circle(imageCopy, centroid, 9,  CV_RGB(252/1, 222/2, 220/121), -1);

		//if axis looks towards centerpoint, flip it
		Point2f towardsZ = Point2f(imagePointsAA[3].x - imagePointsAA[0].x,  imagePointsAA[3].y - imagePointsAA[0].y);
		Point2f towardsCenteroid = Point2f(centroid.x - imagePointsAA[0].x,  centroid.y - imagePointsAA[0].y);
		float phi = towardsZ.dot(towardsCenteroid);					
		if(     phi > 0   && ids.size() < 17){
			correctFlip = -1;
			cout << "-------------------------------" << endl;
			cout << "FOUND FLIPPED MARKER " << markerID << endl;//ids[i] << endl;
			cout << "-------------------------------" << endl;
		}		
										
	}
	return correctFlip;
}
 

void arucoProcessmarkerID(Mat& image,Mat& srcImg,Mat& imageCopy, float ExposureAbsolute, int outWindowID, int solidID, vector< int > ids, vector<Vec3d> rvecs, vector<Vec3d> tvecs, float tileSize, int StartID) {
	

	actualRectSideSize = (8.0f/7.0f) * tileSize;//assuming one extra tile around the 7x7 (5x5 tile) marker (half left, half right)

	if(shapeID==0){ //CORRECTION of 8/7 of 6.9, since actual 21 edron boundary is at 7.5 than 7.885
		actualRectSideSize = 0.075f;
	}

	distSideToSolidCenter = ((sqrt(2.0f)+1.0f)/2.0f) * actualRectSideSize;

	if(shapeID==1 || shapeID==3){ //if Cube shape
		distSideToSolidCenter = actualRectSideSize/2.0f;
	}	


	//vector< int > ids;
	//vector<Vec3d> rvecs, tvecs;

	//ANALYSE FOUND MARKERS
	if (ids.size() > 0) {

		idsCountPrevFrame = ids.size();
		toalFrameCountwithMarkers++; //increase counter of frames in which markers were found

		//if(preview == 1){
		//	aruco::drawDetectedMarkers(imageCopy, corners, ids);
		//}

		for (int i = 0; i < ids.size(); i++) {

			cv::Mat R;
			cv::Rodrigues(rvecs[i],R);			

			//TEXT ON IMAGE
			//cv::putText(imageCopy, //target image
			//    "TURNBACKDIFF = " + to_string(TURNBACKDIFF(0)) + "," +  to_string(TURNBACKDIFF(1))+ "," +  to_string(TURNBACKDIFF(2)) , //text
			//    cv::Point(10, imageCopy.rows / 2), //top-left position
			//    cv::FONT_HERSHEY_DUPLEX,
			//    1.0,
			//    CV_RGB(118, 185, 0), //font color
			//    2);			

			if(preview == 1 ){
				aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 2.5f);				
			}

			vector< Point2f > imagePointsAA;	
			vector< Point3f > axisPointsA;			
			axisPointsA.push_back(Point3f(0, 0, 0));	
			axisPointsA.push_back(Point3f(markerLength * 15.5f, 0, 0));
			axisPointsA.push_back(Point3f(0, markerLength * 15.f, 0));
			axisPointsA.push_back(Point3f(0, 0, markerLength * 15.5f));
			Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //			
			projectPoints(axisPointsA, rvecs[i], tvecs[i], camMatrix, distCoeffs, imagePointsAA, imagePointsVelocities);
					
			//float correctFlip = 1;	
			
			int correctFlip = checkAxisFlip(image, imagePointsAA, imageCopy, ids[i]);


			if(preview == 1 && imagePointsAA.size() > 0){										
						if(imagePointsAA[0].x>0 && imagePointsAA[0].x < camWidth && imagePointsAA[0].y>0 && imagePointsAA[0].y < camHeight &&
						imagePointsAA[1].x>0 && imagePointsAA[1].x < camWidth && imagePointsAA[1].y>0 && imagePointsAA[1].y < camHeight &&
						imagePointsAA[2].x>0 && imagePointsAA[2].x < camWidth && imagePointsAA[2].y>0 && imagePointsAA[2].y < camHeight &&
						imagePointsAA[3].x>0 && imagePointsAA[3].x < camWidth && imagePointsAA[3].y>0 && imagePointsAA[3].y < camHeight
						){
							line(imageCopy, imagePointsAA[0], imagePointsAA[1], Scalar(0, 0, 255/(2)), 3);
							line(imageCopy, imagePointsAA[0], imagePointsAA[2], Scalar(0, 255/(2), 0), 3);
							line(imageCopy, imagePointsAA[0], imagePointsAA[3], Scalar(255/(2), 0, 0), 3);	
						}									
			}				



			//cam pose
			//cv::Mat R;
			//cv::Rodrigues(rvecs[i],R);
			cv::Mat camPose = -R.t() * (cv::Mat)tvecs[i];
			
			
			
			//assign IDs starting from left or central			
			int centralID = StartID+0;
			int leftID1 = StartID+1;
			int leftID2 = StartID+2;
			int leftID3 = StartID+3;
			int leftID4 = StartID+4;
			int leftID5 = StartID+5;
			int leftID6 = StartID+6;
			int leftID7 = StartID+7;			
			int centralUpperID = StartID+8; 
			int leftUpperID1 = StartID+10;
			int leftUpperID2 = StartID+12;
			int leftUpperID3 = StartID+14;
			int topID = StartID+16;
			int leftUpperTriangleID1 = StartID+9;
			int leftUpperTriangleID2 = StartID+11;
			int leftUpperTriangleID3 = StartID+13;
			int leftUpperTriangleID4 = StartID+15;			

			int exposureDEBUG = ExposureAbsolute; 

			Vec3d rvecsBase, tvecsBase;
						
			if((ids[i] != leftUpperTriangleID1 || shapeID==3) && ids[i] != leftUpperTriangleID2 && ids[i] != leftUpperTriangleID3  && (ids[i] != leftUpperTriangleID4 || shapeID==3)){

				//rotate final axis based on rotation vs centralID marker axis				
				Mat rotMatrixFinal = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); 
				float th = 0;
				if(ids[i] == leftID1){ 
					//around Y, 45 degrees
					if(shapeID >= 2){
						th = -0.0f*(PI/4.0f);
					}else{
						th = -1.0f*(PI/4.0f);
					}					
				}
				if(ids[i] == leftID2 || ids[i] == leftUpperID1){ 
					//around X, 90 degrees
					th = -2.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID3){ 
					//around Y, 45 degrees
					if(shapeID >= 2){
						th = -2.0f*(PI/4.0f);
					}else{
						th = -3.0f*(PI/4.0f);		
					}			
				}
				if(ids[i] == leftID4 || ids[i] == leftUpperID2){ 
					//around Y, 45 degrees
					th = -4.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID5){ 
					//around Y, 45 degrees
					if(shapeID >= 2){
						th = -4.0f*(PI/4.0f);
					}else{
						th = -5.0f*(PI/4.0f);	
					}				 
				}
				if(ids[i] == leftID6 || ids[i] == leftUpperID3){ 
					//around Y, 45 degrees
					th = -6.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID7){ 
					//around Y, 45 degrees
					if(shapeID >= 2){
						th = -6.0f*(PI/4.0f);
					}else{
						th = -7.0f*(PI/4.0f);	
					}				
				}
				
				rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1); 

				//ROTATE UPPER PARTS around X axis (rotated in Y axis above already)
				if(ids[i] == leftUpperID1 || ids[i] == leftUpperID2 || ids[i] == leftUpperID3){ 
					if(shapeID == 3){
						//th = 0.0f*(PI/4.0f); //dont rotate further for 3rd cube
						th = 2.0f*(PI/4.0f);
						rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1)*rotMatrixFinal; 
					}else{
						th = 1.0f*(PI/4.0f);
						rotMatrixFinal = 
						(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1)*rotMatrixFinal;
					}
				}
				if(ids[i] == centralUpperID){ 

					if(shapeID >= 2){
						th = 2.0f*(PI/4.0f);//make central upper the top part of 2ond cube
					}else{
						th = 1.0f*(PI/4.0f);
					}
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}						
				if(ids[i] == topID){ 
					th = 2.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}


				//3rd cube
				if(ids[i] == leftUpperTriangleID4 && shapeID == 3){ 
					th = 2.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}
				if(ids[i] == leftUpperTriangleID1 && shapeID == 3){ 
					th = 2.0f*(PI/4.0f);
					rotMatrixFinal = 
					 (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1);
				}

				//plot axis of solid center
				//Vec3d rvecsBase, tvecsBase;
								
				//cast translation vector to matrix				
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,-distSideToSolidCenter * correctFlip, 0,0,0,1); 

				//rot
				Mat rotMatrix = (Mat_<double>(4,4) << 
						R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),0, 
						R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),0, 
						R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),0, 
						0,0,0,1);

				//homogenous transf						
				Mat transHomog = (Mat_<double>(4,1) << tvecs[i](0),tvecs[i](1),tvecs[i](2),1);	
				Mat translateCamMarker = (Mat_<double>(4,4) << 
				1,0,0,tvecs[i](0), 
				0,1,0,tvecs[i](1), 
				0,0,1,tvecs[i](2), 
				0,0,0,1);					
				
				//create rotation matrix, do translation, rotation, translation, then split matrix back to rvec,tvec				
				Mat centerPose = translateCamMarker * rotMatrix * translateMatrix * rotMatrixFinal; // R * (cv::Mat)tvecs[i];
				
				//Take whole
				Mat rotResult = (Mat_<double>(3,3) << 
						centerPose.at<double>(0,0),centerPose.at<double>(0,1),centerPose.at<double>(0,2), 
						centerPose.at<double>(1,0),centerPose.at<double>(1,1),centerPose.at<double>(1,2), 
						centerPose.at<double>(2,0),centerPose.at<double>(2,1),centerPose.at<double>(2,2));
				Mat transResult = (Mat_<double>(1,3) << centerPose.at<double>(0,3),centerPose.at<double>(1,3),centerPose.at<double>(2,3));

				
				//get final rotation and turn it to vector				
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;				

				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				centerPoints.push_back(transResultVec);	
				centerRotations.push_back(rotResult);	
				centerPointsIDs.push_back(ids[i]);		
				

			}// END IDs check to not be triangles (cover only rectangles)
			else if(shapeID != 1 && shapeID != 3){ //if NOT Cube handle triangles

				//triangle summaries
				identifiedTriangleCount++;

				//handle triangles
				
				//Mesokathetos trigonou SideX
				float SideX = actualRectSideSize*(sqrt(3.0f)/2.0f);
		
				//Inner marker side half size SideC (SideB/2)
				float SideC = (0.5f*sin(PI/3.0f)* actualRectSideSize)/(1.0f+sin(PI/3.0f));

				//Perperndicular from Center to mesokathetos end point to triangle upper corner point
				float SideDA = 0.2064f * actualRectSideSize * sqrt(5.0f+2.0f*sqrt(2.0f));
				
				//Perperndicular line length from Center to mesokathetos 
				float SideTR = 0.45542f * actualRectSideSize * sqrt(5.0f+2.0f*sqrt(2.0f));

				//Distance from marker center to perperndicular from Center to mesokathetos end point
				float SideW = SideX -SideDA -SideC;


				Mat rotMatrixFinal = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); 
				float th = 0;
				if(ids[i] == leftUpperTriangleID1){ 
					//around Y, 45 degrees
					th = -1.0f*(PI/4.0f);					
				}				
				if(ids[i] == leftUpperTriangleID2){ 
					//around Y, 45 degrees
					th = -3.0f*(PI/4.0f);					
				}				
				if(ids[i] == leftUpperTriangleID3){ 
					//around Y, 45 degrees
					th = -5.0f*(PI/4.0f);				
				}				
				if(ids[i] == leftUpperTriangleID4){ 
					//around Y, 45 degrees
					th = -7.0f*(PI/4.0f);					
				}
				rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1); 

				//ROTATE UPPER PARTS around X axis (rotated in Y axis above already)
				if(ids[i] == leftUpperTriangleID1 || ids[i] == leftUpperTriangleID2 || ids[i] == leftUpperTriangleID3 
				|| ids[i] == leftUpperTriangleID4)
				{ 					
					th = (35.26f/180.0f)*PI; //(54.73f/180.0f)*PI; //1.0f*(PI/4.0f); // (45/180)*PI // (35.26f/180.0f)*PI;
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1)*rotMatrixFinal;
				}				

				//FIND large to small fiducial marker ration
				float ratio = (tileSize * 100) / 3.2f;  //6.9f / 3.4f; //3.3f; // CHANGE HERE (last was 3.4f)

				//plot axis of solid center
				//Vec3d rvecsBase, tvecsBase;				
				
				//CHECK if dist is same
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,SideW, 0,0,1,-SideTR * correctFlip, 0,0,0,1);

				//rot
				Mat rotMatrix = (Mat_<double>(4,4) << 
						R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),0, 
						R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),0, 
						R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),0, 
						0,0,0,1);

				//homogenous transf				
				Mat translateCamMarker = (Mat_<double>(4,4) << 
						1,0,0,tvecs[i](0)/ratio, 0,1,0,tvecs[i](1)/ratio, 0,0,1,tvecs[i](2)/ratio, 0,0,0,1);					
				
				//create rotation matrix, do translation, rotation, translation, then split matrix back to rvec,tvec				
				Mat centerPose = translateCamMarker * rotMatrix * translateMatrix * rotMatrixFinal; // R * (cv::Mat)tvecs[i];
				
				//Take whole
				Mat rotResult = (Mat_<double>(3,3) << 
						centerPose.at<double>(0,0),centerPose.at<double>(0,1),centerPose.at<double>(0,2), 
						centerPose.at<double>(1,0),centerPose.at<double>(1,1),centerPose.at<double>(1,2), 
						centerPose.at<double>(2,0),centerPose.at<double>(2,1),centerPose.at<double>(2,2));
				Mat transResult = (Mat_<double>(1,3) << centerPose.at<double>(0,3),centerPose.at<double>(1,3),centerPose.at<double>(2,3));

				
				//get final rotation and turn it to vector				
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;

				//plot base axis
				if(preview == 1 ){
					//aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecsBase, tvecsBase, markerLength * 2.2f);
				}

				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				centerPoints.push_back(transResultVec);	
				centerRotations.push_back(rotResult);	
				centerPointsIDs.push_back(ids[i]);		
			}	



			//Mat transResultC = (Mat_<double>(1,3) << 0,0,0);
			vector< Point2f > imagePointsA;	
			vector< Point3f > axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
			axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
			axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));			
			//cv::Rodrigues(rotResultC,rvecsBaseC);
			//Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //	
			//Mat transResultC = (Mat_<double>(1,3) << 0,0,0);	
			//PROJECT SOLID CENTER AXIS
			//projectPoints(axisPoints, rvecsBaseC, tvecsBaseC, CamParam.CameraMatrix, CamParam.Distorsion, imagePoints, imagePointsVelocities);	
			//cout << tvecsBase << endl;
			//projectPoints(axisPoints, rvecsBase, tvecsBase, CamParam.CameraMatrix, CamParam.Distorsion, imagePointsA, imagePointsVelocities);	
			projectPoints(axisPoints, rvecsBase, tvecsBase, camMatrix, distCoeffs, imagePointsA, imagePointsVelocities);



			//projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix, distCoeffs, imagePointsA, imagePointsVelocities);


			if(preview == 1 && imagePointsA.size() > 1){// && correctFlip == 1 &&  1==1){		
							//Mat imagePLOT;
							//iimageCopymageCopy.copyTo(imagePLOT);	
							//// draw axis lines
							if(imagePointsA[0].x>0 && imagePointsA[0].x < camWidth && imagePointsA[0].y>0 && imagePointsA[0].y < camHeight &&
							imagePointsA[1].x>0 && imagePointsA[1].x < camWidth && imagePointsA[1].y>0 && imagePointsA[1].y < camHeight &&
							imagePointsA[2].x>0 && imagePointsA[2].x < camWidth && imagePointsA[2].y>0 && imagePointsA[2].y < camHeight &&
							imagePointsA[3].x>0 && imagePointsA[3].x < camWidth && imagePointsA[3].y>0 && imagePointsA[3].y < camHeight
							){
								line(imageCopy, imagePointsA[0], imagePointsA[1], Scalar(0, 0, 255/(3-correctFlip)), 3-correctFlip);
								line(imageCopy, imagePointsA[0], imagePointsA[2], Scalar(0, 255/(3-correctFlip), 0), 3-correctFlip);
								line(imageCopy, imagePointsA[0], imagePointsA[3], Scalar(255/(3-correctFlip), 0, 0), 3-correctFlip);	
							}	
						imshow("Alll axis",imageCopy);
			}		

	

		}//END FOR LOOP over IDs found

	}//END check if ids found > 0

}//END ARUCO MARKER IDENTIFICATION FUNCTION




////////////////// DO OUTLIERS
//vector< Point2f > imagePoints;	
void doOuliers(){
	vector< Point3f > axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
	axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
	axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));		
	imagePoints.clear();
  	if (centerPointsIDs.size() > 0) {		
			
		//CONSENSUS

		//separate inliers and outliers		
		inlierIDs.clear();
		outlierIDs.clear();

		int consensusMethod = 2;

		cv::Vec<double, 3> centerMean;
		//vector< Point2f > imagePoints;
								

		//METHOD 1
		consensusMethod = 1;		

		//METHOD 1 PLOT
		if(consensusMethod == 1){

			outlierComboIDs.clear(); //empty the erroneous
			comboIteration = 0; //reset iterations count					

			errorID = -1;
			comboDepthLevel = 0;

			//discoverOutliers();
			vector<cv::Vec<double, 3>> centerPointsTMP = centerPoints;
			vector<int> centerPointsIDsTMP = centerPointsIDs;
			std::vector<cv::Mat> centerRotationsTMP = centerRotations;
			
			discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP ); // FIND OUTLIERS
			
			centerPoints    = centerPointsTMP;
			centerPointsIDs = centerPointsIDsTMP;
			centerRotations = centerRotationsTMP;
			zero31_matrix(centerMean);		
			
			inlierIDs.clear();
			outlierIDs.clear();

			for (int i = 0; i < centerPointsIDs.size(); i++) {				
				float Center_to_Prev_Center_dist = matrix31_distance(centerMean,centerPoints[i]);
				
				//threshold, if center is above this will not use point in consensus
				if(Center_to_Prev_Center_dist < 0.06){ //if(Center_to_Prev_Center_dist < 0.05 && pass0){  //CHANGE HERE
					transResultC.at<double>(0,0) = centerPoints[i](0) + transResultC.at<double>(0,0);
					transResultC.at<double>(0,1) = centerPoints[i](1) + transResultC.at<double>(0,1);
					transResultC.at<double>(0,2) = centerPoints[i](2) + transResultC.at<double>(0,2);
					//cout << "centerPoints[" << i << "] = " << centerPoints[i] << endl;					
					counted=counted+1;

					inlierIDs.push_back(centerPointsIDs[i]);
				}else{
					//flip point and add
					//translate along its z axis double the distance
					if(i == centerPointsIDs.size()-1 && inlierIDs.size() == 0){
						transResultC.at<double>(0,0) = centerPoints[i](0) + transResultC.at<double>(0,0);
						transResultC.at<double>(0,1) = centerPoints[i](1) + transResultC.at<double>(0,1);
						transResultC.at<double>(0,2) = centerPoints[i](2) + transResultC.at<double>(0,2);
						inlierIDs.push_back(centerPointsIDs[i]);
						counted=counted+1;				//IF last element and no inliers, choose as center
					}else{
						outlierIDs.push_back(centerPointsIDs[i]);
					}
				}
			}

			if(counted == 0){
				transResultC.at<double>(0,0) = 0;
				transResultC.at<double>(0,1) = 0;
				transResultC.at<double>(0,2) = 0;
				tvecsBaseC = transResultC;
				//return 0;
			}else{
				transResultC.at<double>(0,0) = (transResultC.at<double>(0,0)) / counted;
				transResultC.at<double>(0,1) = (transResultC.at<double>(0,1)) / counted;
				transResultC.at<double>(0,2) = (transResultC.at<double>(0,2)) / counted;
				tvecsBaseC = transResultC;//redo passing to final axis vector
			}

			//ROTATIONS			
			zero33_matrix(rotResultC);

			//cout << "AFTER ZEROING = " << rotResultC << endl;
			Mat rotResultCTMP = (Mat_<double>(3,3) << 0,0,0, 0,0,0, 0,0,0);
			for (int i = 0; i < centerPointsIDs.size(); i++) {	
				add_to_33_matrix (rotResultCTMP, centerRotations[i]);				
				//cout << "Center ID = " << centerPointsIDs[i] << " Center ID Coords = " << centerPoints[i] << endl;
			}
			scale33_matrix(rotResultCTMP, centerPointsIDs.size());
			
			//cout << "Center MEAN = " << centerMean << endl;
			//If distance from mean center above threshold, dont count in centre calculation (reset previous loop based created consensus)
			int countedRots = 0;
			for (int i = 0; i < centerPointsIDs.size(); i++) {
				float Center_to_Prev_Center_dist = matrix33_distance(rotResultCTMP, centerRotations[i]);				
				//if(Center_to_Prev_Center_dist < 1.05){ //threshold, if center is above this will not use point in consensus
					add_to_33_matrix (rotResultC, centerRotations[i]);					
					countedRots=countedRots+1;
				//}
			}
			scale33_matrix(rotResultC, countedRots);			
			cv::Rodrigues(rotResultC,rvecsBaseC);//redo passing to final rotations vector	
			Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //

			//PROJECT SOLID CENTER AXIS
			projectPoints(axisPoints, rvecsBaseC, tvecsBaseC, camMatrix, distCoeffs, imagePoints, imagePointsVelocities);			

		}//END METHOD 1 PLOTS 

				
		
		//KEEP PREVIOUS CENTERS and their PROJECTIONS
		

		
	}
}// END DO OUTLIERS




//////////////// END ARUCO MARKER ID /////////

bool arucoProcess(Mat srcImg, float ExposureAbsolute, int outWindowID, string frameID, int image360part) {
	
	Mat grayImage;
 	cvtColor(srcImg, grayImage, CV_RGB2GRAY);
	//cout << "srcImg.colsC = " << srcImg.cols << " srcImg.rowsC = " << srcImg.rows << " grayImage non zero = " << countNonZero(grayImage) << endl;

   	//OCAM - ARUCO
	Mat image, imageCopy;	

	//ENABLE BOUNDING CALC WIHTOUT autoFocus on
	bool forceBoundingCalc = false;
	if(alpha > 0){
		forceBoundingCalc = true;
	}

	image = srcImg;


	//TIMER
 	double tick = (double)getTickCount();

	//PLOT ALL AXIS
	Mat allCentersImage;
	image.copyTo(allCentersImage);



//////////////////////////////
	//void arucoProcessmarkerID(Mat& image,Mat& srcImg,Mat& imageCopy,
	Mat grayImageA;
 	cvtColor(srcImg, grayImageA, CV_RGB2GRAY);
	//cout << "srcImg.colsB = " << srcImg.cols << " srcImg.rowsB = " << srcImg.rows << " grayImage non zero = " << countNonZero(grayImage) << endl;	

	if(!srcImg.empty() && countNonZero(grayImageA) > 0){
	
	}else{
		return 1;
	}

	//ARUCO FOUND VECTORS (temp variables)
       
        vector< vector< Point2f > > corners, rejected;
       // Vec3d rvec, tvec;

	//Marker pose matrices
	vector<Vec3d> rvecs, tvecs;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

	// refind strategy to detect more markers
        if(refindStrategy){
            //aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix, distCoeffs);
	    //cout << "REFINE ON = " << refindStrategy << endl;
	}

        // estimate board pose  
	//MARKER SIZE DEFINITION
	float tileSize = 0.025f; //small test cube
	//tileSize = 0.08; //8cm test
	tileSize = 0.0635f; //bigger solid A3 page test
	tileSize = 0.0690f; //final solid size			//CHANGE HERE
	//tileSize = 0.048f; //smaller 21edron
	
	if(shapeID==1 || shapeID==3){ //if Cube shape
		tileSize = 0.109f;//0.126f;
	}
	//ESTIMATE MARKERS POSE
	if(ids.size() > 0){
		aruco::estimatePoseSingleMarkers(corners, tileSize, camMatrix, distCoeffs, rvecs, tvecs); //BASIC dimension in meters here!
	}	

	// draw results
	if(preview == 1){
		image.copyTo(imageCopy);
		if(ids.size() > 0) {
		    aruco::drawDetectedMarkers(imageCopy, corners, ids);
		}
		//showRejected = true;
		if(showRejected && rejected.size() > 0){
	    		aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
		}
	}
	if (ids.size() > 0) {		
			if(preview == 1){
				aruco::drawDetectedMarkers(imageCopy, corners, ids);
			}
	}
//////////////////////////////

/// SPLIT
int solidsCount = 2;
vector<int> startIDsPerSolid;
startIDsPerSolid.push_back(0);
startIDsPerSolid.push_back(17);

vector<vector<int>> idsV;
vector<vector<Vec3d>> rvecsV;
vector<vector<Vec3d>> tvecsV;
for (int j = 0; j < solidsCount; j++) {	
	vector<int> tmpIDs;	
	vector<Vec3d> tmpRvecss;
	vector<Vec3d> tmpTvecss;	
	for (int i = 0; i < ids.size(); i++) {						
		if(ids[i] >= startIDsPerSolid[j] && ids[i] <= startIDsPerSolid[j]+16){
			tmpIDs.push_back(ids[i]);
			tmpRvecss.push_back(rvecs[i]);
			tmpTvecss.push_back(tvecs[i]);
		}
	}
	idsV.push_back(tmpIDs);
	rvecsV.push_back(tmpRvecss);
	tvecsV.push_back(tmpTvecss);
}
/// END SPLIT

for (int j = 0; j < solidsCount; j++) {
	resetVTempVariables3();
	//arucoProcessmarkerID(image, srcImg,imageCopy, ExposureAbsolute, outWindowID, 1, idsV[0], rvecsV[0], tvecsV[0], tileSize, 0); //find solid 0 points
	arucoProcessmarkerID(image, srcImg,imageCopy, ExposureAbsolute, outWindowID, 1, idsV[j], rvecsV[j], tvecsV[j], tileSize, startIDsPerSolid[j]); //find solid 0 points
	//process solid 0 points and plot it
	doOuliers();	
	//plot base axis
	if(preview == 1 && imagePoints.size() > 1){			
			// draw axis lines
			if(imagePoints[0].x>0 && imagePoints[0].x < camWidth && imagePoints[0].y>0 && imagePoints[0].y < camHeight &&
			imagePoints[1].x>0 && imagePoints[1].x < camWidth && imagePoints[1].y>0 && imagePoints[1].y < camHeight &&
			imagePoints[2].x>0 && imagePoints[2].x < camWidth && imagePoints[2].y>0 && imagePoints[2].y < camHeight &&
			imagePoints[3].x>0 && imagePoints[3].x < camWidth && imagePoints[3].y>0 && imagePoints[3].y < camHeight
			){
				line(allCentersImage, imagePoints[0], imagePoints[1], Scalar(0, 0, 255/(j+1)), 3);
				line(allCentersImage, imagePoints[0], imagePoints[2], Scalar(0, 255/(j+1), 0), 3);
				line(allCentersImage, imagePoints[0], imagePoints[3], Scalar(255/(j+1), 0, 0), 3);	
			}	
			//imshow("out"+to_string(outWindowID), imageCopy);		
	}
	//Center_to_Cam_dist = sqrt(tvecsBaseC(0)*tvecsBaseC(0) + tvecsBaseC(1)*tvecsBaseC(1) + tvecsBaseC(2)*tvecsBaseC(2));	
	if(preview == 1){
			//toEuler = rotationMatrixToEulerAngles(rotResultC)*(180.0f/PI);
			//cout << "Center to Camera rotation " << toEuler << endl;
			//cout << "Center to Camera distance " << Center_to_Cam_dist << endl;			
			//cout << "identifiedRectangleCount " << identifiedRectangleCount << endl;
			//cout << "identifiedTriangleCount " << identifiedTriangleCount << endl;
	}
}


	if(preview == 1){
			//imshow("out"+to_string(outWindowID), imageCopy);
			imshow("outALL"+to_string(outWindowID), allCentersImage);
			//Mat imageUndistorted;
			//undistort(image, imageUndistorted,camMatrix, distCoeffs);
			//imshow("outUndistorted", imageUndistorted);
	}
	return 1;


	
}

//// END ARUCO


//CHECK IMAGE EQUALITY
bool equal(const Mat& a, const Mat& b)
{
    if ( (a.rows != b.rows) || (a.cols != b.cols) )
        return false;
    Scalar s = sum( a - b );
    return (s[0]==0) && (s[1]==0) && (s[2]==0);
}


//TIMER2
struct timeval beginT2, endT2;
double REAL_seconds_since_start2=0;
struct timeval beginT3, endT3;
double REAL_seconds_since_start3=0;
struct timeval beginT4, endT4;
double REAL_seconds_since_start4=0;

//////// ROS

CommandLineParser parser();

int frameIDS = 0;
void imageCallback(int image360part, cv::Mat srcImg) //(const sensor_msgs::ImageConstPtr& msg, int image360part)
{	
	cv::Mat prevSrcImg;
	int same_images_count=0;
  try
  {	
	//TIMER 4
	gettimeofday(&beginT4, NULL);

	//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	cv::waitKey(5);
	//std::this_thread::sleep_for(std::chrono::milliseconds(5));
	usleep(5);

	//ARUCO START	
	int ExposureAbsolute = 72;


	Mat grayImage;
	cvtColor(srcImg, grayImage, CV_RGB2GRAY);
	string frameID = to_string(frameIDS);//  cv_bridge::toCvShare(msg, "bgr8")->header.frame_id;
	frameIDS++;

	if(!srcImg.empty() && countNonZero(grayImage) > 0){

		Mat srcImgMID = srcImg;
	
		//TIMER 4
		gettimeofday(&endT4, NULL);	
		double elapsed2 = (endT4.tv_sec - beginT4.tv_sec) + ((endT4.tv_usec - beginT4.tv_usec)/1000000.0);	
		//beginT4 = endT4;
		if(totalIterations % 30 == 0) {	
			//cout << "(Grab Camera Frame Time = " << 1000 * elapsed2 << " ms)" << endl;
		}

		//TIMER 3
		gettimeofday(&beginT3, NULL);

		//imshow("out", srcImgMID);
		arucoProcess(srcImgMID,ExposureAbsolute, 1,frameID, image360part);	

		//TIMER 3
		gettimeofday(&endT3, NULL);						
		double elapsed1 = (endT3.tv_sec - beginT3.tv_sec) + ((endT3.tv_usec - beginT3.tv_usec)/1000000.0);	
		//beginT3 = endT3;
		if(totalIterations % 30 == 0) {
			//cout << "(Aruco Calculations Time = " << 1000 * elapsed1 << " ms)" << endl;	
		}

	     //SIMILARITY CHECK (NOTE TO DISABLE beginT=endT; inside Aruco if is used here, otherwise use the one on the ROS node that sends the image)
	     bool FPS_measure=0;
	     if(FPS_measure == 1) {
		//CHECK IF IMAGE SAME AS PREVIOUS //CHECK IMAGE EQUALITY	
		if(!prevSrcImg.empty() && !srcImg.empty() && equal(prevSrcImg,srcImg)){
			same_images_count++;
			//cout << same_images_count << " SAME IMAGES FOUND" << endl;
		}	
		//imshow("out", srcImg);
		srcImg.copyTo(prevSrcImg);

		//TIMER 
		totalIterationsTimer++;
		//if(totalIterationsTimer % 20 == 0) {
		if(totalIterationsTimer == 30 && 1==0) {
						
			//TIMER ACCURATE
			gettimeofday(&endT, NULL);						
			double elapsed = (endT.tv_sec - beginT.tv_sec) + 
		      	((endT.tv_usec - beginT.tv_usec)/1000000.0);
			beginT=endT;
			//cout << "Precision Time taken : " << elapsed << " seconds" << " with iterations:" << totalIterationsTimer << endl;  
			double fps2  = totalIterationsTimer / elapsed;
			//cout << "Estimated Precision frames per second : " << fps2 << endl;						

			//CHECK IF IMAGE SAME AS PREVIOUS //CHECK IMAGE EQUALITY		
			if(same_images_count >0){
				//cout << same_images_count << " SAME IMAGES FOUND" << endl;
				double fps3  = (totalIterationsTimer-same_images_count) / elapsed;
				//cout << "Estimated REAL frames per second : " << fps3 << endl;
			}		
			same_images_count=0;						
			totalIterationsTimer = 0;
		}
	     }//END FPS CALCS
	
	}//! empty check



  }
  catch (cv_bridge::Exception& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	ROS_ERROR("Could not convert from to 'bgr8'.");
  }
}


//sample counter
int POZYXsamplesCounted = 0;

//MAIN
int main(int argc, char **argv)
{     
	//BRIGHTNESS & CONTRAST
	//std::cout<<"* Enter the alpha value [1.0-3.0]: ";std::cin>>alpha;
 	//std::cout<<"* Enter the beta value [0-100]: "; std::cin>>beta;
	alpha = 1;
	beta=1;

  	ros::init(argc, argv, "image_listener");
  	ros::NodeHandle nh;

	//ARUCO START
	CommandLineParser parser(argc, argv, keys);
	markersX = parser.get<int>("w");
	markersY = parser.get<int>("h");
	markerLength = parser.get<float>("l");
	markerSeparation = parser.get<float>("s");
	dictionaryId = parser.get<int>("d");
	showRejected = parser.has("r");
	refindStrategy = parser.has("rs");
	camId = parser.get<int>("ci");
	colorMode = parser.get<int>("g");
	preview = parser.get<int>("p");
	autoFocus = parser.get<int>("af"); //auto focus
	shapeID = parser.get<int>("shape"); //0=21edro, 1=cube
	HorAngle = parser.get<float>("HorAngle"); //measured angles - distances, name outputs based on these 3 values
	VerAngle = parser.get<float>("VerAngle");
	RealDist = parser.get<float>("RealDist");

	//ENABLE POZYX
	usePOZYX = parser.get<int>("usePOZYX"); //enable POZYX
	//ENABLE LIDAR
	useLIDAR = parser.get<int>("useLIDAR"); //enable useLIDAR
	PCDcloud = parser.get<std::string>("PCDcloud");

	//std::ofstream outfile;	
	std::string fileName = "timeH"+to_string(static_cast<int>(HorAngle))
	+"V"+to_string(static_cast<int>(VerAngle))+"D"
	+to_string(static_cast<int>(RealDist*100))+".csv";
 
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
		throw std::ios_base::failure(std::strerror(errno));
	}
	//make sure write fails with exception if something is wrong
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);

	if(usePOZYX == 0){
		outfile << "time" << ";" << "Video Frame ID" << ";" << "Counted IDs" << ";" 
			<< "Counted Rectangles" << ";"  << "Counted Triangles"				
			<< ";" << "Solid Center to Camera distance" << ";" 
			<< "Translation Vector X (Solid to Camera)" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Euler X" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Rodrigues X" << ";" << "Y" << ";" << "Z" << ";"
			<< "Solid Camera RotMatrix (00)" << ";" << "(01)" << ";" << "(02)" << ";"
			<< "(10)" << ";" << "(11)" << ";" << "(12)" << ";"
			<< "(20)" << ";" << "(21)" << ";" << "(22)" << ";"			
			<< "distance MEAN ARUCO" << ";" << "variance Squared ARUCO" << ";"
			<< "Horizontal Angle" << ";" << "Vertical Angle" << ";" << "Horizontal Distance" << ";"
			<< "Inlier - Outlier IDs" << ";"
			<< "Individual marker IDs - transformations"
			<< std::endl;
	}

	if(preview == 1){
		cv::namedWindow("view");
		cv::startWindowThread();
	}
  	//image_transport::ImageTransport it(nh);

	//ARUCO PARAMS
	if(parser.has("c")) {
		bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
		if(!readOk) {
			cerr << "Invalid camera file" << endl;
			return 0;
		}
	}

	detectorParams = aruco::DetectorParameters::create();
	if(parser.has("dp")) {
		bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
		if(!readOk) {
			cerr << "Invalid detector parameters file" << endl;
			return 0;
		}
	}
	
	//detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; 
	//do marker corner refinement -- https://github.com/opencv/opencv_contrib/tree/master/modules/aruco/samples
	//detectorParams->doCornerRefinement = true; //do marker corner refinement 
	//detectorParams->cornerRefinementWinSize = 2; //do marker corner refinement 
 
	// String video;
	if(parser.has("v")) {
		video = parser.get<String>("v");
	}

	if(!parser.check()) {
		parser.printErrors();
		return 0;
	}

	arucoProcessInit();

	//TIMER
	gettimeofday(&beginT, NULL);
	gettimeofday(&beginT3, NULL);
	gettimeofday(&beginT4, NULL);

	//NEW1
	VideoCapture cap;
	
	cap.open(0);
	// Exit if video is not opened
	if(!cap.isOpened())
	{
		cout << "Could not read video file" << endl; 
		return 1; 
	} 
	
	float resMultiplier = 3;
	cap.set(3,640*resMultiplier);
	cap.set(4,360*resMultiplier);
	int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	assert(cap.isOpened());		
	Mat temp1; cap >> temp1;
	while(ros::ok() && true){
		cap >> temp1;
		imageCallback(1, temp1);
		cv::waitKey(2);
	}
	//END NEW1

	
}

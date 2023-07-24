//"CHANGE HERE" is for hardcoded parameters that should be changed

//cd ~/openCVmine/testROS/ocams/
// make all
//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib99.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=3.01 -shape=0 --rs
//cd ~/openCVmine/testROS/ocams/devel/lib/ocams
//./ocams -cc=0 _exposure:=60

//START LIDAR
// roslaunch velodyne_pointcloud VLP16_points.launch

//with LIDAR
// ./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib99.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0 --rs -usePOZYX=0 -useLIDAR=1 -PCDcloud="test.pcd"

//./lidar -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib99.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0 --rs -usePOZYX=0 -useLIDAR=1 -PCDcloud="test.pcd"
//./lidar -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib99.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0 --rs -usePOZYX=0 -useLIDAR=1 -PCDcloud="finalA1.pcd"

//ADD LiDAR - https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/


//rostopic list
//roslaunch velodyne_pointcloud VLP16_points.launch
//roscore
//rosrun rviz rviz -f velodyne

//sudo ifconfig enxd481d7527b1e 192.168.1.200
//sudo route add 102.169.1.201 enxd481d7527b1e
//sudo ifconfig eno1 192.168.1.200
//sudo route add 102.169.1.201 eno1
//ping 192.168.1.201

//http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20HDL-32E

//https://github.com/OpenSLAM/LeGO-LOAM
//https://github.com/erik-nelson/blam/issues/48
//https://answers.ros.org/question/188309/what-does-source-command-actually-do/
//source devel/setup.bash
//roslaunch lego_loam run.launch
//https://stackoverflow.com/questions/52420672/ros-rviz-how-to-visualize-a-point-cloud-that-doesnt-have-a-fixed-frame-transfo

//Make sure to restart lidar provider publisher and to chnage the parameter for real time use of the system in its configuration file.

//SAVE ALL DATA 
//http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data

///EDIT: home/nasos/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h
// extern const bool loopClosureEnableFlag = true; //use true

//edit : https://github.com/OpenSLAM/LeGO-LOAM
//if (abs(angle - sensorMountAngle) <= 20,40) (in imageProjection.cpp)
//cd catkin_ws
//catkin_make




//// BLAM

// https://github.com/erik-nelson/blam

// echo $ROS_PACKAGE_PATH
// /home/nasos/catkin_ws/src:/opt/ros/kinetic/share

// https://github.com/erik-nelson/blam/issues/34
// cd ~/BLAM/internal/src/geometry_utils
// patch -p1 <~/BLAM/geometry_utils.patch.txt
// ./update

// TOP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
//source ${TOP_DIR}/internal/devel/setup.bash --extend


//roslaunch blam_example test_online.launch

// https://github.com/FredHuangBia/SLAM




#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace sensor_msgs;


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


#include <velodyne_pointcloud/point_types.h>

//SURFACE RECONSTRUCTION
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/pcl_config.h>


#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;



/// @todo make sure these includes are really necessary - https://github.com/ros-drivers/velodyne/blob/master/velodyne_pointcloud/src/conversions/colors.cc#L63
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
/** types of output point and cloud */
typedef pcl::PointXYZRGB RGBPoint;
typedef pcl::PointCloud<RGBPoint> RGBPointCloud;

// shorter names for point cloud types in this namespace
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

 // RGB color values
  const int color_red =       0xff0000;
  const int color_orange =    0xff8800;
  const int color_yellow =    0xffff00;
  const int color_green =     0x00ff00;
  const int color_blue =      0x0000ff;
  const int color_violet =    0xff00ff;

  const int N_COLORS = 6;
  int rainbow[N_COLORS] = {color_red, color_orange, color_yellow,
color_green, color_blue, color_violet};
RGBPointCloud::Ptr outMsg(new RGBPointCloud());
//END ADD LiDAR 


//LIDAR CLOUD PARSING
//https://github.com/abhi-kumar/OPENCV_MISC/blob/master/opencv_pcl_filter.cpp
//#include <iostream> 
// Point cloud library
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h> 
// Opencv
//#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp> 
//using namespace cv;
int a = 180;
int b = 180;
int c=  180; 
// PCL Visualizer to view the pointcloud
pcl::visualization::PCLVisualizer viewer ("Get the view here");
//FIND NORMALS
#include <pcl/features/normal_3d.h>
//CREATE 21EDRON DETECTOR
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"
using namespace cv::ppf_match_3d;
cv::ppf_match_3d::PPF3DDetector SolidDetector(0.025, 0.05);
//END LiDAR CLOUD PARSING 



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

#include <time.h>
time_t start = time(0);//clock();//

//ARDUINO COM READ
#include <stdlib.h>
#include <stdio.h>
//#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#define DEBUG 1
#include <thread>

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
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod; //OCAM not supported in ROS openCV ?
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

//ARDUINO COM READ
int fd, n, i;
char buf[256] = "temp text";
vector<int> pozyxOUTPUT;

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
					if(1==0 && nCombinations > 2 && centersWithAndWithoutDistance > distThreshold){						
						cout << "ERROR ID = " << errorID << " MAX DISTANCE = " << currentBiggestDist 
						<< " DISTANT COUNT = " << outofThresCount << endl; 
						outlierComboIDs.push_back(errorID);

						//reconstruct the matrices without the found erroneous marker
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + errorIter);
						centerPointsTMP.erase(centerPointsTMP.begin() + errorIter);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + errorIter);						
				
						// re-run the loop until no erronous is found, then break
						discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP );
					}
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
Ptr<aruco::GridBoard> gridboard;
double totalTime = 0;
int totalIterations = 0;
int totalArucoMarkersFoundIterations=0;
int totalPozyxFoundIterations=0;
double totalError = 0;
Ptr<aruco::Board> board;
int waitTime;
vector< Point2f > boundingRectimagePoints;

//MEASUREMENT FUSION
float varianceSquaredARUCO=0.000001f;
float varianceSquaredPOZYX=0.000001f;
float varianceSquaredLIDAR=0.000001f;
float distanceMEAN_ARUCO=0;
float distanceMEAN_POZYX=0;

float distanceMEAN_tX_POZYX=0;
float distanceMEAN_tY_POZYX=0;
float distanceMEAN_tZ_POZYX=0;
int forgetWindowIgterationsPOSYZ_mean = 0;

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
	dictionary = cv::aruco::generateCustomDictionary(102,5);

	//COLOR OR B&W CHOICE (COLOR BELOW)
	//VideoCapture inputVideo;		
	//END COLOR OR B&W CHOICE (COLOR BELOW)

	float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
		                       markerSeparation);

	// create board object
	gridboard = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);

	board = gridboard.staticCast<aruco::Board>();
 
}//END ARUCO INIT FUNCTION


//KEEP MEMORY OF MEASUEMENTS
Vec3d tvecsBaseCprevious;
Vec3d tvecsBaseCprevious1;
Vec3d tvecsBaseCprevious2;
Vec2d tvecsBase2DCprevious;
Vec2d tvecsBase2DCprevious1;
Vec2d tvecsBase2DCprevious2;
int toalFrameCountwithMarkers;
//#define  CLAMP(x, a, b)  ( (x) > (b) ? (b) : ( (x) < (a) ? (a) : (x) ) )
int idsCountPrevFrame;
float Center_to_Cam_distPrevFrame;
int frameShiftX;
int frameShiftY;//shift of frame after crop

int clamp1(int x,int a,int b){
	if(x > b){
		return b;	
	}else{
		if(x < a){
			return a;	
		}else{
			return x;
		}
	}
	//return (x) > (b) ? (b) : ( (x) < (a) ? (a) : (x) );
}


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
//int errorID = -1;
Mat transResultC = (Mat_<double>(1,3) << 0,0,0);
float distSideToSolidCenter;
float actualRectSideSize;
//summaries
int identifiedTriangleCount=0;
int identifiedRectangleCount=0;
cv::Vec<double, 3> get19;
cv::Vec<double, 3> get27;
vector< int > ids;
float Center_to_Cam_dist = -1;


void resetVTempVariables(){

centerPoints.clear();
centerPointsIDs.clear();
centerRotations.clear();
//Vec3f toEuler; 
counted = 0;
//Vec3d rvecsBaseC, tvecsBaseC;
rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
//separate inliers and outliers
inlierIDs.clear();
outlierIDs.clear();
transResultC = (Mat_<double>(1,3) << 0,0,0);
distSideToSolidCenter=0;
actualRectSideSize=0;
//summaries
identifiedTriangleCount=0;
identifiedRectangleCount=0;
//cv::Vec<double, 3> get19;
//cv::Vec<double, 3> get27;
ids.clear();
Center_to_Cam_dist = -1;

}

void resetVTempVariables2(){

cout << "centerPoints " << centerPoints.size() << "centerPointsIDs " << centerPointsIDs.size() << "centerRotations " << centerRotations.size() << "counted " << counted << endl;		

centerPoints.clear();
centerPointsIDs.clear();
centerRotations.clear();
counted = 0;
}

void arucoProcessmarkerID(Mat& image,Mat& srcImg,Mat& imageCopy, float ExposureAbsolute, int outWindowID) {

	

	//ARUCO FOUND VECTORS (temp variables)
       
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

	//Marker pose matrices
	vector<Vec3d> rvecs, tvecs;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

	// refind strategy to detect more markers
        if(refindStrategy){
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix, distCoeffs);
	    //cout << "REFINE ON = " << refindStrategy << endl;
	}

        // estimate board pose
        int markersOfBoardDetected = 0;
	//if(ids.size() > 0){
		//markersOfBoardDetected = aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
	//}

	//MARKER SIZE DEFINITION
	float tileSize = 0.025f; //small test cube
	//tileSize = 0.08; //8cm test
	tileSize = 0.0635f; //bigger solid A3 page test
	tileSize = 0.0690f; //final solid size			//CHANGE HERE

	if(shapeID==1){ //if Cube shape
		tileSize = 0.109f;//0.126f;
	}

	actualRectSideSize = (8.0f/7.0f) * tileSize;//assuming one extra tile around the 7x7 (5x5 tile) marker (half left, half right)

	if(shapeID==0){ //CORRECTION of 8/7 of 6.9, since actual 21 edron boundary is at 7.5 than 7.885
		actualRectSideSize = 0.075f;
	}

	distSideToSolidCenter = ((sqrt(2.0f)+1.0f)/2.0f) * actualRectSideSize;

	if(shapeID==1){ //if Cube shape
		distSideToSolidCenter = actualRectSideSize/2.0f;
	}

	//triangle calculations - distance to center, offset to triangle center
	//float distTriangleToSolidCenter = 0.45542f * sqrt(5+(2*sqrt(2)))*actualRectSideSize;
	//float DA = 0.2064f * sqrt(5+(2*sqrt(2)))*actualRectSideSize;
	//float BA = (sin(PI/3)/(1+sin(PI/3)))*actualRectSideSize;
	//float CA = BA/2;
	//float YoffsetTriangle = sqrt(3/4)*actualRectSideSize - DA - CA; //offset from found marker axis in Y to get to triangle center
	//cout << "YoffsetTriangle = " << YoffsetTriangle << " distTriangleToSolidCenter = " << distTriangleToSolidCenter << endl;
 	//cout << "tileSize = " << tileSize << " actualRectSideSize =" <<  actualRectSideSize << endl;

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
		if(showRejected && rejected.size() > 0){
	    		aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
		}
	}

	//if(markersOfBoardDetected > 0)
    	//aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

	// DEBUG & ARUCO OUTPUT HOLDING PARAMETERS
	// if we find markers
	
	cv::Vec<double, 3> get27P;
	cv::Mat get27P_dist_coeff;
	double camz27;
	
	

	//ANALYSE FOUND MARKERS
	if (ids.size() > 0) {

		idsCountPrevFrame = ids.size();
		toalFrameCountwithMarkers++; //increase counter of frames in which markers were found

		if(preview == 1){
			aruco::drawDetectedMarkers(imageCopy, corners, ids);
		}

		//GATHER CONSENSUS RESULTS BELOW
		//Mat rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
		
		//vector<cv::Vec<double, 3>> centerPoints;
		//vector<int> centerPointsIDs;
		//std::vector<cv::Mat> centerRotations;

		for (int i = 0; i < ids.size(); i++) {

			if(preview == 1){
				aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);				
			}

			//cam pose
			cv::Mat R;
			cv::Rodrigues(rvecs[i],R);
			cv::Mat camPose = -R.t() * (cv::Mat)tvecs[i];

			double camx = camPose.at<double>(0,0);
			double camy = camPose.at<double>(0,1);
			double camz = camPose.at<double>(0,2);					
						
			// draw axis lines
			//line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);

			//find axis in relation to camera space for Z axis (0,0,0 to 1,1,1)
			cv::Vec<double, 3> axisPoints0;
			axisPoints0(0)=0;
			axisPoints0(1)=0;
			axisPoints0(2)=0;
			cv::Vec<double, 3> axisPoints1;
			axisPoints1(0)=0;
			axisPoints1(1)=0;
			axisPoints1(2)=1;
			//rotate points to camera space, to find angle difference based on common axis
			cv::Mat RotatedToCameraSpaceaxisPoints0 = R * (cv::Mat)axisPoints0;
			cv::Mat RotatedToCameraSpaceaxisPoints1 = R * (cv::Mat)axisPoints1;
			
			//assign IDs starting from left or central
			//small solid (0.025)
			int centralID = 9;
			int leftID1 = 31;
			int leftID2 = 17;
			int leftID3 = 32;
			int leftID4 = 29;
			int leftID5 = 33;
			int leftID6 = 27;
			int leftID7 = 30;
			//large solid (0.0635) - 
			centralID = 0;
			leftID1 = 1;
			leftID2 = 2;
			leftID3 = 3;
			leftID4 = 4;
			leftID5 = 5;
			leftID6 = 6;
			leftID7 = 7;
			int centralUpperID = 8; //CHANGE HERE
			int leftUpperID1 = 10;
			int leftUpperID2 = 12;
			int leftUpperID3 = 14;
			int topID = 16;

			int leftUpperTriangleID1 = 9;
			int leftUpperTriangleID2 = 11;
			int leftUpperTriangleID3 = 13;
			int leftUpperTriangleID4 = 15;

			int exposureDEBUG = ExposureAbsolute; //int exposureDEBUG = camera.get_control("Exposure (Absolute)");
			//cout << "exposure = " << exposureDEBUG <<  endl;

			//2ond row
			int centralUpID = 4;
			//int leftUpID1 = ;

			//dot product 19 and 27
			if(ids[i] == 0){
				//get19 = rvecs[i];
				get19 = RotatedToCameraSpaceaxisPoints1;			
			}
			if(ids[i] == 8){
				get27 = RotatedToCameraSpaceaxisPoints1;
			}			

			//if(ids[i] != 13 && ids[i] != 5 && ids[i] != 6  && ids[i] != 7  && ids[i] != 8   && ids[i] != 10  && ids[i] != 11  && ids[i] != 12 ){ //24
			if(ids[i] != 9 && ids[i] != 11 && ids[i] != 13  && ids[i] != 15){								

				//rotate final axis based on rotation vs centralID marker axis				
				Mat rotMatrixFinal = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); 
				float th = 0;
				if(ids[i] == leftID1){ 
					//around Y, 45 degrees
					th = -1.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID2 || ids[i] == leftUpperID1){ 
					//around X, 90 degrees
					th = -2.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID3){ 
					//around Y, 45 degrees
					th = -3.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID4 || ids[i] == leftUpperID2){ 
					//around Y, 45 degrees
					th = -4.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID5){ 
					//around Y, 45 degrees
					th = -5.0f*(PI/4.0f);					 
				}
				if(ids[i] == leftID6 || ids[i] == leftUpperID3){ 
					//around Y, 45 degrees
					th = -6.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID7){ 
					//around Y, 45 degrees
					th = -7.0f*(PI/4.0f);					
				}
				
				rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1); 

				//ROTATE UPPER PARTS around X axis (rotated in Y axis above already)
				if(ids[i] == leftUpperID1 || ids[i] == leftUpperID2 || ids[i] == leftUpperID3){ 
					th = 1.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1)*rotMatrixFinal;
				}
				if(ids[i] == centralUpperID){ 
					th = 1.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}						
				if(ids[i] == topID){ 
					th = 2.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}

				//plot axis of solid center
				Vec3d rvecsBase, tvecsBase;
								
				//cast translation vector to matrix				
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,-distSideToSolidCenter, 0,0,0,1); 

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

				//CONSENSUS - MOVED OUTSIDE if MARKERS > 0 CHECK - rotResultC is for debug
				//float center_diffX = transResultC.at<double>(0,0) - centerPose.at<double>(0,3);		
				//float center_diffY = transResultC.at<double>(0,1) - centerPose.at<double>(1,3);	
				//float center_diffZ = transResultC.at<double>(0,2) - centerPose.at<double>(2,3);			
				//float Center_to_Prev_Center_dist = sqrt(pow(center_diffX,2) + pow(center_diffY,2) + pow(center_diffZ,2));
				//if (Center_to_Prev_Center_dist < 0.1){

					add_to_33_matrix (rotResultC, centerPose);
					scale33_matrix(rotResultC, 2.0f);
					
					transResultC.at<double>(0,0) = (centerPose.at<double>(0,3) + transResultC.at<double>(0,0)) / 2.0f;
					transResultC.at<double>(0,1) = (centerPose.at<double>(1,3) + transResultC.at<double>(0,1)) / 2.0f;
					transResultC.at<double>(0,2) = (centerPose.at<double>(2,3) + transResultC.at<double>(0,2)) / 2.0f;
				//}			

				//cout << "DEBUG Translation to Center = " << transResult << " DEBUG centerPose Marker ID = " << ids[i] <<  endl;

				//get final rotation and turn it to vector
				//rvecsBase.at<double>(0,0) =
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;

				//plot base axis
				if(preview == 1){
					//aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecsBase, tvecsBase, markerLength * 2.2f);
				}

				//Measurements (1.70m lab table)

				//CALCULATE ROTATION PER PART ID TO CENTER
				if(1==1){
					cv::Mat R1;
					R1 = R;
					double theta = 0; //row,column
					R1.at<double>(0,0) = cos(theta);
					R1.at<double>(0,1) = 0;
					R1.at<double>(0,2) = -sin(theta);

					R1.at<double>(1,0) = 0;
					R1.at<double>(1,1) = 1;
					R1.at<double>(1,2) = 0;

					R1.at<double>(2,0) = sin(theta);
					R1.at<double>(2,1) = 0;
					R1.at<double>(2,2) = cos(theta);

					//Translation vectors					
					cv::Mat camPose9 = -R.t() * ((cv::Mat)tvecs[i]);

					double cam9x = camPose9.at<double>(0,0);
					double cam9y = camPose9.at<double>(0,1);
					double cam9z = camPose9.at<double>(0,2);

					//different approach, direct measurement
					cam9x =tvecs[i](0);
					cam9y =tvecs[i](1);
					cam9z =tvecs[i](2);
				}
				//END CALCULATE ROTATION PER PART ID TO CENTER					

				get27P = tvecs[i];
				get27P_dist_coeff = distCoeffs;	
				camz27 = sqrt(camx*camx + camy*camy +camz*camz);				

				if(preview == 1 && totalIterations % 30 == 0) {
					cout << "DIST = " << camz27 <<  endl;
				}

				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				//centerPoints(i) = transResultVec;

				if(shapeID != 1 || (shapeID == 1 && 
					(ids[i] == centralID || ids[i] == leftID2 || ids[i] == leftID4 || ids[i] == leftID6 || ids[i] == topID)) 
				){
					centerPoints.push_back(transResultVec);	
					centerRotations.push_back(rotResult);

					//summaries				
					identifiedRectangleCount++;
					centerPointsIDs.push_back(ids[i]);
				}

			}// END IDs check to not be triangles (cover only rectangles)
			else if(shapeID != 1){ //if NOT Cube handle triangles

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
				Vec3d rvecsBase, tvecsBase;				
				
				//CHECK if dist is same
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,SideW, 0,0,1,-SideTR, 0,0,0,1);

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

				//CONSENSUS - MOVED OUTSIDE if MARKERS > 0 CHECK - rotResultC is for debug
				float center_diffX = transResultC.at<double>(0,0) - centerPose.at<double>(0,3);		
				float center_diffY = transResultC.at<double>(0,1) - centerPose.at<double>(1,3);	
				float center_diffZ = transResultC.at<double>(0,2) - centerPose.at<double>(2,3);			
				float Center_to_Prev_Center_dist = sqrt(pow(center_diffX,2) + pow(center_diffY,2) + pow(center_diffZ,2));
				if (Center_to_Prev_Center_dist < 0.05 || 
					(transResultC.at<double>(0,0) == 0 && transResultC.at<double>(0,1) == 0 && transResultC.at<double>(0,2) == 0)
				){
					add_to_33_matrix (rotResultC, centerPose);
					scale33_matrix(rotResultC, 2.0f);
					
					transResultC.at<double>(0,0) = (centerPose.at<double>(0,3) + transResultC.at<double>(0,0)) / 2.0f;
					transResultC.at<double>(0,1) = (centerPose.at<double>(1,3) + transResultC.at<double>(0,1)) / 2.0f;
					transResultC.at<double>(0,2) = (centerPose.at<double>(2,3) + transResultC.at<double>(0,2)) / 2.0f;
				}

				//cout << "DEBUG Translation to Center = " << transResult << " DEBUG centerPose Marker ID = " << ids[i] <<  endl;

				//get final rotation and turn it to vector				
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;

				//plot base axis
				if(preview == 1){
					aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecsBase, tvecsBase, markerLength * 2.2f);
				}

				//cout << "SideW = " << SideW <<  " SideTR = " << SideTR << " distSideToSolidCenter=" << distSideToSolidCenter << endl;
				//SideW = 0.00445395 SideTR = 0.100482 distSideToSolidCenter=0.095189

				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				centerPoints.push_back(transResultVec);	
				centerRotations.push_back(rotResult);	
				centerPointsIDs.push_back(ids[i]);		
			}	

			//cout << "Detection Angle " << ids[i] << " = " << rvecs[i] << " vector " << endl;
			//cout << "Cam dists " << RotatedToCameraSpaceaxisPoints << endl;			
		
		}//END FOR LOOP over IDs found
	}//END check if ids found > 0
}//END ARUCO MARKER IDENTIFICATION FUNCTION

//////////////// END ARUCO MARKER ID /////////

//cv::Mat srcImgBright;

bool arucoProcess(Mat srcImg, float ExposureAbsolute, int outWindowID) {

	

   	//OCAM - ARUCO
	Mat image, imageCopy;

	//BOUNDING BOX CREATION

	int ROI_Center_X = -1;
	int ROI_Center_Y = -1;		
	int ROI_Edge_X   = -1;
	int ROI_Edge_Y   = -1;

	//ENABLE BOUNDING CALC WIHTOUT autoFocus on
	bool forceBoundingCalc = false;
	if(alpha > 1){
		forceBoundingCalc = true;
	}


	float distFactor = Center_to_Cam_distPrevFrame;

	//cout << "toalFrameCountwithMarkers = " << toalFrameCountwithMarkers 
	//<< " distFactor =" << distFactor
	//<< " boundingRectimagePoints.size() =" << boundingRectimagePoints.size()
	//<< " idsCountPrevFrame =" << idsCountPrevFrame << endl;

	
	if(toalFrameCountwithMarkers > 5 && idsCountPrevFrame > 0 && distFactor > 0.2f && distFactor < 40.0f 
	&& (autoFocus==1 || forceBoundingCalc) && boundingRectimagePoints.size() > 0){
				
		if(forceBoundingCalc){
			//actual image was not cropped in this case, so there is no shift
			frameShiftX = 0;
			frameShiftY = 0;
		}
		
		
		if(distFactor <= 0 || distFactor > 40){ //CHANGE HERE
			distFactor = 1;
		}
		
		int windowX = (srcImg.size().width*5) / pow((1+distFactor)*2,2.5);
		int windowY = (srcImg.size().height*5) / pow((1+distFactor)*2,2.5);

		if(preview == 1){
			cout << "Width = " << srcImg.size().width << " Height =" << srcImg.size().height << endl;
			cout << "windowX = " << windowX << " windowY =" << windowY << endl;
		}

		ROI_Center_X = clamp1(tvecsBase2DCprevious(0)-windowX + frameShiftX,0,srcImg.size().width  -  frameShiftX);
		ROI_Center_Y = clamp1(tvecsBase2DCprevious(1)-windowY + frameShiftY,0,srcImg.size().height -  frameShiftY);		
		ROI_Edge_X   = clamp1(2*windowX+ 0,0,srcImg.size().width  - ROI_Center_X);
		ROI_Edge_Y   = clamp1(2*windowY+ 0,0,srcImg.size().height - ROI_Center_Y);

		//2ond method - EXACT BOUNDING BOX
		if(boundingRectimagePoints.size() > 0){
			ROI_Center_X= clamp1(boundingRectimagePoints[3].x + frameShiftX,0,srcImg.size().width - frameShiftX);
			ROI_Center_Y= clamp1(boundingRectimagePoints[3].y + frameShiftY,0,srcImg.size().height - frameShiftY);
			ROI_Edge_X  = clamp1(abs(boundingRectimagePoints[1].x - boundingRectimagePoints[3].x) ,0,srcImg.size().width  - ROI_Center_X);
			ROI_Edge_Y  = clamp1(abs(boundingRectimagePoints[1].y - boundingRectimagePoints[3].y) ,0,srcImg.size().height  - ROI_Center_Y);			
		}

		frameShiftX = ROI_Center_X;
		frameShiftY = ROI_Center_Y;

		if(ROI_Edge_X <= 0){
			ROI_Edge_X = 1;		
		}
		if(ROI_Edge_Y <= 0){
			ROI_Edge_Y = 1;		
		}
		if(preview == 1){
			cout << "ROI_Center_X = " << ROI_Center_X << " ROI_Center_Y =" << ROI_Center_Y << endl;
			cout << "ROI_Edge_X = " << ROI_Edge_X << " ROI_Edge_Y =" << ROI_Edge_Y << endl;
			cout << "tvecsBase2DCprevious(0) = " << tvecsBase2DCprevious(0) << " tvecsBase2DCprevious(1) = " << tvecsBase2DCprevious(1) << endl;
		}		

		cv::Rect myROI(ROI_Center_X,ROI_Center_Y,ROI_Edge_X,ROI_Edge_Y);		
		
		if(autoFocus==1){
			image = srcImg(myROI);
		}else{
			image = srcImg;
		}		
	}else{
		image = srcImg;
		frameShiftX = 0;
		frameShiftY = 0;
	}


	Vec3d rvecsBaseC_POZYX, tvecsBaseC_POZYX;
	Mat transResultC_POZYX = (Mat_<double>(1,3) << 0,0,0);
	Mat rotResultC_POZYX = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);

	//TIMER
 	double tick = (double)getTickCount();

	resetVTempVariables();
	arucoProcessmarkerID(image, srcImg,imageCopy, ExposureAbsolute, outWindowID);

	if(1==1 && alpha > 1 && !image.empty()){


		//save variables then reset
		//resetVTempVariables2();

		if(preview == 1){
			imshow("out"+to_string(2), imageCopy);
			//Mat imageUndistorted;
			//undistort(image, imageUndistorted,camMatrix, distCoeffs);
			//imshow("outUndistorted", imageUndistorted);
		}

		//REDO FOR ONLY THE  BRIGHT IMAGE PART, DEFINED BY THE BOUNDING BOX !!!!!!!!!!!!	
		//BRIGHTEN THE IMAGE IN BOUNDING BOX
		
		//cv::Mat srcImgBright = Mat::zeros( image.size(), image.type() );	
		
		//BRIGHTNESS & CONTRAST
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		
		//CROPED
		if(1==1 && ROI_Center_X != -1){
			//IF BOUNDING FOUND, BRIGHTEN PART

			//Since origin is in top-left 
			//and positive values to go to bottom, in OpenCV, axis are:

			//0/0--X-->
			// |
			// Y
			// |
			// v

			//In OpenCV use: mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y)) 
			//to access the same point if x=column and y=row

			//cv::Size s = mat.size();
			//rows = s.height;
			//cols = s.width;

			//cout << "ROI_Center_X = " << ROI_Center_X << " ROI_Center_Y =" << ROI_Center_Y << "Image width =" << image.size().width <<endl;
			//cout << "ROI_Edge_X = " << ROI_Edge_X << " ROI_Edge_Y =" << ROI_Edge_Y << "Image height =" << image.size().height << endl;
			//ROI_Center_X = 741 ROI_Center_Y =577
			//ROI_Edge_X = 190 ROI_Edge_Y =136
			//ROI_Center_X = 742 ROI_Center_Y =577Image width =1280
			//ROI_Edge_X = 190 ROI_Edge_Y =136Image height =960
			//ROI_Center_X = 740 ROI_Center_Y =576Image width =1280
			//ROI_Edge_X = 191 ROI_Edge_Y =137Image height =960
			
			//srcImgBright = image;
			for( int y = ROI_Center_Y; y < ROI_Center_Y+ROI_Edge_Y; y++ ){ 
				for( int x = ROI_Center_X; x < ROI_Center_X+ROI_Edge_X; x++ ){ 
					for( int c = 0; c < 3; c++ ){		      		
						image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*( image.at<Vec3b>(y,x)[c] ) + beta );
						image.at<Vec3b>(y,x)[c] = image.at<Vec3b>(y,x)[c] 
						- image.at<Vec3b>(y,x)[c]*(10/(pow(1-(alpha*image.at<Vec3b>(y,x)[c]),2))  );
					}
				}
			}
			arucoProcessmarkerID(image, srcImg, imageCopy, ExposureAbsolute, 2);
			//cout << "ROI_Center_ IS USED = " << ROI_Center_X << " ROI_Center_Y =" << ROI_Center_Y << endl;
		}else{
			//ELSE BRIGHTEN WHOLE IMAGE
			image.convertTo(image, -1, alpha, beta);
			arucoProcessmarkerID(image, srcImg,imageCopy, ExposureAbsolute, outWindowID);
		}
	}

  	if (centerPointsIDs.size() > 0) { //if (ids.size() > 0) {

		//CONSENSUS
		//Vec3d rvecsBaseC, tvecsBaseC;

		//separate inliers and outliers		
		inlierIDs.clear();
		outlierIDs.clear();

		int consensusMethod = 2;
		//int counted = 0;
		cv::Vec<double, 3> centerMean;
		// PROJECT GLOBAL CENTER to get point in image to do the cropping with ...
		// project axis points
		vector< Point3f > axisPoints;
		axisPoints.push_back(Point3f(0, 0, 0));
		axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
		axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
		axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));
		vector< Point2f > imagePoints;
		//Mat imagePointsVelocities;//jacobian vs camera tranlation(3)-rotation(3)-focal(2)-principal point(2)-distort coeff(2+3)
		//method 0 = mean, 1 = combinations, 2 = distance based clustering

		//METHOD 2		
		if(consensusMethod == 2){
			//check if distances are below a threshold, create a cluster, if not create two and go on clasifying entries to them
			
			int countedA=0;
			for (int i = 0; i < centerPointsIDs.size()-1; i++) {
				for (int j = i+1; j < centerPointsIDs.size(); j++) {					
					float Center_to_Prev_Center_dist = matrix31_distance(centerPoints[i],centerPoints[j]);
					if(Center_to_Prev_Center_dist < 0.05f){ //make center biased towards the inliers
						add_to_31_matrix(centerMean, centerPoints[i]);						
						countedA++;
					}
				}
			}
			if(countedA > 0){
				scale31_matrix(centerMean, countedA);				
			}
			if(centerPointsIDs.size() == 1){
				matrix32_assign_from_to(centerPoints[0], centerMean);							
			}
			
		}//END METHOD 2 

		//METHOD 0
		if(consensusMethod == 2){
			cv::Rodrigues(rotResultC,rvecsBaseC);
			tvecsBaseC = transResultC;			

			if(preview == 1){
				cout << "Center MEAN = " << centerMean << endl;
			}
			//If distance from mean center above threshold, dont count in centre calculation (reset previous loop based created consensus)
			transResultC.at<double>(0,0) = 0;
			transResultC.at<double>(0,1) = 0;
			transResultC.at<double>(0,2) = 0;		
			
		}//END METHOD 0				

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
			
			discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP );

			//cout << "OutlierComboIDs = " << outlierComboIDs.size() << endl;

			//EVALUATE EXTREME CASES and SMALL VS BIG RATIO (if one big, small may skew the result, give more to big !!)
			if(centerPointsTMP.size() == 2){
				int foundTriangle = 0;
				int rectangleID = -1;
				int triangleID = -1;
				//if 2 and one is triangle and the other rectangle, keep rectangle unless very close
				for (int i = 0; i < centerPointsIDsTMP.size(); i++) {
					if(centerPointsIDsTMP[i] == 9 || centerPointsIDsTMP[i] == 11 || centerPointsIDsTMP[i] == 13  || centerPointsIDsTMP[i] == 15){
						foundTriangle += 1;
						triangleID = i;
					}else{
						rectangleID = i;
					}
				}

				if(foundTriangle == 1){					
					centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + triangleID);
					centerPointsTMP.erase(centerPointsTMP.begin() + triangleID);
					centerRotationsTMP.erase(centerRotationsTMP.begin() + triangleID);
				}
			}
			if(centerPointsTMP.size() == 3){
				int foundTriangle = 0;
				int rectangleID = -1;
				int triangleID = -1;
				//if 2 and one is triangle and the other rectangle, keep rectangle unless very close
				for (int i = 0; i < centerPointsIDsTMP.size(); i++) {
					if(centerPointsIDsTMP[i] == 9 || centerPointsIDsTMP[i] == 11 || centerPointsIDsTMP[i] == 13  || centerPointsIDsTMP[i] == 15){
						foundTriangle += 1;
						triangleID = i;
					}else{
						rectangleID = i;
					}
				}
				vector<int> foundTrianglesIDs;  
				if(foundTriangle == 2){		
					for (int i = 0; i < centerPointsIDsTMP.size(); i++) {
						if(centerPointsIDsTMP[i] == 9 || centerPointsIDsTMP[i] == 11 
						   || centerPointsIDsTMP[i] == 13  || centerPointsIDsTMP[i] == 15){
							//if distance big to the rectangle center, remove triangle
								
							float Center_to_Prev_Center_dist = matrix31_distance(centerPointsTMP[rectangleID],centerPointsTMP[i]);

							if(Center_to_Prev_Center_dist > 0.05f){		
								foundTrianglesIDs.push_back(i);								
							}
						}
					}
					for (int i = 0; i < foundTrianglesIDs.size(); i++) {
				   		//cout << "Center IDs of TRIANGLES = " 
						//<< centerPointsIDsTMP[foundTrianglesIDs[i]] << endl;						
					}
					for (int i = 0; i < foundTrianglesIDs.size(); i++) {
						//cout << "Center IDs of TRIANGLES = " << centerPointsIDsTMP[foundTrianglesIDs[i]] << endl;
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + foundTrianglesIDs[i]);
						centerPointsTMP.erase(centerPointsTMP.begin() + foundTrianglesIDs[i]);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + foundTrianglesIDs[i]);

						for (int j = 0; j < foundTrianglesIDs.size(); j++) {
							if(foundTrianglesIDs[j] > foundTrianglesIDs[i]){
								foundTrianglesIDs[j] -= 1;
							}
						}
					}
					for (int i = 0; i < foundTrianglesIDs.size(); i++) {
						//cout << "Center IDs of TRIANGLES = " 
						//<< centerPointsIDsTMP[foundTrianglesIDs[i]] << endl;						
					}					
				}
			}

			centerPoints    = centerPointsTMP;
			centerPointsIDs = centerPointsIDsTMP;
			centerRotations = centerRotationsTMP;
			//centerMean(0) = 0;
			//centerMean(1) = 0;
			//centerMean(2) = 0;
			zero31_matrix(centerMean);

			counted = 0;
			
			if(consensusMethod == 1){
				for (int i = 0; i < centerPointsIDs.size(); i++) {
					add_to_31_matrix(centerMean,centerPoints[i]);					
					if(preview == 1){
						cout << "Center ID = " << centerPointsIDs[i] << " Center ID Coords = " << centerPoints[i] << endl;
					}
				}
				scale31_matrix(centerMean, centerPointsIDs.size());				
				if(centerPointsIDs.size() == 1){
					matrix32_assign_from_to(centerPoints[0],centerMean);							
				}
			}

			if(preview == 1){
				cout << "Center MEAN 2 = " << centerMean << endl;
			}
			//If distance from mean center above threshold, dont count in centre calculation (reset previous loop based created consensus)
			transResultC.at<double>(0,0) = 0;
			transResultC.at<double>(0,1) = 0;
			transResultC.at<double>(0,2) = 0;
		
			
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
				//tvecsBaseC = tvecsBaseCprevious1;
				tvecsBaseC = transResultC;
				//return 0;
			}else{
				transResultC.at<double>(0,0) = (transResultC.at<double>(0,0)) / counted;
				transResultC.at<double>(0,1) = (transResultC.at<double>(0,1)) / counted;
				transResultC.at<double>(0,2) = (transResultC.at<double>(0,2)) / counted;
				tvecsBaseC = transResultC;//redo passing to final axis vector
			}
						
			//tvecsBaseCprevious = tvecsBaseC;//smoothen across frames, save previous found center consensus to use above
		
		
			//cout << "BEFORE ZEROING = " << rotResultC << endl;

			//ROTATIONS			
			zero33_matrix(rotResultC);

			//cout << "AFTER ZEROING = " << rotResultC << endl;
			Mat rotResultCTMP = (Mat_<double>(3,3) << 0,0,0, 0,0,0, 0,0,0);
			for (int i = 0; i < centerPointsIDs.size(); i++) {				

				//rotResultCTMP = add33_matrix(Mat &rotResultCTMP, Mat &centerRotations);
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

				
		//FORM EXACT BOUNDING BOX in 3D and PROJECT IT to get image bounds
		vector< Point3f > boundingBoxPoints;
		boundingBoxPoints.push_back(Point3f(0, 0, 0));
		boundingBoxPoints.push_back(Point3f( distSideToSolidCenter, -actualRectSideSize/2,  distSideToSolidCenter));//lower corners
		boundingBoxPoints.push_back(Point3f( distSideToSolidCenter, -actualRectSideSize/2, -distSideToSolidCenter));
		boundingBoxPoints.push_back(Point3f(-distSideToSolidCenter, -actualRectSideSize/2, -distSideToSolidCenter));
		boundingBoxPoints.push_back(Point3f(-distSideToSolidCenter, -actualRectSideSize/2, distSideToSolidCenter));
		
		boundingBoxPoints.push_back(Point3f( distSideToSolidCenter, distSideToSolidCenter, distSideToSolidCenter));//upper corners
		boundingBoxPoints.push_back(Point3f( distSideToSolidCenter, distSideToSolidCenter, -distSideToSolidCenter));
		boundingBoxPoints.push_back(Point3f(-distSideToSolidCenter, distSideToSolidCenter, -distSideToSolidCenter));
		boundingBoxPoints.push_back(Point3f( -distSideToSolidCenter, distSideToSolidCenter, distSideToSolidCenter));

		//EXTEND BOUNDING BOX TO COVER MOTION and lost markers due to bounding
		for (int i = 0; i < boundingBoxPoints.size(); i++) {
			float expandBy = 1;			
			if(forceBoundingCalc){
				expandBy = 1.2f;
			}
			boundingBoxPoints[i] = boundingBoxPoints[i] * expandBy;
		}
		
		vector< Point2f > boundingBoximagePoints;		
		//PROJECT Bounding box points
		projectPoints(boundingBoxPoints, rvecsBaseC, tvecsBaseC, camMatrix, distCoeffs, boundingBoximagePoints);

		//if(preview == 1 && boundingBoximagePoints[0].x>0 && boundingBoximagePoints[0].x < 1280
		//&& boundingBoximagePoints[0].y>0 && boundingBoximagePoints[0].y < 960){
		if(boundingBoximagePoints[0].x>0 && boundingBoximagePoints[0].x < 1280
			&& boundingBoximagePoints[0].y>0 && boundingBoximagePoints[0].y < 960 &&
			boundingBoximagePoints[1].x>0 && boundingBoximagePoints[1].x < 1280
			&& boundingBoximagePoints[1].y>0 && boundingBoximagePoints[1].y < 960 &&
			boundingBoximagePoints[2].x>0 && boundingBoximagePoints[2].x < 1280
			&& boundingBoximagePoints[2].y>0 && boundingBoximagePoints[2].y < 960 &&									
			boundingBoximagePoints[3].x>0 && boundingBoximagePoints[3].x < 1280
			&& boundingBoximagePoints[3].y>0 && boundingBoximagePoints[3].y < 960

			){
		//if(preview == 1){			

			//sides
			if(preview == 1){
				line(imageCopy, boundingBoximagePoints[1], boundingBoximagePoints[2], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[2], boundingBoximagePoints[3], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[3], boundingBoximagePoints[4], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[4], boundingBoximagePoints[1], Scalar(0, 255, 255), 3);

				line(imageCopy, boundingBoximagePoints[5], boundingBoximagePoints[6], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[6], boundingBoximagePoints[7], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[7], boundingBoximagePoints[8], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[8], boundingBoximagePoints[5], Scalar(0, 255, 255), 3);

				line(imageCopy, boundingBoximagePoints[1], boundingBoximagePoints[5], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[2], boundingBoximagePoints[6], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[3], boundingBoximagePoints[7], Scalar(0, 255, 255), 3);
				line(imageCopy, boundingBoximagePoints[4], boundingBoximagePoints[8], Scalar(0, 255, 255), 3);
			}
			//form 2D box
			float leftX  = boundingBoximagePoints[0].x;
			float rightX = boundingBoximagePoints[0].x;
			float upperY = boundingBoximagePoints[0].y;
			float lowerY = boundingBoximagePoints[0].y;
			for(int i=0; i < boundingBoximagePoints.size(); i=i+1){
				if(leftX > boundingBoximagePoints[i].x){
					leftX = boundingBoximagePoints[i].x;
				}
				if(rightX < boundingBoximagePoints[i].x){
					rightX = boundingBoximagePoints[i].x;
				}
				if(upperY < boundingBoximagePoints[i].y){
					upperY = boundingBoximagePoints[i].y;
				}
				if(lowerY > boundingBoximagePoints[i].y){
					lowerY = boundingBoximagePoints[i].y;
				}
			}
			//vector< Point2f > boundingRectimagePoints;
			if(boundingRectimagePoints.size() > 0){
				boundingRectimagePoints[0] = Point2f(leftX, upperY);
				boundingRectimagePoints[1] = Point2f(rightX, upperY);
				boundingRectimagePoints[2] = Point2f(rightX, lowerY);
				boundingRectimagePoints[3] = Point2f(leftX, lowerY);
			}else{
				boundingRectimagePoints.push_back(Point2f(leftX, upperY));
				boundingRectimagePoints.push_back(Point2f(rightX, upperY));
				boundingRectimagePoints.push_back(Point2f(rightX, lowerY));
				boundingRectimagePoints.push_back(Point2f(leftX, lowerY));
			}
			if(preview == 1){
				line(imageCopy, boundingRectimagePoints[0], boundingRectimagePoints[1], Scalar(0, 255, 0), 3);//bottom
				line(imageCopy, boundingRectimagePoints[1], boundingRectimagePoints[2], Scalar(255, 255, 0), 6);//right
				line(imageCopy, boundingRectimagePoints[2], boundingRectimagePoints[3], Scalar(0, 255, 255), 9);//top 
				line(imageCopy, boundingRectimagePoints[3], boundingRectimagePoints[0], Scalar(255, 255, 255), 12);//left
			}	
		}
		//END FORM EXACT BOUNDING BOX in 3D and PROJECT IT to get image bounds
		
		
		//KEEP PREVIOUS CENTERS and their PROJECTIONS
		tvecsBaseCprevious2 = tvecsBaseCprevious1;
		tvecsBaseCprevious1 = tvecsBaseCprevious;
		tvecsBaseCprevious = tvecsBaseC;
		tvecsBase2DCprevious2 = tvecsBase2DCprevious1;
		tvecsBase2DCprevious1 = tvecsBase2DCprevious;
		tvecsBase2DCprevious(0) = imagePoints[0].x;
		tvecsBase2DCprevious(1) = imagePoints[0].y;

		//plot base axis
		if(preview == 1){
			//aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecsBaseC, tvecsBaseC, markerLength * 7.5f);
			// draw axis lines
			if(imagePoints[0].x>0 && imagePoints[0].x < 1280 && imagePoints[0].y>0 && imagePoints[0].y < 960 &&
			imagePoints[1].x>0 && imagePoints[1].x < 1280 && imagePoints[1].y>0 && imagePoints[1].y < 960 &&
			imagePoints[2].x>0 && imagePoints[2].x < 1280 && imagePoints[2].y>0 && imagePoints[2].y < 960 &&
			imagePoints[3].x>0 && imagePoints[3].x < 1280 && imagePoints[3].y>0 && imagePoints[3].y < 960
			){
				line(imageCopy, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
				line(imageCopy, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
				line(imageCopy, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);	
			}			
		}
		//float Center_to_Cam_dist = sqrt(tvecsBaseC(0)*tvecsBaseC(0) + tvecsBaseC(1)*tvecsBaseC(1) + tvecsBaseC(2)*tvecsBaseC(2));
		Center_to_Cam_dist = sqrt(tvecsBaseC(0)*tvecsBaseC(0) + tvecsBaseC(1)*tvecsBaseC(1) + tvecsBaseC(2)*tvecsBaseC(2));
		Center_to_Cam_distPrevFrame = Center_to_Cam_dist;


		//IMAGE JACOBIAN
		int counted1=1;
		Mat imageJacobian = (Mat_<double>(2*counted1,6) );
		if(counted > 1 && Center_to_Cam_dist > 0 && 1==0) { //if(totalIterations % 30 == 0 && counted > 2 && Center_to_Cam_dist > 0){			
			float pixelSizeX = 0.00001f;
			float pixelSizeY = 0.00001f;
			//find focal length fx,fy from camera matrix
			float Fx = camMatrix.at<double>(0,0) * pixelSizeX;
			float Fy = camMatrix.at<double>(1,1) * pixelSizeY;
			//find principal point Px,Py from camera matrix
			float Px = camMatrix.at<double>(0,2) * pixelSizeX;
			float Py = camMatrix.at<double>(1,2) * pixelSizeY;
			float Depth = Center_to_Cam_dist;
			for(int i=0; i < counted1*2; i=i+2){
				//std::string veloc = "";
				//for(int j=0; j < 2; j++){					
					//float jm11 = 
					imageJacobian.at<double>(i,  0) = -1*(Fx/(pixelSizeX*Depth));	
					imageJacobian.at<double>(i+1,0) = 0;

					imageJacobian.at<double>(i,  1) = 0;	
					imageJacobian.at<double>(i+1,1) = -1*(Fy/(pixelSizeY*Depth));	

					imageJacobian.at<double>(i,  2) = (imagePoints[0].x - Px)/Depth;	
					imageJacobian.at<double>(i+1,2) = (imagePoints[0].y - Py)/Depth;	

					imageJacobian.at<double>(i,  3) = (pixelSizeX*(imagePoints[0].x-Px)*(imagePoints[0].y-Py))/(Fx);	
					imageJacobian.at<double>(i+1,3) = (pixelSizeX*pixelSizeX * 
					(imagePoints[0].y-Py)*(imagePoints[0].y-Py) + Fx*Fx)/(pixelSizeY*Fy);	

					imageJacobian.at<double>(i,  4) = -1*(pixelSizeY*pixelSizeY * 
					(imagePoints[0].x-Px)*(imagePoints[0].x-Px) + Fy*Fy)/(pixelSizeX*Fy);	
					imageJacobian.at<double>(i+1,4) = -1*(pixelSizeY*(imagePoints[0].x-Px)*(imagePoints[0].y-Py))/(Fy);

					imageJacobian.at<double>(i,  5) = (imagePoints[0].y - Py);	
					imageJacobian.at<double>(i+1,5) = -1*(imagePoints[0].x - Px);
					//veloc = veloc + std::to_string(camMatrix.at<double>(i,j)) +"  ";
				//}
				//cout << "Velocity " << i << " coeffs = " << veloc << endl;
			}
			//postprocess
			for(int i=0; i < counted1*2; i=i+2){				
				for(int j=0; j < 6; j++){
					imageJacobian.at<double>(i,j) = (int)(imageJacobian.at<double>(i,j) * 100) / 100;
					imageJacobian.at<double>(i+1,j) = (int)(imageJacobian.at<double>(i+1,j) * 100) / 100; //roundf ceilf floorf
					//if(imageJacobian.at<double>(i,j) < 0.00000001f){
						//imageJacobian.at<double>(i,j) = 0;
					//}
					//if(imageJacobian.at<double>(i+1,j) < 0.00000001f){
						//imageJacobian.at<double>(i+1,j) = 0;
					//}
					//imageJacobian.at<double>(i,j) = 0;
					//imageJacobian.at<double>(i+1,j) =0;
				}
			}
			//std::cout << "counted =" << counted << " imageJacobian = " << std::fixed << std::setprecision(2) << imageJacobian << std::endl;

			//BOUND THE CAMERA MOTION, in order to estimate the maximum translation of the bounding box 
			//cv::Vec<double, 6> maxCameraVelocities; //translations - rotations
			//maxCameraVelocities(0)=0.01f;
			//maxCameraVelocities(1)=0.01f;
			//maxCameraVelocities(2)=0.01f;
			//maxCameraVelocities(3)=0.01f;
			//maxCameraVelocities(4)=0.01f;
			//maxCameraVelocities(5)=0.01f;
			//cv::Vec<double, 2> centerPointMaxVelocities;
			//cout << "imageJacobian = " << imageJacobian << endl; //

			cv::Mat centerPointMaxVelocities;
			Mat maxCameraVelocities = (Mat_<double>(6,1) << 0.001,0.001,0.001,0.001,0.001,0.001) * 1;
			//cout << "maxCameraVelocities = " << maxCameraVelocities << endl; //			

			centerPointMaxVelocities = imageJacobian * maxCameraVelocities; //apply Jacobian to the found solid center
			//cout << "centerPointMaxVelocities = " << centerPointMaxVelocities << endl; //
		}
		//cout << "imageJacobian = " << imageJacobian << endl; //


		if(preview == 1){
			toEuler = rotationMatrixToEulerAngles(rotResultC)*(180.0f/PI);
			//rvecsBaseC = rvecsBaseC*(180.0f/PI);
			cout << "Center to Camera rotation " << toEuler << endl;
			cout << "Center to Camera distance " << Center_to_Cam_dist << endl;
			//summaries
			cout << "identifiedRectangleCount " << identifiedRectangleCount << endl;
			cout << "identifiedTriangleCount " << identifiedTriangleCount << endl;
		}

		//dot
		cv::Vec<double, 3> get19N;
		cv::Vec<double, 3> get27N;
		cv::normalize(get19,get19N,1.0,0.0,NORM_L2);
		cv::normalize(get27,get27N,1.0,0.0,NORM_L2);
		double dot19_27 = get19N.dot(get27N);
		//cout << "Dot 27 29 = " << dot19_27 << "   Detection Angle = " 
		//<< get27N  << "   Trans = " << get27P  << "   get27P_dist_coeff = " << get27P_dist_coeff << endl;
		////cout << "Dot 27 29 = " << dot19_27 << "   Detection Angle 27 = " << get27N  << "   29 = " << get19N  << endl;
		//cout << "Cam dist to 27 = " << camz27 << endl;

		totalError = totalError+(cv::abs(0.7071067811865475-dot19_27));  
		double error = totalError/totalIterations;
		if(totalIterations % 30 == 0) {
			cout << "Angle Error 0-8 (45 deg) = " << error << " Angle cos (should be 0.7071):" << dot19_27 << endl;
		}


		//FILE OUT
		//outfile << to_string(totalTime) << std::endl;
		std::string inliers = "";
		std::string outliers = "";
		for(int i=0; i < inlierIDs.size(); i++){	
			//inliers = inliers+std::to_string(inlierIDs[i])+";";
		}
		for(int i=0; i < outlierIDs.size(); i++){	
			//outliers = outliers+std::to_string(outlierIDs[i])+";";
		}

		//DO FOR NEW COMBINATIONS METHOD FOR OUTLIERS
		for(int i=0; i < centerPointsIDs.size(); i++){	
			inliers = inliers+std::to_string(centerPointsIDs[i])+";";
		}
		for(int i=0; i < outlierComboIDs.size(); i++){	
			outliers = outliers+std::to_string(outlierComboIDs[i])+";";
		}

		double seconds_since_start = difftime( time(0), start);

		if(totalIterations == 0){		
			gettimeofday(&beginT, NULL);
		}

		//TIMER ACCURATE
		gettimeofday(&endT, NULL);						
		double elapsed = (endT.tv_sec - beginT.tv_sec) + ((endT.tv_usec - beginT.tv_usec)/1000000.0);
		beginT = endT;		

		if(totalIterations == 0){			
			REAL_seconds_since_start = 0;
		}else{
			REAL_seconds_since_start = REAL_seconds_since_start + elapsed;
		}


		float varianceSquaredARUCO_final = -1;
		distanceMEAN_ARUCO = distanceMEAN_ARUCO + Center_to_Cam_dist;
		if(preview == 1){
			cout << "distanceMEAN_ARUCO ===== " << distanceMEAN_ARUCO << " iter=" << totalArucoMarkersFoundIterations 
			<< " MEAN=" << distanceMEAN_ARUCO/(totalArucoMarkersFoundIterations+1) << endl;
		}		
		varianceSquaredARUCO = varianceSquaredARUCO + pow(Center_to_Cam_dist-(distanceMEAN_ARUCO/(totalArucoMarkersFoundIterations+1)),2); //
		varianceSquaredARUCO_final = sqrt(varianceSquaredARUCO / (totalArucoMarkersFoundIterations+1));	

		
		if(usePOZYX==1){

			//POZYX frame rate (count number of samples in this Aruco iteration and compare with previous and the times the Aruco iterations happen)
			//Also use the sample difference to get the mean !!!

			float POZYX_OUTPUT_X = 0;
			float POZYX_OUTPUT_Y = 0;
			float POZYX_OUTPUT_Z = 0;
			float POZYX_OUTPUT_ROLL = 0;
			float POZYX_OUTPUT_PITCH = 0;
			float POZYX_OUTPUT_YAW = 0;
			if(pozyxOUTPUT.size() > 0){
				//REAL_seconds_since_start - REAL_PREV_seconds_since_start
			
				double secsDiffCurrentPrevArucoIteration = REAL_seconds_since_start - REAL_PREV_seconds_since_start;
				double samplePOZYXDiffCurrentPrevArucoIteration  = pozyxOUTPUT.size() - REAL_PREV_POZYX_samples;

				double POZYX_Frame_Rate = samplePOZYXDiffCurrentPrevArucoIteration / secsDiffCurrentPrevArucoIteration;	
				POZYX_Frame_Rate = POZYX_Frame_Rate / 6;
				//if(preview == 1){
					//cout << "POZYX_Frame_Rate = " << POZYX_Frame_Rate << endl;
				//}

				//cout << "pozyxOUTPUT.size() = " << pozyxOUTPUT.size() << " REAL_PREV_POZYX_samples= " << REAL_PREV_POZYX_samples << endl;

				int counterStart = REAL_PREV_POZYX_samples;
				//float testRounding = REAL_PREV_POZYX_samples % 6;
				//if(testRounding > 0){
				//	counterStart = (int)(REAL_PREV_POZYX_samples/6);
				//	counterStart = counterStart * 6;
				//}						
				int samplePOZYXDiffCurrentPrevArucoIterationROUNDED  = pozyxOUTPUT.size() - counterStart;

				//cout << "samplePOZYXDiffCurrentPrevArucoIterationROUNDED A = " 
				//<< samplePOZYXDiffCurrentPrevArucoIterationROUNDED << endl;

				int counterInner = 0;
			
				if(samplePOZYXDiffCurrentPrevArucoIterationROUNDED > 0){
					for (i=counterStart; i< pozyxOUTPUT.size(); i+=6){			
						POZYX_OUTPUT_YAW = POZYX_OUTPUT_YAW + pozyxOUTPUT[i];
						POZYX_OUTPUT_ROLL = POZYX_OUTPUT_ROLL + pozyxOUTPUT[i+1];
						POZYX_OUTPUT_PITCH = POZYX_OUTPUT_PITCH + pozyxOUTPUT[i+2];	
						POZYX_OUTPUT_X = POZYX_OUTPUT_X + pozyxOUTPUT[i+3];
						POZYX_OUTPUT_Y = POZYX_OUTPUT_Y + pozyxOUTPUT[i+4];
						POZYX_OUTPUT_Z = POZYX_OUTPUT_Z + pozyxOUTPUT[i+5];
						counterInner++;
					}
					samplePOZYXDiffCurrentPrevArucoIterationROUNDED = counterInner;	

					//cout << "samplePOZYXDiffCurrentPrevArucoIterationROUNDED " 
					//<< samplePOZYXDiffCurrentPrevArucoIterationROUNDED << endl;				
			
					POZYX_OUTPUT_X = POZYX_OUTPUT_X / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_Y = POZYX_OUTPUT_Y / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_Z = POZYX_OUTPUT_Z / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_ROLL = POZYX_OUTPUT_ROLL / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_PITCH = POZYX_OUTPUT_PITCH / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_YAW = POZYX_OUTPUT_YAW / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
				}else{			
					//for (i=pozyxOUTPUT.size()-6; i< pozyxOUTPUT.size(); i+=6){		
					//	POZYX_OUTPUT_YAW = pozyxOUTPUT[i];
					//	POZYX_OUTPUT_ROLL = pozyxOUTPUT[i+1];
					//	POZYX_OUTPUT_PITCH = pozyxOUTPUT[i+2];	
					//	POZYX_OUTPUT_X = pozyxOUTPUT[i+3];
					//	POZYX_OUTPUT_Y = pozyxOUTPUT[i+4];
					//	POZYX_OUTPUT_Z = pozyxOUTPUT[i+5];
						//counterInner++;
					//}
					for (i=0; i< pozyxOUTPUT.size(); i++){
						if(i>= pozyxOUTPUT.size()-6){
							if(i== pozyxOUTPUT.size()-4){
								//cout << "Pitch " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_PITCH = pozyxOUTPUT[i];
							}else
							if(i== pozyxOUTPUT.size()-5){
								//cout << "Roll " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_ROLL = pozyxOUTPUT[i];
							}else
							if(i== pozyxOUTPUT.size()-6){
								//cout << "Yaw " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_YAW = pozyxOUTPUT[i];
							}else
							if(i== pozyxOUTPUT.size()-3){
								//cout << "X " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_X = pozyxOUTPUT[i];
							}else
							if(i== pozyxOUTPUT.size()-2){
								//cout << "Y " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_Y = pozyxOUTPUT[i];
							}else
							if(i== pozyxOUTPUT.size()-1){
								//cout << "Z " << i << "= "<< pozyxOUTPUT[i] << endl;
								POZYX_OUTPUT_Z = pozyxOUTPUT[i];
							}
							//else{ cout << "XYZ= " << i << "= "<< pozyxOUTPUT[i] << endl; }
						}
					}			
				}
			
				if(preview == 1){
					cout << "POZYX Full Mean Yaw: " << POZYX_OUTPUT_YAW << " Roll:" << POZYX_OUTPUT_ROLL << " Pitch:" << POZYX_OUTPUT_PITCH << endl;
				}
				//cout << "POZYX Full Mean Roll " << POZYX_OUTPUT_ROLL << endl;
				//cout << "POZYX Full Mean Pitch " << POZYX_OUTPUT_PITCH << endl;
				if(preview == 1){
					cout << "POZYX Full Mean X:" << POZYX_OUTPUT_X << " Y:" << POZYX_OUTPUT_Y << " Z:" << POZYX_OUTPUT_Z << endl;
				}
				//cout << "POZYX Full Mean Y" << POZYX_OUTPUT_Y << endl;
				//cout << "POZYX Full Mean Z" << POZYX_OUTPUT_Z << endl;

				REAL_PREV_seconds_since_start = REAL_seconds_since_start; //Keep previous here, after using it
				REAL_PREV_POZYX_samples = pozyxOUTPUT.size();
			}



			//RealDist = measured distance, input to script
			//POZYX DISTANCE - ANGLES
			if(totalIterations % 30 == 0) {
				cout << "====================  " << endl;
				cout << "pozyxOUTPUT ===== " << endl;
				for (i=0; i< pozyxOUTPUT.size(); i++){
					if(i>= pozyxOUTPUT.size()-6){
						if(i== pozyxOUTPUT.size()-4){
							cout << "Pitch " << i << "= "<< pozyxOUTPUT[i] << endl;
						}else
						if(i== pozyxOUTPUT.size()-5){
							cout << "Roll " << i << "= "<< pozyxOUTPUT[i] << endl;
						}else
						if(i== pozyxOUTPUT.size()-6){
							cout << "Yaw " << i << "= "<< pozyxOUTPUT[i] << endl;
						}else{ 
							cout << "XYZ= " << i << "= "<< pozyxOUTPUT[i] << endl;
						 }
					}
				}		
				cout << "END pozyxOUTPUT ===== " << endl;
				cout << "====================  " << endl;
			}

			//MEASUREMENT FUSION over totalIterations
			//float varianceSquaredARUCO=0.000001f;
			//float varianceSquaredPOZYX=0.000001f;
			//float varianceSquaredLIDAR=0.000001f;
					

			float DistancePOZYX_relative_to_camera = -1; //when not measured put -1 to notify		
			float varianceSquaredPOZYX_final = -1;
		
			float relativeDistanceCameraPozyx_X = 250.0;	//310.0;  	//CHANGE HERE
			float relativeDistanceCameraPozyx_Y = 6250.0; 	//90.0;
			float relativeDistanceCameraPozyx_Z = 800.0;  	//1005.0; 	//457;//pozyxOUTPUT[pozyxOUTPUT.size()-1]; //520 - 63 = 457mm
			float DistPOZYXAxisToGround = 2500.0;		//1460.0f;
				
			if(pozyxOUTPUT.size() > 0){

				totalPozyxFoundIterations++;

				//DistancePOZYX_relative_to_camera = sqrt(pow((pozyxOUTPUT[pozyxOUTPUT.size()-3]-relativeDistanceCameraPozyx_X)/1000,2) 
				//+ pow((pozyxOUTPUT[pozyxOUTPUT.size()-2]-relativeDistanceCameraPozyx_Y)/1000,2) 
				//+ pow((pozyxOUTPUT[pozyxOUTPUT.size()-1]-relativeDistanceCameraPozyx_Z)/1000,2));
				DistancePOZYX_relative_to_camera = sqrt(
	 			  pow((POZYX_OUTPUT_X-relativeDistanceCameraPozyx_X)/1000.0,2) 
				+ pow((POZYX_OUTPUT_Y-relativeDistanceCameraPozyx_Y)/1000.0,2) 
				+ pow((POZYX_OUTPUT_Z-relativeDistanceCameraPozyx_Z)/1000.0,2));
			
				distanceMEAN_POZYX = distanceMEAN_POZYX + DistancePOZYX_relative_to_camera;
			
				if(preview == 1){
					cout << "distanceMEAN_POZYX ===== " << distanceMEAN_POZYX/(totalPozyxFoundIterations+1) << endl;
					cout << "DistancePOZYX_relative_to_camera ===== " << DistancePOZYX_relative_to_camera << endl;
				}
			
				varianceSquaredPOZYX = varianceSquaredPOZYX + pow(DistancePOZYX_relative_to_camera-(distanceMEAN_POZYX/(totalPozyxFoundIterations+1)),2); 
				varianceSquaredPOZYX_final = sqrt(varianceSquaredPOZYX / (totalPozyxFoundIterations+1));//desquared
			
			//	cout << "varianceSquaredPOZYX_final ===== " << varianceSquaredPOZYX_final << endl;

				float distanceHAT_FUSION = ((Center_to_Cam_dist/varianceSquaredARUCO_final) + (DistancePOZYX_relative_to_camera/varianceSquaredPOZYX_final)) /
				((1.0f/varianceSquaredARUCO_final) + (1.0f/varianceSquaredPOZYX_final));
			//	cout << "distanceHAT_FUSION ===== " << distanceHAT_FUSION << endl;

				float distanceHAT_FUSION_WITH_MEAN = (((Center_to_Cam_dist * distanceMEAN_ARUCO)/varianceSquaredARUCO_final) 
				+ ((DistancePOZYX_relative_to_camera * distanceMEAN_POZYX)/varianceSquaredPOZYX_final)) /
				((distanceMEAN_ARUCO/varianceSquaredARUCO_final) + (distanceMEAN_POZYX/varianceSquaredPOZYX_final));
			//	cout << "distanceHAT_FUSION_WITH_MEAN ===== " << distanceHAT_FUSION_WITH_MEAN << endl;

				//ROTATE ANGLES TO GET FINAL POZYX oRIENTATION BASED ON BASE POZYX AXIS

				//TRANSLATE to CAMERA (need to rotate too, not yet implemented !!!)			

				//forgetWindowIgterationsPOSYZ_mean++;
				if(totalPozyxFoundIterations % 3 == 0) {
					distanceMEAN_tX_POZYX = ((POZYX_OUTPUT_X)/1000.0);
					distanceMEAN_tY_POZYX = ((POZYX_OUTPUT_Y)/1000.0);
					distanceMEAN_tZ_POZYX = ((DistPOZYXAxisToGround-POZYX_OUTPUT_Z)/1000.0);
					forgetWindowIgterationsPOSYZ_mean = 1;
				}else{
					if(1==0 && totalPozyxFoundIterations % 1 < 1/1){ //start eliminating out of mean measurements
						if( ((POZYX_OUTPUT_X)/1000.0) - distanceMEAN_tX_POZYX > 0.1f){
							//If far from mean, use mean instead
							distanceMEAN_tX_POZYX = (distanceMEAN_tX_POZYX + distanceMEAN_tX_POZYX);
						}else{
							distanceMEAN_tX_POZYX = (distanceMEAN_tX_POZYX + (POZYX_OUTPUT_X)/1000.0);
						}
					
						if( ((POZYX_OUTPUT_Y)/1000.0) - distanceMEAN_tY_POZYX > 0.1f){
							//If far from mean, use mean instead
							distanceMEAN_tY_POZYX = (distanceMEAN_tY_POZYX + distanceMEAN_tY_POZYX);
						}else{
							distanceMEAN_tY_POZYX = (distanceMEAN_tY_POZYX + (POZYX_OUTPUT_Y)/1000.0);
						}

						if( ((DistPOZYXAxisToGround-POZYX_OUTPUT_Z)/1000.0) - distanceMEAN_tZ_POZYX > 0.1f){
							//If far from mean, use mean instead
							distanceMEAN_tZ_POZYX = (distanceMEAN_tZ_POZYX + distanceMEAN_tZ_POZYX);
						}else{
							distanceMEAN_tZ_POZYX = (distanceMEAN_tZ_POZYX + (DistPOZYXAxisToGround-POZYX_OUTPUT_Z)/1000.0);
						}					
					}else{
						distanceMEAN_tX_POZYX = (distanceMEAN_tX_POZYX + (POZYX_OUTPUT_X)/1000.0);
						distanceMEAN_tY_POZYX = (distanceMEAN_tY_POZYX + (POZYX_OUTPUT_Y)/1000.0);
						distanceMEAN_tZ_POZYX = (distanceMEAN_tZ_POZYX + (DistPOZYXAxisToGround-POZYX_OUTPUT_Z)/1000.0);
						//forgetWindowIgterationsPOSYZ_mean++;
					}
					forgetWindowIgterationsPOSYZ_mean++;
				}

				//TRANSFORMATIONS OF AXIS FOR POZYX TO CAMERA REPRESENTATION
				float transX = distanceMEAN_tX_POZYX / (forgetWindowIgterationsPOSYZ_mean);
				float transY = distanceMEAN_tY_POZYX / (forgetWindowIgterationsPOSYZ_mean);
				float transZ = distanceMEAN_tZ_POZYX / (forgetWindowIgterationsPOSYZ_mean);		
			
				Mat translateMatrixPOZYX = (Mat_<double>(4,4) << 1,0,0,transX, 0,1,0,transY, 0,0,1,transZ, 0,0,0,1); 
				//cout << "translateMatrixPOZYX = " << translateMatrixPOZYX << endl; 
				float pitch = POZYX_OUTPUT_PITCH* PI / 180.0; //on local POZYX TAG X axis, take inverse sign ?
				float roll = POZYX_OUTPUT_ROLL* PI / 180.0; //on local POZYX TAG Y axis			
				float yaw = POZYX_OUTPUT_YAW* PI / 180.0; //on local POZYX TAG Z axis
				//start rotation in Z due to difference in yaw of base axis and magnetic north direction (cos,-sin,0,  sin,cos,0,  0,0,1)
			
				Mat rotMatrixPOZYX_Z1 = rotation_matrix_Z_rad((-40.0 * PI / 180.0)+yaw);

				int sign = -1; //FIX ISSUE WHERE RIGHT SIDE ROLL FLIPS X - Y AXIS (Y is backwards)
				if(abs(POZYX_OUTPUT_PITCH) > 90){
					sign = 1;			
				}
				Mat rotMatrixPOZYX_Z90 = rotation_matrix_Z_rad((sign*90.0* PI / 180.0));			

				Mat rotMatrixPOZYX_Y = rotation_matrix_Y_rad(roll);
				Mat rotMatrixPOZYX_X = rotation_matrix_X_rad(pitch);
				Mat rotMatrixPOZYX_Z2 = rotation_matrix_Z_rad(yaw);			
				Mat finalPozyxPose = translateMatrixPOZYX * rotMatrixPOZYX_Z90 * rotMatrixPOZYX_Z1  * rotMatrixPOZYX_Y  * rotMatrixPOZYX_X;

				//TEST MATRICES
				//Mat test1 = (Mat_<double>(4,4) << cos((-40.0* PI / 180.0)+yaw),-sin((-40.0* PI / 180.0)+yaw),0,0, 
				//sin((-40.0* PI / 180.0)+yaw),cos((-40.0* PI / 180.0)+yaw),0,0,0,0,1,0, 0,0,0,1);
				//Mat test11 = rotMatrixPOZYX_Y;
				//Mat test111 = rotMatrixPOZYX_X;

				//cout << "rotMatrixPOZYX_Z1 * rotMatrixPOZYX_Z2 = " << test1 << endl; 
				//cout << "yaw = " << yaw << endl; 
				//cout << "rotMatrixPOZYX_Y = " << test11 << endl; 
				//cout << "rotMatrixPOZYX_X = " << test111 << endl; 

				//cout << "finalPozyxPose = " << finalPozyxPose << endl; 			
				Vec3f toEulerPozyx = rotationMatrixToEulerAngles(finalPozyxPose)*(180.0f/PI);
				//cout << "finalPozyxPose toEulerPozyx = " << toEulerPozyx << endl; 

				//FINAL POSE WITH RESPECT TO CAMERA
				Mat translateMatrixPOZYX1 = (Mat_<double>(4,4) << 
				1,0,0,-relativeDistanceCameraPozyx_Y/1000.0, 
				0,1,0,-(DistPOZYXAxisToGround-relativeDistanceCameraPozyx_Z)/1000.0, 
				0,0,1,-relativeDistanceCameraPozyx_X/1000.0, 
				0,0,0,1);

				Mat translateMatrixPOZYX2 = (Mat_<double>(4,4) << 
				1,0,0,relativeDistanceCameraPozyx_X/1000.0, 
				0,1,0,relativeDistanceCameraPozyx_Y/1000.0, 
				0,0,1,(DistPOZYXAxisToGround-relativeDistanceCameraPozyx_Z)/1000.0, 
				0,0,0,1);

				Mat translateMatrixPOZYX11 = (Mat_<double>(4,4) << 
				1,0,0,relativeDistanceCameraPozyx_Y/1000.0, 
				0,1,0,(DistPOZYXAxisToGround-relativeDistanceCameraPozyx_Z)/1000.0, 
				0,0,1,relativeDistanceCameraPozyx_X/1000.0, 
				0,0,0,1);

				float angleROT = -90.0 * PI / 180.0;		

				Mat rotMatrixPOZYX_Y1 = rotation_matrix_Y_rad(angleROT);
				Mat rotMatrixPOZYX_X1 = rotation_matrix_X_rad(angleROT);
				Mat rotMatrixPOZYX_Z3 = rotation_matrix_Z_rad(angleROT);

				Mat finalPozyxPoseCAM = translateMatrixPOZYX1 * rotMatrixPOZYX_Y1 * rotMatrixPOZYX_X1 * finalPozyxPose;
			
				//cout << "finalPozyxPoseCAM = " << finalPozyxPoseCAM << endl; 
				Mat test4 = translateMatrixPOZYX2 * rotMatrixPOZYX_Y1 * rotMatrixPOZYX_Z3;
				//cout << "test4 = " << test4 << endl; 

				//PLOT POZYX AXIS
				vector< Point2f > imagePoints_POZYX;

				//Vec3d rvecsBaseC_POZYX, tvecsBaseC_POZYX;
				//Mat transResultC_POZYX = (Mat_<double>(1,3) << 0,0,0);
				//Mat rotResultC_POZYX = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
			
				transResultC_POZYX.at<double>(0,0) = finalPozyxPoseCAM.at<double>(0,3);
				transResultC_POZYX.at<double>(0,1) = finalPozyxPoseCAM.at<double>(1,3);
				transResultC_POZYX.at<double>(0,2) = finalPozyxPoseCAM.at<double>(2,3);//-(2.0*relativeDistanceCameraPozyx_X/1000.0) ;
					
				tvecsBaseC_POZYX = transResultC_POZYX;
				//cout << "transResultC_POZYX = " << transResultC_POZYX << endl; 
				//ROTATIONS
				rotResultC_POZYX.at<double>(0,0) = finalPozyxPoseCAM.at<double>(0,0);
				rotResultC_POZYX.at<double>(0,1) = finalPozyxPoseCAM.at<double>(0,1);
				rotResultC_POZYX.at<double>(0,2) = finalPozyxPoseCAM.at<double>(0,2);
				rotResultC_POZYX.at<double>(1,0) = finalPozyxPoseCAM.at<double>(1,0);
				rotResultC_POZYX.at<double>(1,1) = finalPozyxPoseCAM.at<double>(1,1);
				rotResultC_POZYX.at<double>(1,2) = finalPozyxPoseCAM.at<double>(1,2);
				rotResultC_POZYX.at<double>(2,0) = finalPozyxPoseCAM.at<double>(2,0);
				rotResultC_POZYX.at<double>(2,1) = finalPozyxPoseCAM.at<double>(2,1);
				rotResultC_POZYX.at<double>(2,2) = finalPozyxPoseCAM.at<double>(2,2);
				//cout << "rotResultC_POZYX = " << rotResultC_POZYX << endl; 

				cv::Rodrigues(rotResultC_POZYX,rvecsBaseC_POZYX);//redo passing to final rotations vector				

				//PROJECT SOLID CENTER AXIS
				Mat imagePointsVelocities1 = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //
				// project axis points pozyx
				vector< Point3f > axisPointsPOZYX;
				axisPointsPOZYX.push_back(Point3f(0, 0, 0));
				axisPointsPOZYX.push_back(Point3f(markerLength * 5.5f, 0, 0));
				axisPointsPOZYX.push_back(Point3f(0, markerLength * 5.5f, 0));
				axisPointsPOZYX.push_back(Point3f(0, 0, markerLength * 5.5f));
				projectPoints(axisPointsPOZYX, rvecsBaseC_POZYX, tvecsBaseC_POZYX, camMatrix, distCoeffs, imagePoints_POZYX, imagePointsVelocities1);

				//plot base axis
				if(preview == 1){				
					// draw axis lines
					if(imagePoints_POZYX[0].x>0 && imagePoints_POZYX[0].x < 1280 && imagePoints_POZYX[0].y>0 && imagePoints_POZYX[0].y < 960){
						line(imageCopy, imagePoints_POZYX[0], imagePoints_POZYX[1], Scalar(0, 0, 155), 3);
						line(imageCopy, imagePoints_POZYX[0], imagePoints_POZYX[2], Scalar(0, 155, 0), 3);
						line(imageCopy, imagePoints_POZYX[0], imagePoints_POZYX[3], Scalar(155, 0, 0), 3);	
					}			
				}
				//END PLOT POZYX AXIS
			}
			//POZYX DISTANCE - ANGLES - RESET BUFFER CREATED IN SEPARATE THREAD
			if(outWindowID == 2 && pozyxOUTPUT.size() > 1000){
				pozyxOUTPUT.clear(); //Clear only in 2ond brightness !!!  //CHANGE HERE
			}


			
			



			//CREATE SAVE FILE IF POZYX IS ACTIVATED			
			if (centerPointsIDs.size() > 0) {
				//outfile << to_string(totalTime) << ";" << outWindowID << ";" << counted << ";" << identifiedRectangleCount << ";"  << identifiedTriangleCount 
				outfile << to_string(REAL_seconds_since_start) << ";" << outWindowID << ";" << counted << ";" 
				<< identifiedRectangleCount << ";"  << identifiedTriangleCount 
				//<< ";" << errorID
				<< ";" << DistancePOZYX_relative_to_camera			
				<< ";" << Center_to_Cam_dist << ";" 
				<< tvecsBaseC(0) << ";" << tvecsBaseC(1) << ";" << tvecsBaseC(2) << ";" 
				<< toEuler(0) << ";" << toEuler(1) << ";" << toEuler(2) << ";" 
				<< rvecsBaseC(0) << ";" << rvecsBaseC(1) << ";" << rvecsBaseC(2) << ";"
				<< rotResultC.at<double>(0,0) << ";" << rotResultC.at<double>(0,1) << ";" << rotResultC.at<double>(0,2) << ";"
				<< rotResultC.at<double>(1,0) << ";" << rotResultC.at<double>(1,1) << ";" << rotResultC.at<double>(1,2) << ";"
				<< rotResultC.at<double>(2,0) << ";" << rotResultC.at<double>(2,1) << ";" << rotResultC.at<double>(2,2) << ";"
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-3]-relativeDistanceCameraPozyx_X)/1000 << ";" 
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-2]-relativeDistanceCameraPozyx_Y)/1000 << ";" 
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-1]-relativeDistanceCameraPozyx_Z)/1000 << ";" 
				<< POZYX_OUTPUT_X/1000 << ";" 
				<< POZYX_OUTPUT_Y/1000 << ";" 
				<< POZYX_OUTPUT_Z/1000 << ";" 
				<< POZYX_OUTPUT_YAW << ";" << POZYX_OUTPUT_ROLL << ";" << POZYX_OUTPUT_PITCH << ";" 
				<< tvecsBaseC_POZYX(0) << ";" << tvecsBaseC_POZYX(1) << ";" << tvecsBaseC_POZYX(2) << ";" 
				<< rvecsBaseC_POZYX(0) << ";" << rvecsBaseC_POZYX(1) << ";" << rvecsBaseC_POZYX(2) << ";"
				<< rotResultC_POZYX.at<double>(0,0) << ";" << rotResultC_POZYX.at<double>(0,1) << ";" << rotResultC_POZYX.at<double>(0,2) << ";"
				<< rotResultC_POZYX.at<double>(1,0) << ";" << rotResultC_POZYX.at<double>(1,1) << ";" << rotResultC_POZYX.at<double>(1,2) << ";"
				<< rotResultC_POZYX.at<double>(2,0) << ";" << rotResultC_POZYX.at<double>(2,1) << ";" << rotResultC_POZYX.at<double>(2,2) << ";"
				<< distanceMEAN_ARUCO << ";" << varianceSquaredARUCO_final << ";" << distanceMEAN_POZYX << ";" << varianceSquaredPOZYX_final << ";" 
				<< inliers << outliers
				<< std::endl;

				if(preview == 1){
					cout << "Outliers = " << outliers << " Inliers = " << inliers << "Window ID = " << outWindowID << endl;
				}

				if(counted > 0){
					totalArucoMarkersFoundIterations++;
				}
			}
			//END CREATE SAVE FILE

		}//END if(usePOZYX==1){
		


		
		//LIDAR
		if(useLIDAR==1){
			//PROJECT SOLID CENTER AXIS
			Mat imagePointsVelocities2 = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //
			// project axis points pozyx		
		
			cout << "LIDAR points XYZ outMsg->points.size() = " << outMsg->points.size()<< endl;
			for (size_t i = 0; i < outMsg->points.size(); i=i+2)
			{
				RGBPoint p;
				p.x = outMsg->points[i].x;
				p.y = outMsg->points[i].y;
				p.z = outMsg->points[i].z;
				//axisPointsLIDAR.push_back(Point3f(p.x, p.y, p.z));
				//Vec3d rvecsBaseC_LIDAR, tvecsBaseC_LIDAR;
				//Mat tvecsBaseC_LIDAR = (Mat_<double>(1,3) << 0+i*0.1,0,3); //p.x,p.y,p.z);

				//http://velodynelidar.com/docs/manuals/VLP-16%20User%20Manual%20and%20Programming%20Guide%2063-9243%20Rev%20A.pdf

				//Mat tvecsBaseC_LIDAR = (Mat_<double>(1,3) << p.x,p.z,p.y);
				Mat tvecsBaseC_LIDAR = (Mat_<double>(1,3) << p.x,p.z,p.y) * 1;

				Mat rvecsBaseC_LIDAR = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);

				vector< Point3f > axisPointsLIDAR;
				axisPointsLIDAR.push_back(Point3f(0, 0, 0));
				axisPointsLIDAR.push_back(Point3f(markerLength * 0.005f, 0, 0));
				axisPointsLIDAR.push_back(Point3f(0, markerLength * 0.005f, 0));
				axisPointsLIDAR.push_back(Point3f(0, 0, markerLength * 0.005f));
			
				//PLOT POZYX AXIS
				vector< Point2f > imagePoints_LIDAR;
				projectPoints(axisPointsLIDAR, rvecsBaseC_LIDAR, tvecsBaseC_LIDAR, camMatrix, distCoeffs, imagePoints_LIDAR, imagePointsVelocities2);
				//cout << "points XYZ = " << p.x << "," << p.y << "," << p.z << endl;
				float dist = sqrt(pow(tvecsBaseC_LIDAR.at<double>(0,0),2) 
						+ pow(tvecsBaseC_LIDAR.at<double>(0,1),2) 
						+ pow(tvecsBaseC_LIDAR.at<double>(0,2),2));
				if(preview == 1 && dist > 1 && dist < 2){// && dist < 12.6){// && p.y < 0){				
					// draw axis lines					
					if(imagePoints_LIDAR[0].x>0 && imagePoints_LIDAR[0].x < 1280 && imagePoints_LIDAR[0].y>0 && imagePoints_LIDAR[0].y < 960){
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[1], Scalar(0, 155, 55), 3);
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[2], Scalar(155, 55, 0), 3);
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[3], Scalar(55, 0, 155), 3);	
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[1], Scalar(p.r, p.g, p.b)*255, 3);
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[2], Scalar(p.r, p.g, p.b)*255, 3);
						//line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[3], Scalar(p.r, p.g, p.b)*255, 3);
						line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[1], Scalar(0,255 - 48*dist,  0)*2, 3);
						line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[2], Scalar(0,255 - 48*dist,  0)*2, 3);
						line(imageCopy, imagePoints_LIDAR[0], imagePoints_LIDAR[3], Scalar(0,255 - 48*dist,  0)*2, 3);
					}							
				}
			}
		}
		//END LIDAR



		//CREATE SAVE FILE IF POZYX IS NOT ACTIVATED	
		if(usePOZYX==0){
			if (centerPointsIDs.size() > 0) {
				//outfile << to_string(totalTime) << ";" << outWindowID << ";" << counted << ";" << identifiedRectangleCount << ";"  << identifiedTriangleCount 
				outfile << to_string(REAL_seconds_since_start) << ";" << outWindowID << ";" << counted << ";" 
				<< identifiedRectangleCount << ";"  << identifiedTriangleCount 
				//<< ";" << errorID
				//<< ";" << DistancePOZYX_relative_to_camera			
				<< ";" << Center_to_Cam_dist << ";" 
				<< tvecsBaseC(0) << ";" << tvecsBaseC(1) << ";" << tvecsBaseC(2) << ";" 
				<< toEuler(0) << ";" << toEuler(1) << ";" << toEuler(2) << ";" 
				<< rvecsBaseC(0) << ";" << rvecsBaseC(1) << ";" << rvecsBaseC(2) << ";"
				<< rotResultC.at<double>(0,0) << ";" << rotResultC.at<double>(0,1) << ";" << rotResultC.at<double>(0,2) << ";"
				<< rotResultC.at<double>(1,0) << ";" << rotResultC.at<double>(1,1) << ";" << rotResultC.at<double>(1,2) << ";"
				<< rotResultC.at<double>(2,0) << ";" << rotResultC.at<double>(2,1) << ";" << rotResultC.at<double>(2,2) << ";"
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-3]-relativeDistanceCameraPozyx_X)/1000 << ";" 
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-2]-relativeDistanceCameraPozyx_Y)/1000 << ";" 
				//<< (pozyxOUTPUT[pozyxOUTPUT.size()-1]-relativeDistanceCameraPozyx_Z)/1000 << ";" 				
				<< distanceMEAN_ARUCO << ";" << varianceSquaredARUCO_final << ";" 
				<< inliers << outliers
				<< std::endl;

				if(preview == 1){
					cout << "Outliers = " << outliers << " Inliers = " << inliers << "Window ID = " << outWindowID << endl;
				}

				if(counted > 0){
					totalArucoMarkersFoundIterations++;
				}
			}
		}
	}//END id's > 0 CHECK
	else{
		idsCountPrevFrame = 0;
	}

	double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
	totalTime += currentTime;	

	totalIterations++;
	if(totalIterations % 30 == 0) {
	    cout << "Detection Time = " << currentTime * 1000 << " ms "
	         << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << "(TotalTime = " << 1000 * totalTime << " ms)"<< endl;
	}



		//TIMER ACCURATE
		gettimeofday(&endT, NULL);						
		double elapsed1 = (endT.tv_sec - beginT.tv_sec) + ((endT.tv_usec - beginT.tv_usec)/1000000.0);
		beginT = endT;		
		REAL_seconds_since_start = REAL_seconds_since_start + elapsed1;

		if(totalIterations % 30 == 0) {
		    cout //<< "Detection Time Accurate = " << elapsed1 * 1000 << " ms "
			 << "(Mean = " << 1000 * REAL_seconds_since_start / double(totalIterations) << " ms)" 
			 << "(Aruco Output Rate =" << double(totalIterations) / REAL_seconds_since_start
			 << "(TotalTime = " << 1000 * REAL_seconds_since_start << " ms)"<< " Window ID=" << outWindowID 
			 << " totalIterations=" << totalIterations  << endl;
		}		



	if (imageCopy.empty()){
		return 0;
	}	
 	
	if(preview == 1){
		imshow("out"+to_string(outWindowID), imageCopy);
		//Mat imageUndistorted;
		//undistort(image, imageUndistorted,camMatrix, distCoeffs);
		//imshow("outUndistorted", imageUndistorted);
	}
	//END OCAM - ARUCO
		
    return true;
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
cv::Mat srcImg;
cv::Mat prevSrcImg;
int same_images_count=0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

 

 try
  {
	
	//TIMER 4
	gettimeofday(&beginT4, NULL);

	//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	cv::waitKey(5);

	//ARUCO START	
	int ExposureAbsolute = 72;
	srcImg = cv_bridge::toCvShare(msg, "bgr8")->image;

	//TIMER 4
	gettimeofday(&endT4, NULL);	
	double elapsed2 = (endT4.tv_sec - beginT4.tv_sec) + ((endT4.tv_usec - beginT4.tv_usec)/1000000.0);	
	//beginT4 = endT4;
	if(totalIterations % 30 == 0) {	
		cout << "(Grab Camera Frame Time = " << 1000 * elapsed2 << " ms)" << endl;
	}	

	//TIMER 3
	gettimeofday(&beginT3, NULL);

	//imshow("out", srcImg);
	arucoProcess(srcImg,ExposureAbsolute, 1);	

	//TIMER 3
	gettimeofday(&endT3, NULL);						
	double elapsed1 = (endT3.tv_sec - beginT3.tv_sec) + ((endT3.tv_usec - beginT3.tv_usec)/1000000.0);	
	//beginT3 = endT3;
	if(totalIterations % 30 == 0) {
		cout << "(Aruco Calculations Time = " << 1000 * elapsed1 << " ms)" << endl;	
	}	


	if(1==0){ //A
		
		cv::Mat srcImgBright = Mat::zeros( srcImg.size(), srcImg.type() );

		//BRIGHTNESS & CONTRAST
		/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
		for( int y = 0; y < srcImg.rows; y++ ){ 
			for( int x = 0; x < srcImg.cols; x++ ){ 
				for( int c = 0; c < 3; c++ ){
		      			//srcImgBright.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*( srcImg.at<Vec3b>(y,x)[c] ) + beta);
					srcImgBright.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*( srcImg.at<Vec3b>(y,x)[c] ) + beta );
					srcImgBright.at<Vec3b>(y,x)[c] = srcImgBright.at<Vec3b>(y,x)[c] 
					- srcImg.at<Vec3b>(y,x)[c]*(10/(pow(1-(alpha*srcImgBright.at<Vec3b>(y,x)[c]),2))  );
			     	}
		    	}
		}

		//MERGE IMAGES HDR STYLE
		if(1==0){// B
			vector<Mat> images;
		    	vector<float> times;
			images.push_back(srcImg);
		 	images.push_back(srcImgBright);
			times.push_back(1.0f / 60.0f);//need 1/exposure time
			times.push_back(1.0f / 220.0f);
		    	//loadExposureSeq(argv[1], images, times);
		    	Mat response;
		    	Ptr<CalibrateDebevec> calibrate = createCalibrateDebevec();
		    	calibrate->process(images, response, times);
		   	Mat hdr;
		    	Ptr<MergeDebevec> merge_debevec = createMergeDebevec();
		    	merge_debevec->process(images, hdr, times, response);
		    	Mat ldr;
		    	Ptr<TonemapDurand> tonemap = createTonemapDurand(2.2f);
		    	tonemap->process(hdr, ldr);
		    	Mat fusion;
		    	Ptr<MergeMertens> merge_mertens = createMergeMertens();
		    	merge_mertens->process(images, fusion);
		    	//imwrite("fusion.png", fusion * 255);
		}

		arucoProcess(srcImgBright,ExposureAbsolute, 2);
		//ARUCO END
	
	}//END if 1==0 A



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
	if(totalIterationsTimer == 30) {
						
		//TIMER ACCURATE
		gettimeofday(&endT, NULL);						
		double elapsed = (endT.tv_sec - beginT.tv_sec) + 
              	((endT.tv_usec - beginT.tv_usec)/1000000.0);
		beginT=endT;
		cout << "Precision Time taken : " << elapsed << " seconds" << " with iterations:" << totalIterationsTimer << endl;  
		double fps2  = totalIterationsTimer / elapsed;
		cout << "Estimated Precision frames per second : " << fps2 << endl;						

		//CHECK IF IMAGE SAME AS PREVIOUS //CHECK IMAGE EQUALITY		
		if(same_images_count >0){
			cout << same_images_count << " SAME IMAGES FOUND" << endl;
			double fps3  = (totalIterationsTimer-same_images_count) / elapsed;
			cout << "Estimated REAL frames per second : " << fps3 << endl;
		}		
		same_images_count=0;						
		totalIterationsTimer = 0;
	}
     }//END FPS CALCS


	




  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


//sample counter
int POZYXsamplesCounted = 0;

// POZYX THREAD
void task1(string msg)
{	

	//TIMER2 - SET START
	if(totalIterations == 0) {
		gettimeofday(&beginT2, NULL);
	}


	while(2>1){

	

	//pozyxOUTPUT.clear();
	//ARDUINO COM READ
		/* Send byte to trigger Arduino to send string back */
		//write(fd, "0", 1);
		/* Receive string from Arduino */
		n = read(fd, buf, 256);
		/* insert terminating zero in the string */
		buf[n] = 0;
		//SPLIT NUMBERS
		std::vector<int> vectMEASUREMENTS;
		std::stringstream ss(buf);
		int icount;
		while (ss >> icount)
		{
			vectMEASUREMENTS.push_back(icount);
			if (ss.peek() == ','){
				ss.ignore();
			}
		}

		int pos1 = 25;
		int pos2 = 26;
		int pos3 = 27;
		int rot1 = 11;
		int rot2 = 12;
		int rot3 = 13;

		//afetr adding ID in column 0
		pos1 = 26;
		pos2 = 27;
		pos3 = 28;
		rot1 = 12;
		rot2 = 13;
		rot3 = 14;

		for (i=0; i< vectMEASUREMENTS.size(); i++){
			if(i == rot1 || i==rot2 || i==rot3){
				//float angle = ((int)(vectMEASUREMENTS.at(i))/16) % 360; 
				float angle = ((int)(vectMEASUREMENTS.at(i))/16) % 360; 
				
				//angle = vectMEASUREMENTS.at(i); 
				if(i == rot1){
					//angle = angle - 215;
					if(abs(angle) > 180){
						//angle = 360 - abs(angle);					
					}
				}
				if(i == rot2){
					angle = -angle;

					float angleNext = ((int)(vectMEASUREMENTS.at(i+1))/16) % 360; 
					if(abs(angleNext) > 170){
						//angle = (angle/abs(angle)) * (90 - abs(angle) + 90);
					}
				}
				//std::cout << angle <<std::endl;
				pozyxOUTPUT.push_back(angle);
			}
			//print POZYX corrds
			if(i == pos1 || i==pos2 || i==pos3){
				//std::cout << vectMEASUREMENTS.at(i) <<std::endl;
				pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			}


			//show ID
			if(i == 0){
				//std::cout << vectMEASUREMENTS.at(i) <<std::endl;
			}
		}
		for (i=0; i< vectMEASUREMENTS.size(); i++){
			if(i == rot1 || i==rot2 || i==rot3){				
				//float angle = vectMEASUREMENTS.at(i); 
				//std::cout << angle <<std::endl;
			}
		}
		//printf("%i bytes read, buffer contains: %s\n", n, buf);
		//cout << "buf = " << buf << endl;			
		//END ARDUINO COM READ	

		if(vectMEASUREMENTS.size() > 0){
			POZYXsamplesCounted++;
		}
		//cout << "POZYXsamplesCounted= " << POZYXsamplesCounted << endl;
		//cout << "(pozyxOUTPUT.size() = " << pozyxOUTPUT.size() << endl;
		//if(pozyxOUTPUT.size() == 120) {//if(totalIterations % 30 == 0) {
		if(POZYXsamplesCounted == 20) {//if(pozyxOUTPUT.size() % 30 == 0) {
				gettimeofday(&endT2, NULL);						
				double elapsed1 = (endT2.tv_sec - beginT2.tv_sec) + ((endT2.tv_usec - beginT2.tv_usec)/1000000.0);
				beginT2 = endT2;		
				REAL_seconds_since_start2 = REAL_seconds_since_start2 + elapsed1;
				//
				cout //<< "Detection Time Accurate = " << elapsed1 * 1000 << " ms "
					 << "(Mean2 = " << 1000 * elapsed1 / POZYXsamplesCounted << " ms)" 
					 << "(POZYX Output Rate =" << double(POZYXsamplesCounted) / elapsed1 << ")"
					 << "(TotalTime2 = " << 1000 * elapsed1 << " ms)" 
					 << " total counted =" << POZYXsamplesCounted << endl;
				POZYXsamplesCounted=0;
		}

	}
    	//cout << "task1 says: " << msg;
}




//LIDAR
void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  	BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  		printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

  /** @brief Callback for Velodyne PointXYZRI messages. */
 ros::Subscriber input_;
ros::Publisher output_;
 
//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 // void convertPoints(const VPointCloud::ConstPtr &inMsg)
void convertPoints(const VPointCloud::ConstPtr& inMsg)
{

	cout << "points XYZ = "  << endl;

	//if (output_.getNumSubscribers() == 0)         // no one listening?
	//  return;                                     // do nothing

	// allocate an PointXYZRGB message with same time and frame ID as
	// input data
	//outMsg.clear();
	outMsg->points.clear();
	outMsg->header.stamp = inMsg->header.stamp;
	outMsg->header.frame_id = inMsg->header.frame_id;
	outMsg->height = 1;

	for (size_t i = 0; i < inMsg->points.size(); ++i)
	{
		RGBPoint p;
		p.x = inMsg->points[i].x;
		p.y = inMsg->points[i].y;
		p.z = inMsg->points[i].z;

		if(totalIterations % 30 == 0) {
			cout << "points XYZ = " << p.x << "," << p.y << "," << p.z << endl;
		}

		// color lasers with the rainbow array
		int color = inMsg->points[i].ring % N_COLORS;
		p.rgb = *reinterpret_cast<float*>(rainbow+color);

		outMsg->points.push_back(p);
		++outMsg->width;
	}
	/*
	    for (size_t i = 0; i < outMsg->points.size(); ++i)
	    {
		RGBPoint p;
		p.x = outMsg->points[i].x;
		p.y = outMsg->points[i].y;
		p.z = outMsg->points[i].z;
		cout << "points XYZ = " << p.x << "," << p.y << "," << p.z << endl;
	    }
	*/




	//LiDAR PREVIEW
	
	//FIND NORMALS
	int displayNormals = 1;
	if(displayNormals = 1){
			//#include <pcl/features/normal_3d.h>
			//http://pointclouds.org/documentation/tutorials/normal_estimation.php
			// Create the normal estimation class, and pass the input dataset to it
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			ne.setInputCloud (outMsg);

			// Create an empty kdtree representation, and pass it to the normal estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
			ne.setSearchMethod (tree);

			// Output datasets
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

			// Use all neighbors in a sphere of radius 3cm
			ne.setRadiusSearch (0.03 * 100); //*0.01875

			// Compute the features
			ne.compute (*cloud_normals);			
			//END FIND NORMALS

			viewer.addCoordinateSystem(0.05);			
			viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (outMsg, cloud_normals, 1, 0.15f,"scene_cloud1");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "scene_cloud1");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.003, "scene_cloud1");

			viewer.addPointCloud (outMsg, "scene_cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scene_cloud");
						
			viewer.spinOnce ();
			viewer.removePointCloud("scene_cloud");
			viewer.removePointCloud("scene_cloud1");
	}else{
			viewer.addPointCloud (outMsg, "scene_cloud");
			viewer.spinOnce ();
			viewer.removePointCloud("scene_cloud");
	}

    	//output_.publish(outMsg);
}

bool solidTrained = false;
//END LIDAR




//MAIN
int main(int argc, char **argv)
{ 

	//BRIGHTNESS & CONTRAST
	//std::cout<<"* Enter the alpha value [1.0-3.0]: ";std::cin>>alpha;
 	//std::cout<<"* Enter the beta value [0-100]: "; std::cin>>beta;

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
	std::string fileName = "timeH"+to_string(HorAngle)+"V"+to_string(VerAngle)+"D"+to_string(RealDist)+".csv"; 
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
		throw std::ios_base::failure(std::strerror(errno));
	}
	//make sure write fails with exception if something is wrong
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);
	//outfile.open(fileName, std::ios_base::app);
	//outfile << fileName << std::endl;
	//outfile << "Data"; 

	if(usePOZYX == 1){
		outfile << "time" << ";" << "windowID" << ";" << "Counted IDs" << ";" 
			<< "Counted Rectangles" << ";"  << "Counted Triangles" 			
			<< ";" << "Distance POZYX relative to camera"			
			<< ";" << "Solid Center to Camera distance" << ";" 
			<< "Translation Vector X (Solid to Camera)" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Euler X" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Rodrigues X" << ";" << "Y" << ";" << "Z" << ";"
			<< "Solid Camera RotMatrix (00)" << ";" << "(01)" << ";" << "(02)" << ";"
			<< "(10)" << ";" << "(11)" << ";" << "(12)" << ";"
			<< "(20)" << ";" << "(21)" << ";" << "(22)" << ";"		
			<< "Raw (Base) POZYX X (m)" << ";" 
			<< "Y (m)" << ";" 
			<< "Z (m)" << ";" 
			<< "Raw POZYX YAW" << ";" << "Raw POZYX ROLL" << ";" << "Raw POZYX PITCH" << ";" 
			<< "Translation Vector X (POZYX to Camera)" << ";" << "Y" << ";" << "Z" << ";" 
			<< "POZYX Camera Rodrigues X" << ";" << "Y" << ";" << "Z" << ";"
			<< "POZYX Camera RotMatrix (00)" << ";" << "(01)" << ";" << "(02)" << ";"
			<< "(10)" << ";" << "(11)" << ";" << "(12)" << ";"
			<< "(20)" << ";" << "(21)" << ";" << "(22)" << ";"
			<< "distance MEAN ARUCO" << ";" << "variance Squared ARUCO" << ";" << "distance MEAN POZYX" << ";" << "variance Squared POZYX" << ";" 
			<< "Inlier - Outlier IDs"
			<< std::endl;
	}
	if(usePOZYX == 0){
		outfile << "time" << ";" << "windowID" << ";" << "Counted IDs" << ";" 
			<< "Counted Rectangles" << ";"  << "Counted Triangles"				
			<< ";" << "Solid Center to Camera distance" << ";" 
			<< "Translation Vector X (Solid to Camera)" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Euler X" << ";" << "Y" << ";" << "Z" << ";" 
			<< "Solid Camera Rodrigues X" << ";" << "Y" << ";" << "Z" << ";"
			<< "Solid Camera RotMatrix (00)" << ";" << "(01)" << ";" << "(02)" << ";"
			<< "(10)" << ";" << "(11)" << ";" << "(12)" << ";"
			<< "(20)" << ";" << "(21)" << ";" << "(22)" << ";"			
			<< "distance MEAN ARUCO" << ";" << "variance Squared ARUCO" << ";"
			<< "Inlier - Outlier IDs"
			<< std::endl;
	}

	if(preview == 1){
		cv::namedWindow("view");
		cv::startWindowThread();
	}
  	image_transport::ImageTransport it(nh);

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
	//detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; //do marker corner refinement 
	//OCAM not supported in ROS openCV ?

	//detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; 
	//do marker corner refinement -- https://github.com/opencv/opencv_contrib/tree/master/modules/aruco/samples
	//detectorParams->doCornerRefinement = true; //do marker corner refinement 
	//detectorParams->doCornerRefinement = 
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

	if(usePOZYX==1){
		//ARDUINO COM READ
		//int fd, n, i;
		//char buf[64] = "temp text";
		struct termios toptions;

		/* open serial port */
		fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
		printf("fd opened as %i\n", fd);
		  
		/* wait for the Arduino to reboot */
		usleep(3500000);
		  
		/* get current serial port settings */
		tcgetattr(fd, &toptions);
		/* set 9600 baud both ways */
		//cfsetispeed(&toptions, B115200);
		//cfsetospeed(&toptions, B115200);
		cfsetispeed(&toptions, B500000);
		cfsetospeed(&toptions, B500000);
		/* 8 bits, no parity, no stop bits */
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;
		/* Canonical mode */
		toptions.c_lflag |= ICANON;
		/* commit the serial port settings */
		tcsetattr(fd, TCSANOW, &toptions);
		//END ARDUINO COM READ
	}


	//TIMER
	gettimeofday(&beginT, NULL);
	gettimeofday(&beginT3, NULL);
	gettimeofday(&beginT4, NULL);



	//LiDAR
	if(useLIDAR==1){
		/*
		PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
		//points_msg->header = "header"; // TODO fill in header
		points_msg->height = mat.rows; // if this is a "full 3D" pointcloud, height is 1; if this is Kinect-like pointcloud, height is the vertical resolution
		points_msg->width  = mat.cols;
		points_msg->is_bigendian = false;
		points_msg->is_dense = false; // there may be invalid points

		sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
		// this call also resizes the data structure according to the given width, height and fields
		pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

		sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

		for (...) // TODO get/generate point coordinates and colors
		{
		  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
		  {
		    // TODO fill in x, y, z, r, g, b local variables
		    *iter_x = x;
		    *iter_y = y;
		    *iter_z = z;
		    *iter_r = r;
		    *iter_g = g;
		    *iter_b = b;
		  }
		}
			//END LiDAR
			ros::init(argc, argv, "sub_pcl");
			ros::NodeHandle nh;
			ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
			ros::spin();
		*/

		// subscribe to VelodyneScan packets
	    	input_ = nh.subscribe("velodyne_points", 10, convertPoints);
		//             &convertPoints);
		//this,
		//ros::TransportHints().tcpNoDelay(true));
	}
	//END LIDAR




	//LIDAR POINT CLOUD PARSING
	if(useLIDAR==1){


//https://answers.ros.org/question/261330/how-to-connect-vlp-16-velodyne-to-indigo/
//wiki.ros.org/velodyne/Tutorials/Getting Started with the HDL-32E
//https://phddocs.readthedocs.io/en/latest/hardware/puck_setup.html


		//https://abhishek4273.com/2014/07/01/opencv-and-pclfiltering-a-pcd-file/

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	 
		//pcl::io::loadPCDFile (argv[1], *cloud);  PCDcloud
		pcl::io::loadPCDFile (PCDcloud, *cloud);
		pcl::copyPointCloud( *cloud,*cloud_filtered);

		cout << "Cloud read and copied ............" << endl;
	 
		float i;
		float j;
		float k;
	 
		cv::namedWindow( "picture");
	 
		cvCreateTrackbar("X_limit", "picture", &a, 190, NULL);
		cvCreateTrackbar("Y_limit", "picture", &b, 190, NULL);
	  	cvCreateTrackbar("Z_limit", "picture", &c, 190, NULL);
	 
		a = a - 30;
		b = b - 30;
		c = c - 30;

		char last_c = 0;

		//pcl::visualization::PCLVisualizer viewer ("picture");
		
		viewer.setBackgroundColor (0.06, 0.06, 0.095);//set backgroung according to the color of points

		pcl::PassThrough<pcl::PointXYZ> pass;

		cout << "PCDcloud FILE = " << PCDcloud << endl;		
		bool viewerInitialized = false;
		while (!viewer.wasStopped ())
		{
			if(!viewerInitialized){
				viewerInitialized=true;
				pcl::copyPointCloud(*cloud_filtered, *cloud);        
		 
				i = 0.1*((float)a);
				j = 0.1*((float)b);
				k = 0.1*((float)c);		
			 
				//cout << "i = " << i << " j = " << j << " k = " << k << endl;

				//cout << "PCDcloud FILE = " << PCDcloud << endl; 	 
		
				pass.setInputCloud (cloud);
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (0.6, 4.1);(-j, j);//(0.8, 2.5);//-j,j //(0.6, 7.1);
				pass.filter (*cloud);
			 
				pass.setInputCloud (cloud);
				pass.setFilterFieldName ("x");
				pass.setFilterLimits (-i, i); //-i,i
				pass.filter (*cloud);

				pass.setInputCloud (cloud);
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (-k,k); //-k,k
				pass.filter (*cloud);


				cloud->sensor_origin_ << 0.0, 0, 0.0, 0;
	  			//cloud->sensor_orientation_ = Eigen::Quaternionf (0, 1, 0, 1);


	cout << "PCDcloud FILE 1111 .......   1 1 1= " << endl;	

				//FIND NORMALS
				//#include <pcl/features/normal_3d.h>
				//http://pointclouds.org/documentation/tutorials/normal_estimation.php
				// Create the normal estimation class, and pass the input dataset to it
				pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
				ne.setInputCloud (cloud);

				// Create an empty kdtree representation, and pass it to the normal estimation object.
				// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
				ne.setSearchMethod (tree);

				// Output datasets
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

				// Use all neighbors in a sphere of radius 3cm
				ne.setRadiusSearch (0.032 * 100); //(0.03 * 100);

				// Compute the features
				ne.compute (*cloud_normals);
				//cout << "Cloud Normals = " << cloud_normals->points.size () << "cloud points = " << cloud->points.size () << endl;  
				//END FIND NORMALS

	cout << "PCDcloud FILE 2222 .......   2 2 2 = " << endl;

				//TEST PCDs
				//https://sourceforge.net/projects/pointclouds/files/PCD%20datasets/


				bool meshGeneration = true;
				if(meshGeneration){
					// Concatenate the XYZ and normal fields*
					pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
				  	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
				  	//* cloud_with_normals = cloud + normals
	
				  	// Create search tree*
				  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
				  	tree2->setInputCloud (cloud_with_normals);

				  	// Initialize objects
				  	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
				  	pcl::PolygonMesh triangles;

				 	// Set the maximum distance between connected points (maximum edge length)
				  	gp3.setSearchRadius (0.745); //10.025 //0.025 //5.025

				  	// Set typical values for the parameters
				 	gp3.setMu (8.5); //2.5
				  	gp3.setMaximumNearestNeighbors (19); //100
				  	gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees //M_PI/4
				  	gp3.setMinimumAngle(M_PI/18); // 10 degrees
				  	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
				  	gp3.setNormalConsistency(false);

				  	// Get result
				  	gp3.setInputCloud (cloud_with_normals);
				  	gp3.setSearchMethod (tree2);
				  	gp3.reconstruct (triangles);

				  	// Additional vertex information
				  	std::vector<int> parts = gp3.getPartIDs();
				  	std::vector<int> states = gp3.getPointStates();

					//save file
					pcl::io::saveVTKFile ("mesh.vtk", triangles);

					// Load the input file
					pcl::PolygonMesh mesh;
					pcl::io::loadPolygonFileVTK ("mesh.vtk", mesh);

//https://github.com/PointCloudLibrary/pcl/blob/master/test/surface/test_gp3.cpp
					//pcl::io::saveOBJFile ("mesh.obj", triangles);
pcl::TextureMesh texMesh;
//texMesh.tex_polygons = triangles.polygons; // <<<<<<< Error <no operator "=" matches these operands>
//texMesh.header = triangles.header;
//texMesh.cloud = triangles.cloud;
texMesh.cloud = triangles.cloud;
    // add the 1st mesh
texMesh.tex_polygons.push_back(triangles.polygons);
//gp3.updateMesh(cloud_with_normals, triangles, texMesh);
//pcl::io::saveOBJFile("model.obj",texMesh); 


/// VTK TO PLY
// Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName ("mesh.vtk");
  reader->Update ();
  polydata = reader->GetOutput ();
 // print_info ("Loaded %s with %lu points/vertices.\n","mesh.vtk", polydata->GetNumberOfPoints ());

  // Convert to PLY and save
  vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New ();
  writer->SetInputData (polydata);
  writer->SetArrayName ("Colors");
  writer->SetFileTypeToASCII ();
  writer->SetFileName ("mesh.ply");
writer->Write ();


					//using namespace pcl;
					//using namespace pcl::io;
					//using namespace pcl::console;
					
					//print version of PCL
					std::cout << PCL_VERSION << std::endl;
				}	

				viewer.addCoordinateSystem(0.05);
				//viewer.addCoordinateSystem(0.05f,cloud->points[0].x,cloud->points[0].y,cloud->points[0].z );
				viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 0.15f,"scene_cloud1");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "scene_cloud1");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.513, "scene_cloud1"); //0.003

	cout << "PCDcloud FILE 2222 .......   3 3 3 = " << endl;

				viewer.addPointCloud (cloud, "scene_cloud");
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scene_cloud");
				//viewer.addPointCloudNormals (cloud_normals, "scene_cloud");
				//viewer -> addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");			
				//viewer.addPointCloudNormals(cloud, cloud_normals, 10, 0.05);
				//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, cloud_normals, 10, 0.05f,viewname,0);
				//cout << "Cloud Normals = " << cloud_normals->points[0] << "cloud points1 = " << cloud->points[0] << endl; 
				//cout << "Cloud Normals = " << cloud_normals->points[10] << "cloud points2 = " << cloud->points[10] << endl;  
				//cout << "Cloud Normals = " << cloud_normals->points[30] << "cloud points3 = " << cloud->points[30] << endl;  
			
	cout << "PCDcloud FILE 2222 .......   4 4 4 = " << endl;

				////CREATE 21EDRON DETECTOR
				//ppf_match_3d::PPF3DDetector SolidDetector(0.025, 0.05);
				//https://github.com/opencv/opencv_contrib/blob/3adc7ad61826600fbdc6dacb9bccc81bc00ef35a/modules/surface_matching/src/ppf_helpers.cpp
				//https://docs.opencv.org/3.0-beta/modules/surface_matching/doc/surface_matching.html
				if(1==0 && !solidTrained){				

					Mat pc = Mat(cloud->points.size (), 6, CV_32FC1);
					for (int i = 0; i < cloud->points.size (); i=i+10)
					{
						pc.at<float>(i,  0) = cloud->points[i].x;
						pc.at<float>(i,  1) = cloud->points[i].y;
						pc.at<float>(i,  2) = cloud->points[i].z;
						pc.at<float>(i,  3) = cloud_normals->points[i].normal_x;
						pc.at<float>(i,  4) = cloud_normals->points[i].normal_y;
						pc.at<float>(i,  5) = cloud_normals->points[i].normal_z;
					}

					//cout << "Cloud Normals = " << cloud_normals->points[30].normal_z << "cloud points3 = " << cloud->points[30].x << endl;
					SolidDetector.trainModel(pc);

					vector<Pose3DPtr> results;
					SolidDetector.match(pc, results, 1.0/10.0, 0.05);
					cout << "Poses: " << endl;
					// print the poses
					for (size_t i=0; i<results.size(); i++)
					{
						if(i==0){
						    solidTrained=true;		
						    Pose3DPtr pose = results[i];
						    cout << "Pose Result " << i << endl;
						    pose->printPose();

							//matrix.at<unsigned char>(j,i)[0]

						    //cout << "pose = " << pose->pose[0] << ","<< pose->pose[1] << ","<< pose->pose[2] << ","<< pose->pose[3] << "," << endl;
						//	 cout << pose->pose[4] << ","<< pose->pose[5] << ","<< pose->pose[6] << ","<< pose->pose[7] << "," << endl;
						//	 cout << pose->pose[8] << ","<< pose->pose[9] << ","<< pose->pose[10] << ","<< pose->pose[11] << "," << endl;
						//	 cout << pose->pose[12] << ","<< pose->pose[13] << ","<< pose->pose[14] << ","<< pose->pose[15] << endl;
						    cout << "results[i].pose1 = " << pose->alpha << " Votes:" << pose->numVotes << endl;
						}
					}
					//print pose
					//https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_load_match.cpp
					//cout << "results[i].pose = " << results[0]->pose << endl;

					//https://github.com/opencv/opencv_contrib/blob/3adc7ad61826600fbdc6dacb9bccc81bc00ef35a/modules/surface_matching/src/pose_3d.cpp
				}
				////END CREATE 21EDRON DETECTOR
			}//End VIWER INITIALIATION

cout << "PCDcloud FILE 2222 .......   5 5 5 = " << endl;

			viewer.spinOnce ();
			//viewer.removePointCloud("scene_cloud");
			//viewer.removePointCloud("scene_cloud1");

cout << "PCDcloud FILE 2222 .......   6 6 6 = " << endl;

			waitKey(10);
		}
	}
	//END LIDAR POINT CLOUD PARSING






	//ARUCO END
	image_transport::Subscriber sub = it.subscribe("left/image_raw", 1, imageCallback);

	// POZYX THREAD
	if(usePOZYX==1){
		// Constructs the new thread and runs it. Does not block execution.
	    	thread t1(task1, "Hello");
	    	// Makes the main thread wait for the new thread to finish execution, therefore blocks its own execution.
	    	//t1.join();
	}

	ros::spin();

	cv::destroyWindow("view");

	if(1==0){
		bool quit = false;
		while (!quit) { 
		//CONTROLS
		/* Show image */
			    	//cv::imshow(windowName.c_str(), srcImg);
			    	char key1 = cv::waitKey(10);

			    	/* Keyboard options */
			    	switch (key1) {
			    	/* When press the 'q' key then quit. */
			    	case 'q':				
			    		quit = true;
					//system ("PAUSE");
			    		break;
			    	/* When press the '[' key then decrease the exposure time. */
			    	case '[':
			    		//exposure = camera.get_control("Exposure (Absolute)");
			    		//camera.set_control("Exposure (Absolute)", --exposure);
					//system ("RESUME");
			    		break;

					/* When press the ']' key then increase the exposure time. */
			    	//case ']':
			    		//exposure = camera.get_control("Exposure (Absolute)");
			    		//camera.set_control("Exposure (Absolute)", ++exposure);
			    		//break;

					/* When press the '-' key then decrease the brightness. */
			    	//case '-':
			    		//exposure = camera.get_control("Brightness");
			    		//camera.set_control("Brightness", --brightness);
			    		//break;

					/* When press the '=' key then increase the brightness. */
			    	//case '=':
			    		//exposure = camera.get_control("Brightness");
			    		//camera.set_control("Brightness", ++brightness);
			    		//break;
			    	default:
			    		break;
			    	}
		}
		return 0;
	}
}

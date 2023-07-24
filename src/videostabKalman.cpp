//BACK148
//./track ./vids/exp2a3.avi 190 0.55 0 1 0 1 0 0 1
//./track ./vids/DJI_0219.MP4 0 1 1 1 1 1 0 0 1
//./track ./vids/DJI_0219.MP4 30 1 0 1 0 1 0 0 1
//./track ./vids/video5.avi 140 0.55 0 1 0 1 0 0 1
//./track ./vids/video6b.mp4 140 0.55 0 1 0 1 0 0 1
//./track ./vids/video5b.avi 0 0.75 0 1 0 1 0 0 1
//./track ./vids/video5.avi 0 0.55 0 1 1 2 0 0 1

//./track ./13/outVIDEO_2021-05-24_05-06-31.avi 2220 0.75 0 1 0 2 0 0 1 0.3 0.025 9 0.2 0.1 26 45 416 0.2
//./track ./13/outVIDEO_2021-05-24_05-06-31.avi 2220 1 1 1 0 2 0 0 1 0.3 0.025 9 0.2 0.1 26 45 416 0.2
//
//plotFromVikon = false;
//
//BACK 136- 138 HOMOG only paper
//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 0 1 1 0 1
//./track ./vids/video5.avi 1 0.65 0 1 0 1 0 0 1
//./track ./vids/video2.avi 1 0.65 0 1 0 1 0 0 1
//./track ./vids/video1.avi 1 0.65 0 1 0 1 0 150 1
//./track ./vids/Tube1.mp4 1 1 1 0 0 1 0 0 1
//./track ./vids/M1.mp4 1 1 1 0 0 1 0 0 1


//Start Frame (offline video)
//Downscale factor
//Draw all rectangles (0,1), use 2ond layer (0,1), use PTZ (0,1), multiply source resolution (640,360)
//Use 2ond layer (0,1), use PTZ (0,1), multiply source resolution (640,360)
//Use PTZ (0,1), multiply source resolution (640,360)
//Multiply source resolution (640,360)
//Cut propellers (Vulkan)
//Lower screen cutoff (pixels)
//Debug mode on(1) - off(0)

//RUN NO PTZ with VICON PLOT (older files) on
//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 0 1 1 0 1 (use old log read method !! before added time and add plotfromvicon - enable FUSION1 - half PTZ scale B)
//./track ./vids/Tube1.mp4 1 1 1 0 0 1 0 0 1 (remove plot from vicon to run - enable //Tube1,4.mp4)

// ./track ./vids/video1.avi 1 1 0 1 0 1 0 150 1 (diable all extra settings to 1==0 and use initial defaults !!!)

// ./track ./vids/video6b.mp4 1 0.45 0 1 0 1 0

/*
Thanks Nghia Ho for his excellent code.
And,I modified the smooth step using a simple kalman filter .
So,It can processes live video streaming.
modified by chen jia.
email:chenjia2013@foxmail.com
*/
//g++ -ggdb videostabKalman.cpp -o stab `pkg-config --cflags --libs /home/nasos/installation/OpenCV-3.4.4/lib/pkgconfig/opencv.pc` -std=c++11 
//perspective correction - warpaffineperspecive
//https://www.learnopencv.com/image-alignment-feature-based-using-opencv-c-python/
//https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html

//./track ./vids/video1.avi 1 0.85 0 1 0

//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 0.9 0 1 0 1 1
//./track ./vids/Tube1.mp4 1 0.95 1 0 0 2 0

//./track ./RESULTS1/outVIDEO_2020-01-14_14-48-37.avi 0 1 0 1 1 1 1 0 1







#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <fstream>

//MOSSE
#include <opencv2/tracking.hpp>
//#include <opencv2/tracking/tracker.hpp>
#include <opencv2/video/tracking.hpp>
//MOSSE TWEAKBLE
//#include "./mosse2Tracker.cpp"
//#include <opencv2/tracking/tracking_legacy.hpp>
//https://github.com/opencv/opencv_contrib/blob/f1c3d0e5e789a213ab8ab2828f360e520a3b35ce/modules/tracking/samples/samples_utility.hpp
//https://github.com/opencv/opencv_contrib/issues/2780


//CLUSTER
#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#include "dbscan.h"
#include "dbscan.cpp"
//#include <vector>
//#include <cmath>
#include <algorithm>
//CLUSTER DBSCAN
#include <stdio.h>

//ORB
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
//END ORB





///////////// YOLO 4
//#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <numeric>
//#include <darknet.h>
//#include <opencv2/dnn.hpp>
//#include <opencv2/dnn/all_layers.hpp>
//#include <opencv2/highgui.hpp>



//PTZ - ROS
//PTZ - ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include "std_msgs/String.h"
//GPS
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PoseStamped.h>
sensor_msgs::NavSatFix GPSfix, GPSfix_raw;
sensor_msgs::TimeReference TimeReference;
geometry_msgs::PoseStamped localPose;

#include <iomanip>
#include <time.h>




//NEW1
int YoloA_enter_and_found = 0; //if enter become 1, if found become 2
int YoloB_enter_and_found = 0;
int Mosse_enter_and_found = 0;
int Homog_enter_and_found = 0;
int YoloA_enter_and_found1 = 0; //if enter become 1, if found become 2
int YoloB_enter_and_found1 = 0;
int Mosse_enter_and_found1 = 0;
int Homog_enter_and_found1 = 0;
int YoloA_enter_and_found2 = 0; //if enter become 1, if found become 2
int YoloB_enter_and_found2 = 0;
int Mosse_enter_and_found2 = 0;
int Homog_enter_and_found2 = 0;
int not_found_long_time = 0;


int frameID=0;	

float GPSPosLat = 0;
float GPSPosLon = 0;
float GPSPosAlt = 0;
float GPSTime = 0;

std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}
void getGpsPositionCallback (const sensor_msgs::NavSatFix::ConstPtr& NavSatFixRaw_) //(const std_msgs::UInt32::ConstPtr& msg)
{
	GPSfix_raw=*NavSatFixRaw_;
	//Logging
	//outfile << TimeReference.time_ref.sec << ";" << TimeReference.time_ref.sec <<  ";" 
	//			<< GPSfix.latitude << ";" << GPSfix.longitude << ";" << GPSfix.altitude << ";"
	//				<< GPSfix_raw.latitude << ";" << GPSfix_raw.longitude << ";" << GPSfix_raw.altitude << ";"
	//					<< localPose.pose.position.x << ";" << localPose.pose.position.y << ";" << localPose.pose.position.z << ";"
	//							<< std::endl;

	//cout << "GPS TEST 1" << endl;
	//ROS_INFO("I heard: [%i]", msg->data);
	GPSPosLat = GPSfix.latitude;
	GPSPosLon = GPSfix.longitude;
//	GPSPosAlt = GPSfix_raw.altitude;
	//GPSTime = 0;
}
void getGpsTimeCallback (const sensor_msgs::TimeReference::ConstPtr TimeReference_) //(const std_msgs::UInt32::ConstPtr& msg)
{
	TimeReference=*TimeReference_;

	//ROS_INFO("I heard: [%i]", msg->data);
	GPSTime = TimeReference.time_ref.sec * 1000000000 + TimeReference.time_ref.nsec ;
}
//get local position output from EKF
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& localPose_)
{
	localPose=*localPose_;
	GPSPosAlt = localPose.pose.position.z; //GET ALTITUDE FROM BAROMETER !!!
}
//END ROS
//PTZ CONTROLS
int toggleThread = 0;
/* UDP client in the internet dom-ain */
//https://stackoverflow.com/questions/11702673/send-hex-values-through-udp-socket
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
//void error(const char *);
int UDPcounter = 0;
int UDPdelay = 2;

unsigned char *ascii_to_utf8(unsigned char c)
{
	unsigned char *out;	
	if(c < 128)
	{
		out = (unsigned char *)calloc(2, sizeof(char));
		out[0] = c;
		out[1] = '\0';
	}
	else
	{
		out = (unsigned char *)calloc(3, sizeof(char));
		out[1] = (c >> 6) | 0xC0;
		out[0] = (c & 0x3F) | 0x80;
		out[2] = '\0';
	}	
	return out;
}
unsigned char* ASCIItoUNICODE(unsigned char ch)                                                             
{                                                                                                            
	//unsigned char Val[2];
 	unsigned char*  Val = (unsigned char*) malloc (2);                                                                                        
	if ((ch < 192)&&(ch != 168)&&(ch != 184))  {Val[0] = 0; Val[1] = ch;    return Val;}                         
	if (ch == 168) {Val[0] = 208;   Val[1] = 129;   return Val;}                                                 
	if (ch == 184) {Val[0] = 209;   Val[1] = 145;   return Val;}                                                 
	if (ch < 240)  {Val[0] = 208;   Val[1] = ch-48; return Val;}                                                 
	if (ch < 249)  {Val[0] = 209;   Val[1] = ch-112;        return Val;}                                         
}                                                                                                            
unsigned int* ConvertString (unsigned char *string)                                                          
{                                                                                                            
	unsigned int size=0, *NewString;                                                                         
	unsigned char* Uni;                                                                                  
	while (string[size++]!=0);                                                                               
		NewString = (unsigned int*)malloc(sizeof(unsigned int)*2*size-1);                                    
		NewString[0]=2*size-1;                                                                               
		size=0;                                                                                              
		while (string[size]!=0)                                                                              
		{                                                                                                    
		    Uni = ASCIItoUNICODE(string[size]);                                                              
		    NewString[2*size+1]=Uni[0];                                                                      
		    NewString[2*size+2]=Uni[1];                                                                      
		    size++;                                                                                          
		}                                                                                                    
        return NewString;                                                                                    
}  

std::wofstream fs("testout.txt");
float currentPan = 0;
float currentTilt = 0;
float currentZoom = 0;

float prevPan = 0;
float prevTilt = 0;
float prevZoom = 0;

bool tcpInit = false;
bool tcpSent = false;

bool udpSend(std::string hex, bool tcpMode, bool zoomMode){//const char *msg){

if(UDPcounter == 0 || 1==1){
   int sock, n;
   unsigned int length;
   struct sockaddr_in server;
   struct hostent *hp;   
   int srLength = strlen(hex.c_str()) ;  
   std::stringstream ss; 
   unsigned char buffer[srLength/2];
   unsigned int buffer1;
   int offset = 0;
   int countIn1 = 0;
   while (offset < hex.length()) {
	ss.clear();
	ss << std::hex << hex.substr(offset, 2);
	ss >> buffer1;
	//hexCh.push_back(static_cast<unsigned char>(buffer1));
	buffer[countIn1] = static_cast<unsigned char>(buffer1);
	offset += 2;
        countIn1++;
   }

   hp = gethostbyname("192.168.10.107");
   if (hp==0) {
	//error("Unknown host");
   }

	if(!tcpMode){
		sock = socket(AF_INET, SOCK_DGRAM, 0);
	}else{
		sock = socket(AF_INET, SOCK_STREAM, 0);//sock = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	}
	if (sock < 0) {
		//error("socket");
	}   

	server.sin_family = AF_INET;
	bcopy((char *)hp->h_addr, 
		(char *)&server.sin_addr,
		hp->h_length);
	int port_ID = atoi("1259");
	if(tcpMode){
		port_ID = atoi("5678");
	}
	server.sin_port = htons(port_ID);

	if(tcpMode){
		int status;
		// connect the client socket to server socket 
		if (connect(sock, (const struct sockaddr *)&server, sizeof(server)) != 0) {  
			printf("connection with the server failed...HARD\n"); 			
		} 
		else{
			//printf("connected to the server.. sending get request\n"); 
			// function for chat     			
			unsigned char buff[32] = {0};    			

			if(send(sock, buffer, sizeof(buffer) , 0) < 0)
			{
				printf("Server send FAILED");				
			}else{
				//cout << "Sent to server:  " << buffer << endl;
			}

			//sendto(sock,buffer,sizeof(buffer),0,(const struct sockaddr *)&server,length); 
			bzero(buffer, sizeof(buffer)); 

			//printf("Reading From Server : %s", buffer); 
			vector<char> result;			
			
			for (int i = 0; i < 1; i++){
					wchar_t input[2048];
					wstring message;
					unsigned int data_receive;								

					char bufferA[1] = {};
				  	string reply;
					int countIn = 10;//PAN - TILT !! (-4 = 6 for zoom)

					if(zoomMode){
						countIn = 6;
					}
					
					int counterWhile = 0;	

					//CONNECT READ STREAM					
					char pan_Zoom[]="0000";
					char tilt[]="0000";					
	
				  	while (countIn > 0) {						
				    		if( recv(sock , bufferA , sizeof(bufferA) , 0) < 0)
				    		{							
				    		}else{
							reply += bufferA[0];
							char Hexy = *bufferA;							
						
							if(counterWhile >= 2 && counterWhile <= 5){								
								pan_Zoom[counterWhile-2] = Hexy;								
							}
							if(counterWhile >= 6 && counterWhile <= 9){								
								tilt[counterWhile-6] = Hexy;
							}
						}						
						counterWhile++;
						countIn--;
					}
					
					char bufHex[4];
					std::stringstream ss1;	
					for(int j = 0; j < 4; j++){
					    sprintf(bufHex, "%X", pan_Zoom[j]);
					    ss1.clear();
					    ss1 << bufHex;								
					}
					std::string result(ss1.str());					
					int PAN_ZOOM = (int)strtol(result.c_str(), 0, 16);				
	
					if(!zoomMode){
						std::stringstream ss2;						
						for(int j = 0; j < 4; j++){
						    sprintf(bufHex, "%X", tilt[j]);
						    ss2.clear();
						    ss2 << bufHex;								
						}
						std::string result1(ss2.str());
						//cout << "TILT HEX: " << result1 << endl;

						int TILT = (int)strtol(result1.c_str(), 0, 16);
						//cout << "TILT DECIMAL: " << TILT << endl;

						//FILL GLOBAL PAN VARIABLE, with PAN_ZOOM
						currentPan = PAN_ZOOM;	
						//FILL GLOBAL TILT VARIABLE, with TILT
						currentTilt = TILT;	
					}else{
						//FILL GLOBAL ZOOM VARIABLE, with PAN_ZOOM
						currentZoom = PAN_ZOOM;				
					}					
					close(sock);
			}			
		}
	}else{
	   length=sizeof(struct sockaddr_in);	  
		n=sendto(sock,buffer,sizeof(buffer),0,(const struct sockaddr *)&server,length);
		if (n < 0) {			
		}	   
	}
   	close(sock);
}
   UDPcounter++;
   if(UDPcounter > UDPdelay){
     UDPcounter = 0;
   }
} 
 
// Convert to string 
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()
 


float panRate = 0; float tiltRate = 0; float zoomRate = 0;
//END PTZ - ROS
#include <thread>
//END PTZ - ROS

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::dnn; 

#include <opencv2/core/ocl.hpp> //GPU OPENCL
//OCL FEATURE TRACKER
void UMatToVector(const UMat & um, std::vector<Point2f> & v) 
{
	v.resize(um.size().area());
        um.copyTo(Mat(um.size(), CV_32FC2, &v[0]));
}

// Convert to string 
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()




//PLOT VIKON DATA
#include <Eigen/Dense>
#include <Eigen/Geometry> 
using Eigen::MatrixXd;
using Eigen::Quaternion;
using Eigen::Transform;
//ROTATION
//https://www.learnopencv.com/rotation-matrix-to-euler-angles/
// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<float>(3,3) << //Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<float>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<float>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
     
    return R;
 
}
void multiplyQuaternion(const Mat& q1,const Mat& q2, Mat& q)
{
    // First quaternion q1 (x1 y1 z1 r1)
    const float x1=q1.at<float>(0);
    const float y1=q1.at<float>(1);
    const float z1=q1.at<float>(2);
    const float r1=q1.at<float>(3);

    // Second quaternion q2 (x2 y2 z2 r2)
    const float x2=q2.at<float>(0);
    const float y2=q2.at<float>(1);
    const float z2=q2.at<float>(2);
    const float r2=q2.at<float>(3);


    q.at<float>(0)=x1*r2 + r1*x2 + y1*z2 - z1*y2;   // x component
    q.at<float>(1)=r1*y2 - x1*z2 + y1*r2 + z1*x2;   // y component
    q.at<float>(2)=r1*z2 + x1*y2 - y1*x2 + z1*r2;   // z component
    q.at<float>(3)=r1*r2 - x1*x2 - y1*y2 - z1*z2;   // r component
}
//void rotate_vector_by_quaternion(const Vector3& v, const Quaternion& q, Vector3& vprime)
//{
    // Extract the vector part of the quaternion
//    Vector3 u(q.x, q.y, q.z);

    // Extract the scalar part of the quaternion
//    float s = q.w;

    // Do the math
//    vprime = 2.0f * dot(u, v) * u
//          + (s*s - dot(u, u)) * v
//          + 2.0f * s * cross(u, v);
//}
void rotate_vector_by_quaternion(const Vec3f& v, const Mat& q, Vec3f& vprime)
{
    // Extract the vector part of the quaternion
    Vec3f u(q.at<float>(0), q.at<float>(1), q.at<float>(2)); //x,y,z,w = 0,1,2,3

    // Extract the scalar part of the quaternion
    float s = q.at<float>(3);

    // Do the math
    vprime = 2.0f * u.dot(v) * u
          + (s*s - u.dot(u)) * v
          + 2.0f * s * u.cross(v);
}
//Mat eulerAnglesToRotationMatrix(Vec3f &theta)
//void multiplyQuaternion(const Mat& q1,const Mat& q2, Mat& q)
//void rotate_vector_by_quaternion(const Vec3f& v, const Mat& q, Vec3f& vprime)
//END PLOT VIKON DATA




//GLOBALS
vector<Rect> estimatedWindows;
Rect2d bbox2(1, 1, 100, 100);
Rect2d bbox2P(1, 1, 100, 100);
#define CVA_PI   3.1415926535897932384626433832795

//ANGLE BETWEEN VECTORS
float angleBetween(const Point &v1, const Point &v2)
{
    float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    float dot = v1.x * v2.x + v1.y * v2.y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return CVA_PI;
    else
        return acos(a); // 0..PI
}

//PLOT RECTANGLES
int drawAllRectangles = 1;
Rect getTrackWindowAroundPoints(float scaleFactorLK1, Mat plotPoints, Mat canvas, bool plotDebug, int maxContours){
				//DRAW RECTANGLE AROUND DRONE
				Rect trackFrameEstimate(-100, -100, 10, 10);//put out of frame to initialize
				Point trackFrameEstimateCenter;
				const bool useGpu = true;
				cv::ocl::setUseOpenCL(useGpu);
				cv::dilate(plotPoints.getUMat(ACCESS_RW), plotPoints.getUMat(ACCESS_RW), getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));
				float scaling = 1; //scaleFactorLK1 
				vector<vector<Point> > contours;
        			vector<Vec4i> hierarchy;
				findContours(plotPoints, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));
				//Find contours
			 	vector<vector<Point> > contours_poly(contours.size());
				vector<Rect> boundRect(contours.size());
				//approximate contours by rectangles
				for (int i = 0; i < contours.size(); i++)
				{
				  approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
				  boundRect[i] = boundingRect(Mat(contours_poly[i]));
				}
				//draw bounded recatangles				
				int areaThreshold = 5;
				float prevArea = 0;
				int rectID = -1;
				//SEARCH FOR BIGGEST AREA RECTANGLE - EXTEND TO MULTIPLE TARGETS LATER
				if(contours.size() > 0 &&  contours.size() < maxContours){ //25 //55					
					for (int i = 0; i< contours.size(); i++)
					{ 							
						bool nearScreenAndSmall = false;
						int offset = 10;
						if(boundRect[i].x < offset || boundRect[i].x + boundRect[i].width > canvas.cols - offset 
						|| boundRect[i].y < offset || boundRect[i].y + boundRect[i].height > canvas.rows - offset){
							//Check if box touches screen edges
							if(boundRect[i].width < canvas.cols/6 && boundRect[i].height < canvas.rows/6){
								nearScreenAndSmall = true;		
							}		
						}
						if(!nearScreenAndSmall){				 
							if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) > prevArea){
								rectID = i;				
								prevArea = contourArea(contours[i]);
							}

							if(drawAllRectangles == 1){
								rectangle(canvas, boundRect[i].tl() * (1/scaleFactorLK1), boundRect[i].br() * (1/scaleFactorLK1), CV_RGB(255, 0, 255), 2, 8, 0);
							}		
						}
					}

				}
				imshow("COUTURS", canvas);
				if(rectID >= 0){					
					trackFrameEstimate.x = boundRect[rectID].x * (1/scaleFactorLK1);
					trackFrameEstimate.y = boundRect[rectID].y * (1/scaleFactorLK1);
					trackFrameEstimate.width = boundRect[rectID].width * (1/scaleFactorLK1);
					trackFrameEstimate.height = boundRect[rectID].height * (1/scaleFactorLK1);					
					trackFrameEstimateCenter.x = trackFrameEstimate.x + trackFrameEstimate.width/2;
					trackFrameEstimateCenter.y = trackFrameEstimate.y + trackFrameEstimate.height/2;					
				}				
				return trackFrameEstimate;
}//END getTrackWindowAroundPoints


//CLUSTERING
struct EuclideanDistanceFunctor
{
    int _dist2;
    EuclideanDistanceFunctor(int dist) : _dist2(dist*dist) {}

    bool operator()(const Point& lhs, const Point& rhs) const
    {
        return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < _dist2;
    }
};


//ORB
//https://www.learnopencv.com/image-alignment-feature-based-using-opencv-c-python/
const int MAX_FEATURES = 1500;
const float GOOD_MATCH_PERCENT = 0.95f; 

//keep previous frame data
std::vector<KeyPoint> keypoints1, keypoints2;
Mat descriptors1, descriptors2;

//void alignImages(Mat &im1, Mat &im2, Mat &im1Reg, Mat &h)
void alignImages(Mat &im1Gray, Mat &im2Gray, Mat &h, std::vector<Point2f> &points1, std::vector<Point2f> &points2)
{   
	// Convert images to grayscale
	//Mat im1Gray, im2Gray;
	//cvtColor(im1Gray, im1Gray, CV_BGR2GRAY);
	//cvtColor(im2Gray, im2Gray, CV_BGR2GRAY);
	//cout << "im1Gray " << im1Gray.type() << endl;
	//cout << "im2Gray " << im2Gray.type() << endl;
	//cout << "im1Gray " << im1Gray.cols << endl;
	//cout << "im2Gray " << im2Gray.cols << endl;
	   
	// Variables to store keypoints and descriptors
	//std::vector<KeyPoint> keypoints1, keypoints2;
	//Mat descriptors1, descriptors2;
	   
points1.clear();
points2.clear();

	// Detect ORB features and compute descriptors.
	Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
	//pass previous current frame data keypoints1, descriptors1 to previous frame now, then update with new current frame below
	if(frameID < 2){
	 	orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);
		cout << "Computing first point samples with ORB" << endl;
	}else{
		//descriptors2 = descriptors1;
		//keypoints2 = keypoints1;
		keypoints2.clear();

		//keypoints2 = keypoints1;
		for( size_t i = 0; i < keypoints1.size(); i++ )
		{
			keypoints2.push_back( keypoints1[ i ] );		    	
		}

		keypoints1.clear();
		descriptors1.copyTo(descriptors2);
	}

	//UPDATE keypoints1, descriptors1
	orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);

///////////////////////keypoints2 = keypoints1;

	//cout << "descriptors1 " << descriptors1.cols << endl;
	//cout << "descriptors2 " << descriptors2.cols << endl;
	//imshow("im1", im1Gray);
	//imshow("im2", im2Gray);

	if((descriptors1.cols > 0 && descriptors2.cols == 0 ) || (descriptors2.cols > 0 && descriptors1.cols == 0) ){
		return;
	}
   
	// Match features.
	bool cleanUPPoints = true;

	if(cleanUPPoints){
		  // Match features.
		  std::vector<DMatch> matches;
		  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		  matcher->match(descriptors1, descriptors2, matches, Mat());
		   
		  // Sort matches by score
		  std::sort(matches.begin(), matches.end());
		   
		  // Remove not so good matches
		  const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
		  matches.erase(matches.begin()+numGoodMatches, matches.end());   
		   
		  // Draw top matches
		  //Mat imMatches;
		  //drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
		  //imwrite("matches.jpg", imMatches);   
		   
		  // Extract location of good matches
		  //std::vector<Point2f> points1, points2;
		   //points1.clear();
		   //points2.clear();
		  for( size_t i = 0; i < matches.size(); i++ )
		  {
		    points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
		    points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
		  }
	}else{
		//do not clean up
		int counter=keypoints1.size();
		if(keypoints1.size() > keypoints2.size()){
			counter=keypoints2.size();
		}
		for( size_t i = 0; i < counter; i++ )
		{
		    points1.push_back( keypoints1[ i ].pt );
		    points2.push_back( keypoints2[ i ].pt );
		}
	}
   
  // Find homography
if( points1.size()  == 0  ||points2.size()  == 0  ){
		return;
	}
cout << "points1 count" << points1.size() << endl;
cout << "points2 count" << points2.size() << endl;
  h = findHomography( points1, points2, RANSAC);
   
  // Use homography to warp image
  //warpPerspective(im1, im1Reg, h, im2.size());   
}
//END ORB


//CORRRELATION ESTIMATE
//https://answers.opencv.org/question/15842/how-to-find-correlation-coefficient-for-two-images/
//https://docs.opencv.org/master/d4/dc6/tutorial_py_template_matching.html
double correlation(cv::Mat &image_1, cv::Mat &image_2)   {

	// convert data-type to "float"
	cv::Mat im_float_1;
	image_1.convertTo(im_float_1, CV_32F);
	cv::Mat im_float_2;
	image_2.convertTo(im_float_2, CV_32F);

	int n_pixels = im_float_1.rows * im_float_1.cols;

	// Compute mean and standard deviation of both images
	cv::Scalar im1_Mean, im1_Std, im2_Mean, im2_Std;
	meanStdDev(im_float_1, im1_Mean, im1_Std);
	meanStdDev(im_float_2, im2_Mean, im2_Std);

	// Compute covariance and correlation coefficient
	double covar = (im_float_1 - im1_Mean).dot(im_float_2 - im2_Mean) / n_pixels;
	double correl = covar / (im1_Std[0] * im2_Std[0]);

	return correl;
}
//END CORRELATION ESTIMATE

///// PTZ
bool usePTZ = false;
bool cutPropellers = false; 
//FILE OUTPUT
std::ofstream outfile;

//READ PTZ PAN - TILT - ZOOM
int frameCounter = 0;
void task1(){
	if(usePTZ){
		//v0.2
		prevPan = currentPan;
		prevTilt = currentTilt;	
		//READ PAN FROM CAMERA
		std::string getPanTiltPTZ = "81090612FF"; 
		udpSend(getPanTiltPTZ, true, false);
	}	
}
void task2(){
	if(usePTZ){
		//v0.2
		prevZoom = currentZoom;	
		std::string getPanTiltPTZ = "81090447FF"; 
		udpSend(getPanTiltPTZ, true, true);	
	}
}
/// END PTZ







///////////////////// PTZ FLOW ADDITION ///////////////////
//SPARKFUN SENSOR
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#define DEBUG 1
#include <stdio.h>
//#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <string> 
#include <thread>

bool enableRollCompensation = false;
bool useExactRoll = true; 
float maxZoom = 16384;

//GLOBAL VARIABLES
bool plotDebug = true; //true //false
int ch=0;
char inputTERMINAL;
int plotDebugEnabled = 0;
bool plotDebugTrackingFUSION = true;
//int plotDebugFusionEnabled = 1;
bool PTZ_HOMOGRAPHY_FUSION = false;
bool enablePIDracking = false;
bool enablePIDZoom = false;
bool enableTestZoom = false;
bool enableTestPanTilt = false;
int maxTestZoom = 7800;
int maxTestTilt = 60000;
bool useSparkfun = true;//true; false
double YawRate =    0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (1+3)]);
double pitchRate =  0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (2+3)]);
double rollRate =   0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (3+3)]);
double accelXRate = 0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (6+3)]);
double accelYRate = 0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (5+3)]);
double accelZRate = 0;//(pozyxOUTPUT[pozyxOUTPUT.size() - (4+3)]);
//float panRate = 0; float tiltRate = 0; float zoomRate = 0;
//float panRate = 0; float tiltRate = 0; float zoomRate = 0;
double FClinearVelocityX;
double FClinearVelocityY;
double FClinearVelocityZ;

//SIGN
int sign(float x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}

//KEYBOARD CONTROLS
//Keyboard function used to terminate the program on user input
int kbhit(void)
{
	//cout << "  KEYBOARD HANDLER ..." << endl;

	struct termios oldt, newt;
	int oldf;
	
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	
	ch = getchar();
	
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	
	//if (ch != EOF)
	if (ch == 113 || ch == 81) //q
	{
	  return 0;

	}else if (ch == 84 ||ch == 116){ //t
	  	if(enablePIDracking){
				enablePIDracking = false;
				cout << "  -------------------------  Disabled PID ... ------------------------- " << endl;
		}else{
				enablePIDracking = true;
				cout << "  -------------------------  Enabled PID ... ------------------------- " << endl;
		}
	}else if (ch == 122 ||ch == 90){ //z
		if(enablePIDracking){
		  	if(enablePIDZoom){
					enablePIDZoom = false;
					cout << "  -------------------------  Disabled PID Zoom ... ------------------------- " << endl;
			}else{
					enablePIDZoom = true;
					cout << "  -------------------------  Enabled PID Zoom ... ------------------------- " << endl;
			}
		}else{
			if(enableTestZoom){
					enableTestZoom = false;
					cout << "  -------------------------  Disabled TEST PAN TILT Zoom ... ------------------------- " << endl;
			}else{
					enableTestZoom = true;
					cout << "  -------------------------  Enabled TEST PAN TILT Zoom ... ------------------------- " << endl;
			}
			
		}
	
	}else if (ch == 112 ||ch == 80){ //p
			//bool enableTestPanTilt = true;	
			if(enableTestPanTilt){
					enableTestPanTilt = false;
					cout << "  -------------------------  Disabled TEST PAN TILT ... ------------------------- " << endl;
			}else{
					enableTestPanTilt = true;
					cout << "  -------------------------  Enabled TEST PAN TILT ... ------------------------- " << endl;
			}
	}else if (ch == 115 ||ch == 83){ //s - sparkfun sensor on - off
			//bool enableTestPanTilt = true;	
			if(useSparkfun){
					useSparkfun = false;
					cout << "  -------------------------  Disabled Sparkfun ... ------------------------- " << endl;
			}else{
					useSparkfun = true;
					cout << "  -------------------------  Enabled Sparkfun ... ------------------------- " << endl;
			}
	}else if (ch == 72 ||ch == 104){ //h - HOME camera
		//go to home position				
		std::string homePTZ = "81010604FF";
		udpSend(homePTZ, false, false);
		//zoom out
		string PP = "2";//zoom speed 0 to 7
		std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
		udpSend(zoomPTZ, false, false);
		//stop motion
		string WW = "00";
		std::string panPTZ = "81010601"+WW+WW+"0303FF";
		cout << "  ------------------------- Reseting Camera to HOME position... -------------------------" << endl;
	}else if (ch == 114 ||ch == 82){ //r - roll enable
		if(enableRollCompensation){
			enableRollCompensation = false;
			cout << "  ------------------------- Roll compensation disabled ... -------------------------" << endl;
		}else{
			enableRollCompensation = true;
			cout << "  ------------------------- Roll compensation enabled ... -------------------------" << endl;
		}		
	}else if (ch == 70 ||ch == 102){ //f
		if(PTZ_HOMOGRAPHY_FUSION){
			PTZ_HOMOGRAPHY_FUSION = false;
			plotDebugEnabled = 1;
			//plotDebugFusionEnabled = false;
			cout << "  -------------------------  Disabled FUSION ... ------------------------- " << endl;
		}else{
			PTZ_HOMOGRAPHY_FUSION = true;
			plotDebugEnabled = 0;
			//plotDebugFusionEnabled = true;
			cout << "  -------------------------  Enabled FUSION ... ------------------------- " << endl;
		}
	}
	//bool enablePIDZoom = true;
	return 1;
}

//SPARKFUN SENSOR
//bool useSparkfun;//true; false
//ARDUINO COM READ (global variables)
int fd, n, i;
char buf[256] = "temp text";
std::vector<double> pozyxOUTPUT;
int POZYXsamplesCounted = 0;
//SPARKFUN THREAD
void initComSparkFun(){		
		struct termios toptions;		
		fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
		printf("fd opened as %i\n", fd);		
		usleep(3500000);		
		tcgetattr(fd, &toptions);		
		cfsetispeed(&toptions, B115200);
		cfsetospeed(&toptions, B115200);		
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;		
		toptions.c_lflag |= ICANON;		
		tcsetattr(fd, TCSANOW, &toptions);
}
//ROTATION
//https://www.learnopencv.com/rotation-matrix-to-euler-angles/
// Calculates rotation matrix given euler angles.


//HANDLE INPUT ASYNC
//TERMINAL INPUT
void task4()
{
		char inputTERMINAL;
       		cin >> inputTERMINAL;
		if(inputTERMINAL == 't' || inputTERMINAL == 'T') //(T)oggle PID - test sin
		{
			if(enablePIDracking){
				//enablePIDracking = false;
				//cout << "Disabled PID ..." << endl;
			}else{
				//enablePIDracking = true;
				//cout << "Enabled PID ..." << endl;
			}
			//bool enablePIDracking = false;
			//bool enablePIDZoom = true;
		    //break;
		}
		if(inputTERMINAL == 'q' || inputTERMINAL == 'Q') //(T)oggle PID - test sin
		{
			//break;
		}
}

Mat GravityPerAxis;
//v0.1
float sumDX = 0; 
float sumDY = 0;
double velocityXGrav = 0;
double velocityYGrav = 0;
double velocityZGrav = 0;
double dXGrav = 0;
double dYGrav = 0;
double dZGrav = 0;
double TRANSLATION_X_Grav = 0;
double TRANSLATION_Y_Grav = 0;
double TRANSLATION_Z_Grav = 0;

vector<double> bufferAccelX;
vector<double> bufferAccelY;
vector<double> bufferAccelZ;
Vec3f Eulers;
Vec3f EulersRAD;

double prevTimeSPARK = (double)getTickCount();

void task3(string msg)
{	
	cout << "thread start"  << endl;
	if(!useSparkfun){
		cout << "thread ended"  << endl;
		return; //return here than use if when creating the thread, as this GAVE A BUG !!!! because after if is done will terminate the thread !!!!!!!
		//THIS DID NOT ALWAYS HAPPEN, it worked before, so to check
	}

	while(2>1){	
		//ARDUINO COM READ
		/* Send byte to trigger Arduino to send string back */
		//write(fd, "0", 1);
		/* Receive string from Arduino */
		n = read(fd, buf, 256);
		/* insert terminating zero in the string */
		buf[n] = 0;
		//SPLIT NUMBERS
		std::vector<double> vectMEASUREMENTS;
		std::stringstream ss(buf);
		int icount;
		//cout << "ss = " << buf << endl;
		//while (ss >> icount)
		//{
		//	vectMEASUREMENTS.push_back(icount);
		//	cout << "icount = " << icount << endl;
		//	if (ss.peek() == ','){
		//		ss.ignore();
		//	}
		//	if (ss.peek() == ' '){
		//		ss.ignore();
		//	}
		//}

		std::string s = buf;
		std::string delimiter = ", ";

		size_t pos = 0;
		std::string token;
		while ((pos = s.find(delimiter)) != std::string::npos) {
		    token = s.substr(0, pos);
		    //std::cout << token << std::endl;
		    double temp = ::atof(token.c_str());
		    vectMEASUREMENTS.push_back(temp);
		    s.erase(0, pos + delimiter.length());
		}
		//std::cout << s << std::endl;

		int time = 0;
		int pos1 = 1;
		int pos2 = 2;
		int pos3 = 3;
		int rot1 = 4;
		int rot2 = 5;
		int rot3 = 6;
		
		//MESUREMENTS EXPLAINED
		//https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide?_ga=2.98187228.239901204.1571724721-1885329595.1571724721
		// accelX is x-axis acceleration in g's
		// gyroX is x-axis rotation in dps
		// magX is x-axis magnetic field in uT

		//cout << "vectMEASUREMENTS size = " << vectMEASUREMENTS.size() << endl;
		for (i=0; i< vectMEASUREMENTS.size(); i++){
			//cout << "vectMEASUREMENTS.at  " << i << " =" << vectMEASUREMENTS.at(i) << endl;			
			//if(i == time||i == pos1 || i==pos2 || i==pos3){				
				//pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			//}
			//if(i == rot1 || i==rot3 || i==rot3){				
				pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			//}			
		}

		//cout << endl;

		//KEEP EULER
		//Vec3f Eulers;


		//time, heading,accelration, gyro, euler (T,H,A,G,E in Sparkfun setup)


		for (i=0; i< pozyxOUTPUT.size(); i++){
			//if(i == rot1 || i==rot2 || i==rot3){				
				//float angle = vectMEASUREMENTS.at(i); 
			if(i == pozyxOUTPUT.size() - (7+3)){//if(i == pozyxOUTPUT.size() - 7){
				//cout << endl;std::cout << "Time:" << endl;
			}
			if(i == pozyxOUTPUT.size() - (6+3)){//6
				//cout << endl;std::cout << "Acceleration X,Y,Z:" << endl;
			}
			if(i == pozyxOUTPUT.size() - (3+3)){//3
				//cout << endl;std::cout << "Roll, pitch, Yaw RATE:" << endl;
			}
			if(i == pozyxOUTPUT.size() - (0+3)){// Pitch
				//cout << endl;std::cout << "Roll, pitch, Yaw:" << endl;//ACTUALLY IS "Pitch, roll, Yaw:" IN SPARKUN !!!
				Eulers[1] = pozyxOUTPUT[i];
			}

			//GET EULER
			if(i == pozyxOUTPUT.size() - (0+2)){// Roll			
				Eulers[0] = pozyxOUTPUT[i];
			}
			if(i == pozyxOUTPUT.size() - (0+1)){// Yaw			
				Eulers[2] = pozyxOUTPUT[i];
			}


			if(i > pozyxOUTPUT.size() - (8+3)){	//8	
				//std::cout << pozyxOUTPUT[i] <<std::endl;
			}
				
			//}
		}					
		//END ARDUINO COM READ


		//ROTATE GRAVITY - 0.96
	//	std::cout << "Eulers:" << Eulers[0] << ", " << Eulers[1] << ", " << Eulers[2] << std::endl; // PITCH - ROLL - YAW   // ROLL - PITCH - YAW
		EulersRAD[0] = Eulers[0] * CV_PI / 180;
		EulersRAD[1] = Eulers[1] * CV_PI / 180;
		EulersRAD[2] = Eulers[2] * CV_PI / 180;
		Mat EulerRotationMatrix = eulerAnglesToRotationMatrix(EulersRAD); //(Eulers);
	//	std::cout << "Euler Rotation Matrix:" << EulerRotationMatrix << std::endl;

		Vec3f GravityVec;
		GravityVec[0] = 0;
		GravityVec[1] = 0;
		GravityVec[2] = 0.994;//0.981;

		//GravityPerAxis =Mat(GravityVec).t() * EulerRotationMatrix ;//GravityPerAxis = EulerRotationMatrix * Mat(GravityVec);
		GravityPerAxis = EulerRotationMatrix * Mat(GravityVec);
		Vec3f GravityPerAxisV = (Vec3f)GravityPerAxis;
		float magnitude = sqrt(GravityPerAxisV[0]*GravityPerAxisV[0] + GravityPerAxisV[1]*GravityPerAxisV[1] + GravityPerAxisV[2]*GravityPerAxisV[2]);
	//	std::cout << "Gravity per axis:" << GravityPerAxis << std::endl;
	//	std::cout << "Gravity per axis magnitude:" << magnitude << std::endl;

		//std::cout << "______________" << endl;

		if(vectMEASUREMENTS.size() > 0){
			POZYXsamplesCounted++;
		}	


				//  1.  ////////////////////////////////////////////// ACCELERATION INTEGRAL //////////////////////////////////// //v0.1
				double timeNow = (double)getTickCount();
				//(double)getTickCount() - timer, panRate
				double timeDiff = (timeNow - prevTimeSPARK) * (1/getTickFrequency());
				accelXRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (6+3)]);
				accelYRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (5+3)]);
				accelZRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (4+3)]);
				Vec3f grav = (Vec3f)GravityPerAxis;
				double accelXGrav = (accelXRate-grav[0]);
				double accelYGrav = (accelYRate-grav[1]);
				double accelZGrav = (accelZRate-grav[2]);
				double accelXGravSUM = 0;
				double accelYGravSUM = 0;
				double accelZGravSUM = 0;
				if(1==1){	

					//BIAS plus side more
					if(accelXGrav < 0){
						//accelXGrav = 2.5 * accelXGrav;
					}
					if(accelXGrav > 0){
						//accelXGrav = 2.75 * accelXGrav;
						accelXGrav = 1 * accelXGrav;
					}
					if(accelYGrav > 0){
						//accelYGrav = 1.1 * accelYGrav;
					}
					if(accelYGrav < 0){
						//accelYGrav = 1.2 * accelYGrav;
					}
					if(accelZGrav < 0){
						//accelZGrav = 2.5 * accelZGrav;
					}	
					
							
					//THRESHOLD low measurements
					float firstThreashold = 0.065;//0.125; //0.75
					if(abs(accelXGrav) > firstThreashold){
						cout << "accel X:" << accelXGrav << endl;
					}else{ accelXGrav=0; }
					if(abs(accelYGrav) > firstThreashold){
						cout << "accel Y:" << accelYGrav << endl;
					}else{ accelYGrav=0; }
					if(abs(accelZGrav) > firstThreashold){
						cout << "accel Z:" << accelZGrav << endl;
					}else{ accelZGrav=0; }					

					//Insert samples in buffer
					bufferAccelX.push_back(accelXGrav);
					bufferAccelY.push_back(accelYGrav);
					bufferAccelZ.push_back(accelZGrav);
					
					//MEAN of measurements
					int samplesCount = 16; //TO DO - define based on data rate
					for (int i=0; i < samplesCount; i++){
						accelXGravSUM += bufferAccelX[bufferAccelX.size()-1-i];
						accelYGravSUM += bufferAccelY[bufferAccelY.size()-1-i];
						accelZGravSUM += bufferAccelZ[bufferAccelZ.size()-1-i];
					}
					accelXGravSUM /= samplesCount;
					accelYGravSUM /= samplesCount;
					accelZGravSUM /= samplesCount;

					//OPTICAL FLOW integration 
					//SINCE WE HAVE DRONE, if gyros show leveled drone and no accelration for some time, we can reset the linear velocity or lower it gradually !!!!!!!!!!!!!!!!!!!!!!
					//MUST also find out which part of sumDX, sumDY is coming from rotational motion

					float cutoffAccelOptical = 0.42; //ranges 0.18 to 1.50 - put  0.12 //0.42
					//cout << "sumD1X:" << sumDX << ", sumD1Y:" << sumDY << endl;
					if(abs(sumDX) < cutoffAccelOptical && abs(sumDY) < cutoffAccelOptical){		// if drone speed near zero, assume no motion
							//cout << "sumDX:" << sumDX << ", sumDY:" << sumDY << endl;
							velocityZGrav = 0;//velocityXGrav = 0; velocityYGrav = 0; velocityZGrav = 0;
					}
					if(abs(sumDX) < cutoffAccelOptical){		// if drone speed near zero, assume no motion
							//cout << "sumDX:" << sumDX << ", sumDY:" << sumDY << endl;
							velocityXGrav = 0;//velocityXGrav = 0; velocityYGrav = 0; velocityZGrav = 0;
					}
					if(abs(sumDY) < cutoffAccelOptical){		// if drone speed near zero, assume no motion
							//cout << "sumDX:" << sumDX << ", sumDY:" << sumDY << endl;
							velocityYGrav = 0;//velocityXGrav = 0; velocityYGrav = 0; velocityZGrav = 0;
					}
					if(abs(accelXGravSUM) < 0.0004){
						accelXGravSUM=sign(accelXGravSUM)*0.000001;
						if(abs(sumDX) < cutoffAccelOptical && abs(sumDY) < cutoffAccelOptical){		// if drone speed near zero, assume no motion
							velocityXGrav = 0;
						}
					}
					if(abs(accelYGravSUM) < 0.0004){
						accelYGravSUM=sign(accelYGravSUM)*0.000001;
						if(abs(sumDX) < cutoffAccelOptical && abs(sumDY) < cutoffAccelOptical){
							velocityYGrav = 0;
						}
					}
					accelZGravSUM = accelZGravSUM - 0.004;//0.002;
					if(abs(accelZGravSUM) < 0.0054){
						accelZGravSUM=sign(accelZGravSUM)*0.000001;
						if(abs(sumDX) < cutoffAccelOptical && abs(sumDY) < cutoffAccelOptical){
							velocityZGrav = 0;
						}
					}

					//update - integrate speed
					//cout << "Sign accel X:" << sign(accelXGravSUM) << endl;cout << "Sign accel Y:" << sign(accelYGravSUM) << endl;cout << "Sign accel Z:" << sign(accelZGravSUM) << endl;
					//cout << "sumDX:" << sumDX << ", sumDY:" << sumDY << endl;
					
					if(	(abs(accelXGravSUM) > 0 && abs(accelXGravSUM) < 140) ||
						(abs(accelYGravSUM) > 0 && abs(accelYGravSUM) < 140) || 
						(abs(accelZGravSUM) > 0 && abs(accelZGravSUM) < 140)
					){
						velocityXGrav = velocityXGrav + accelXGravSUM * timeDiff;
						velocityYGrav = velocityYGrav + accelYGravSUM * timeDiff;
						velocityZGrav = velocityZGrav + accelZGravSUM * timeDiff;

						dXGrav = 0;
						dYGrav = 0;
						dZGrav = 0;
						dXGrav = velocityXGrav * timeDiff * 1111;
						dYGrav = velocityYGrav * timeDiff * 1111;
						dZGrav = velocityZGrav * timeDiff * 1111;

						TRANSLATION_X_Grav += dXGrav*1;
						TRANSLATION_Y_Grav += dYGrav*1;
						TRANSLATION_Z_Grav += dZGrav*1;
					
						if(abs(dXGrav) > 0.005){
							//cout << "dXGrav = " << dXGrav << endl; 
						}
						if(abs(dYGrav) > 0.005){
							//cout << "dYGrav = " << dYGrav << endl; 
						}
						if(abs(dZGrav) > 0.005){
							//cout << "dZGrav = " << dZGrav << endl; 
						}
						//cout << endl; 
						//cout << "accel X:" << accelXGravSUM << endl;cout << "accel Y:" << accelYGravSUM << endl;cout << "accel Z:" << accelZGravSUM << endl;
						//cout << "velocityXGrav = " << velocityXGrav << ", velocityYGrav = " << velocityYGrav  << ", velocityZGrav = " << velocityZGrav << endl; 
						//cout << "dXGrav = " << dXGrav << ", dYGrav = " << dYGrav  << ", dZGrav = " << dZGrav << endl; 
						//cout << "sumDX = " << sumDX << ", sumDY = " << sumDY << endl;
						cout << "X = " << TRANSLATION_X_Grav << endl; cout << "Y = " << TRANSLATION_Y_Grav << endl; cout << "Z = " << TRANSLATION_Z_Grav << endl;
						//cout << "accel X:" << accelXGrav << endl;cout << "accel Y:" << accelYGrav << endl;cout << "accel Z:" << accelZGrav << endl; cout << endl;
						//sumDX, sumDY
					}
				}//END DX_DY_DZ
				prevTimeSPARK = (double)getTickCount();
				//  1 end.  ////////////////////////////////////////////// END ACCELERATION INTEGRAL ////////////////////////////////////

	
	} 	
}
 

///END SPARKFUN handler

///////////////////////////// END PTZ FLOW ADDITION ////////////////










////////////////////////////////// YOLO 4 ////////////////////////////
constexpr auto default_batch_size = 1;


float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };

float get_color(int c, int x, int max)
{
    float ratio = ((float)x/max)*5;
    int i = floor(ratio);
    int j = ceil(ratio);
    ratio -= i;
    float r = (1-ratio) * colors[i][c] + ratio*colors[j][c];
    return r;
}

double get_time_point() {
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    //uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count();
    return std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch()).count();
}


struct config_type {
    std::string name;
    int backend;
    int target;
};

// select backend target combinations that you want to test
//std::vector<config_type> backends = {
   // {"OCV CPU", cv::dnn::DNN_BACKEND_OPENCV, cv::dnn::DNN_TARGET_CPU},
  //  {"OCV OpenCL", cv::dnn::DNN_BACKEND_OPENCV, cv::dnn::DNN_TARGET_OPENCL},
  //  {"OCV OpenCL FP16", cv::dnn::DNN_BACKEND_OPENCV, cv::dnn::DNN_TARGET_OPENCL_FP16},

    //{"IE CPU", cv::dnn::DNN_BACKEND_INFERENCE_ENGINE, cv::dnn::DNN_TARGET_CPU},

    //{"CUDA FP32", cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA},
    //{"CUDA FP16", cv::dnn::DNN_BACKEND_CUDA, cv::dnn::DNN_TARGET_CUDA_FP16}
//};

//float thresh=0.5;//参数设置
 //   float nms=0.35;
 //   int classes=1;
 //auto net = cv::dnn::readNetFromDarknet(cfgfile, weightfile);
//auto output_names = net.getUnconnectedOutLayersNames();
//vector<cv::Mat> detections;
bool foundWindow = false;
bool foundWindowInCropped = false;
bool tryDifferentRes = false;
float fps = 0.0;
Rect prevRect = Rect(0,0,1,1);
Rect prevRectFixed =  Rect(0,0,1,1);
float prevPrevRectX = 0;
Rect currentYoloRect = Rect(0,0,1,1);
float currentScore =0;

bool fullOptimal = true;

void infereDrone(Mat frame, Mat blob, int sizing, vector<string> classNamesVec,  float thresh,  float nms,  int classes, cv::dnn::Net net , std::vector< String > output_names, vector<cv::Mat> detections, int redoing){

	if(redoing == 0){
		YoloA_enter_and_found = 1; YoloA_enter_and_found1++;
	}
	if(redoing == 1){
		YoloB_enter_and_found = 1; YoloB_enter_and_found1++;
	}

	currentScore = 1000;

	foundWindow = false;
	foundWindowInCropped = false;
	Mat resized;
	//int sizing = 224;//124;//224;//416;	
	//resize(frame, resized, cv::Size(sizing,sizing),0,0,INTER_LINEAR);;//Mat resized = resize(frame, sizeof(224, 224));
        cv::dnn::blobFromImage(frame, blob, 1./255., cv::Size(sizing, sizing), cv::Scalar(), true, false, CV_32F);
        net.setInput(blob);

   //     double before = get_time_point();
        net.forward(detections, output_names);
   //     double after  = get_time_point();
        //float fps = 1000000. / (after - before);
//	fps = 1000000. / (after - before);

        std::vector<int> indices[classes];
        std::vector<cv::Rect> boxes[classes];
        std::vector<float> scores[classes];

        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * frame.cols;
                auto y = output.at<float>(i, 1) * frame.rows;
                auto width = output.at<float>(i, 2) * frame.cols;
                auto height = output.at<float>(i, 3) * frame.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);
                //cout << x << y << width << height << num_boxes << output<< endl;
                for (int c = 0; c < classes; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= thresh)
                    {
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }

        for (int c = 0; c < classes; c++){
            cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, nms, indices[c]);   
	}

        for (int c= 0; c < classes; c++)
        {

	    currentScore = indices[c].size();
            for (size_t i = 0; i < indices[c].size(); ++i)
            {
                int offset = 123457 % 80;
                float red = 255*get_color(2,offset,80);
                float green = 255*get_color(1,offset,80);
                float blue = 255*get_color(0,offset,80);
                const auto color = Scalar(blue, green, red);

                auto idx = indices[c][i];
                const auto& rect = boxes[c][idx];

		if(plotDebug){
                	cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);
		}
		foundWindow = true;
		
		if(redoing == 0){
			prevRect = rect;
		}
		
		if(redoing == 1){
			//cout << "FOUND in CROPPED" << rect <<  endl;
			//   cv::rectangle(frame, cv::Point(rect.x+6, rect.y+6), cv::Point(rect.x + rect.width-6, rect.y + rect.height-6), color, 3);
			
			//prevRect = Rect( rect.x + prevRect.x - 4*rect.width/2,  rect.y + prevRect.y - 4*rect.height/2, rect.width * 2, rect.height * 2 ); //recast to original image coordinates before crop
			//prevRect = Rect( rect.x + prevRect.x,  rect.y + prevRect.y, rect.width , rect.height  );
			prevRect = Rect( rect.x + prevRectFixed.x - 0*prevRectFixed.width,  rect.y + prevRectFixed.y - 0*prevRectFixed.height, rect.width , rect.height  );
			foundWindowInCropped = true;
		}

		if(redoing == 2){
			cout << "FOUND in REPEAT" << rect <<  endl;
		}

		if(redoing == 0){
			YoloA_enter_and_found = 2; YoloA_enter_and_found2++;
		}
		if(redoing == 1){
			YoloB_enter_and_found = 2; YoloB_enter_and_found2++;
		}

                std::ostringstream label_ss;
                label_ss << classNamesVec[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];

		currentScore = scores[c][idx];

                auto label = label_ss.str();
                
                int baseline;
                auto label_bg_sz = getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
		if(plotDebug){
				rectangle(frame, Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
				putText(frame, label.c_str(), Point(rect.x, rect.y - baseline - 5), FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
		}

		if(redoing == 1){
			if(plotDebug){			
				imshow("video cropped",frame);
			}
		}
            }
        }
	if(!foundWindow && (redoing == 1 || redoing == 2)){
		if(plotDebug){
			imshow("video FULL cropped",frame);
		}
		if(!fullOptimal){
			prevRect = Rect(0,0,1,1);
		}
	}
}
///////////////////////////////// END YOLO 4 /////////////////////////


//DEFINE GLOBAL TRACKER HERE !!!!
//#define trackerUsed cv::tracking::TrackerMOSSE
//#define trackerUsed TrackerBoosting
//#define trackerUsed TrackerMIL
#define trackerUsed TrackerMOSSE
//#define trackerUsed TrackerTLD
//#define trackerUsed TrackerMedianFlow
//#define trackerUsed TrackerGOTURN
//#define trackerUsed TrackerCSRT











vector<Point2f> dronePointsINSIDEbbox;


int main(int argc, char **argv)
{




	////////////////// YOLO 4 /////////////////////////////

	string cfgfile = "../cfg/yolov4-tiny-3l-drone.cfg";
    //if(argc > 1)
     //   cfgfile = argv[1]; 
    string weightfile = "../weights/yolov4-tiny-3l-drone_last.weights";
    //if(argc > 2)
    //    weightfile = argv[2]; 
    string videopath = argv[1];// "../demo/Thermal2.mp4";
    //if(argc > 3)
    //    videopath = argv[3]; 
    string namefilepath = "../cfg/drone.names";
    //if(argc > 4)
    //    namefilepath = argv[4]; 
    string savepath = "output.avi";
   // if(argc > 5)
     //   savepath = argv[5]; 

    vector<string> classNamesVec;
    ifstream classNamesFile(namefilepath);

    if (classNamesFile.is_open()){
        string className = "";
        while (getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }

    float thresh=0.5;//
    float nms=0.35;
    int classes=1;
    
    auto net = cv::dnn::readNetFromDarknet(cfgfile, weightfile);
    //net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    //net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    auto output_names = net.getUnconnectedOutLayersNames();

    auto net2 = cv::dnn::readNetFromDarknet(cfgfile, weightfile);
    net2.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net2.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    auto output_names2 = net2.getUnconnectedOutLayersNames();

    auto net3 = cv::dnn::readNetFromDarknet(cfgfile, weightfile);
    net3.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net3.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    auto output_names3 = net3.getUnconnectedOutLayersNames();

    VideoCapture capture(videopath);
    int frame_w = capture.get(3);
    int frame_h = capture.get(4);
    //VideoWriter video(savepath, cv::VideoWriter::fourcc('M','J','P','G'),60, Size(frame_w,frame_h));
    //capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    Mat frame, blob;
    vector<cv::Mat> detections;    

    bool stop=false;
    while(1==0 && !stop)
    {
        //cout<<frame.size<<endl;
        if (!capture.read(frame))
        {
            printf("fail to read.\n");
            return 0;
        }

      	double before = get_time_point();
       
  
	
	int sizing = 224;//124;//224;//416;
	Mat copyFrame;
	frame.copyTo(copyFrame);
	Mat copyBlob;
	blob.copyTo(copyBlob);	
	//cout << "prevRect X=" << prevRect.x << endl;
	if(prevRect.x == 0 || !fullOptimal){
		infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net,output_names, detections, 0);// 0 == LOWER RESOLUTION
		//cout << "INININI 111" << endl;
	}


	if(!foundWindow || (prevRect.x > 0 && fullOptimal)){

		//infereDrone(frame, blob, 224, classNamesVec, thresh, nms, classes,net,output_names, detections, 0);// 0 == LOWER RESOLUTION

		if(prevRect.x > 0){

				//cout << "Window not found, using cropped image around previous window" << endl;
				//cout << "prevRect=" << prevRect << endl;

				int ax = prevRect.x  - 1.0*prevRect.width;
				int ay = prevRect.y  - 1.0*prevRect.height;
				int aw = prevRect.width  * 3;
				int ah = prevRect.height * 3;
				if(ax < 0){
					ax = 0;
				}
				if(ay < 0){
					ay = 0;
				}
				if(ax + aw > frame.cols){
					aw = frame.cols - ax;
				}
				if(ay + ah > frame.rows){
					ah = frame.rows - ay;
				}
				//cout << "ax:" << ax << " ay:" << ay << " aw:" << aw << " ah:" << ah << endl;
				Rect cropRect = Rect(ax, ay, aw, ah);
				
				//cout << "w:" << frame.cols << " h:" << frame.rows << " cropRect:" << cropRect << endl;
				detections.clear();
				Mat cropedImage = frame(cropRect);
				Mat copiedIMG; cropedImage.copyTo(copiedIMG);
				infereDrone(copiedIMG, blob, 416, classNamesVec, thresh, nms, classes,net3,output_names3, detections, 1); //1 == CROPPED //224 //416
			
				if(plotDebug){
					imshow("video cropped",copiedIMG);
				}

				float red = 255*get_color(2,1,80);
				float green = 255*get_color(1,1,80);
				float blue = 255*get_color(0,1,80);
				const auto color = Scalar(blue, green, red);

				if(plotDebug){
					cv::rectangle(frame, cv::Point(prevRect.x, prevRect.y), cv::Point(prevRect.x + prevRect.width, prevRect.y + prevRect.height), color/2, 3);
				}

				//LABELS
				std::ostringstream label_ss;
				label_ss << classNamesVec[0] << ": " << std::fixed << std::setprecision(2) << currentScore;
				auto label = label_ss.str();
				int baseline;
				auto label_bg_sz = getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
				if(plotDebug){
					rectangle(frame, Point(prevRect.x, prevRect.y - label_bg_sz.height - baseline - 10), cv::Point(prevRect.x + label_bg_sz.width, prevRect.y), color, cv::FILLED);
					putText(frame, label.c_str(), Point(prevRect.x, prevRect.y - baseline - 5), FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
				}
		}
		if(!foundWindow){
				//cout << "Window not found, using higher resolution" << endl;
				//detections.clear();
				//infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 2); //2 == FULL RESOLUTION
		}

		if(1==0){
			if(prevRect.x > 0){
				cout << "Window not found, using cropped image around previous window" << endl;
				infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 1); //1 == CROPPED
			}
			if(!foundWindow){
				cout << "Window not found, using higher resolution" << endl;
				detections.clear();
				//infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 2); //2 == FULL RESOLUTION
			}
		}
	}


	double after  = get_time_point();
        
	fps = 1000000. / (after - before);

        string fpsString = to_string(fps);
	if(plotDebug){
        	putText(frame, fpsString, Point(50,50), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0),2);

        	imshow("video",frame);
	}

        int c=waitKey(1);
        if((char)c==27)
            break;
        else if(c>=0)
            waitKey(0);
        //video.write(frame);

    }
    //video.release();
   // return 0;

	////////////////// END YOLO 4 /////////////////////////






















	if(argc < 2) {
		cout << "./VideoStab [video.avi]" << endl;
		return 0;
	}

	//cout << "start Frame, downscale factor, draw all rectangles (0,1), use 2ond layer (0,1), use PTZ (0,1), multiply source resolution (640,360)" << endl;
	cout << "Start Frame (offline video)" 	<< endl;
	cout << "Downscale factor" 		<< endl;
	cout << "Draw all rectangles (0,1), use 2ond layer (0,1), use PTZ (0,1), multiply source resolution (640,360)" << endl;
	cout << "Use 2ond layer (0,1), use PTZ (0,1), multiply source resolution (640,360)" << endl;
	cout << "Use PTZ (0,1), multiply source resolution (640,360)" << endl;
	cout << "Multiply source resolution (640,360)" << endl;
	cout << "Cut propellers (Vulkan)" << endl;
	cout << "Lower screen cutoff (pixels)" << endl;
	cout << "Debug mode on(1) - off(0)" << endl;

	waitKey(1000);

	int startFrame = atoi(argv[2]);	
	float scaleFrame = atof(argv[3]);
	drawAllRectangles = atoi(argv[4]);
	bool use2ondBackLayerINPUT = false;
	if(atoi(argv[5]) == 1){
		use2ondBackLayerINPUT = true;
	}		
	if(atoi(argv[6]) == 1){
		usePTZ = true;
	}
	
	
	float resMultiplier = atof(argv[7]);

	if(atoi(argv[8]) == 1){
		cutPropellers = true;
	}

	float lowerCutoffPixels = 0;
	if(argc > 7){
		lowerCutoffPixels = atoi(argv[9]);
		cout << "set lower screen cutoff to " << lowerCutoffPixels << endl; 
	}

	//int plotDebugEnabled = 0;
	if(argc > 8){
		plotDebugEnabled = atoi(argv[10]);
		if(plotDebugEnabled == 1){
			cout << "Debug mode ON " << endl; 
		}else{
			cout << "Debug mode OFF " << endl; 
		}
	}else{
			cout << "Debug mode ON " << endl; 
	}

	float scaler = scaleFrame;//0.85;//0.85;
	VideoCapture cap;
	if(!usePTZ){
		cap.open(argv[1]);
	}else{
		cap.open("/dev/v4l/by-id/usb-2e7e_PTZ_Optics_Camera-video-index0");
		// Exit if video is not opened
		if(!cap.isOpened())
		{
			cout << "Could not read video file" << endl; 
			return 1; 
		} 
	}


	
	



	//cap.set(3,1920);
  	//cap.set(4,1080);
	//cap.set(3,1280);
  	//cap.set(4,720);
	cap.set(3,640*resMultiplier);
  	cap.set(4,360*resMultiplier);

	//ROS
	ros::init(argc, argv, "arrayflowPublisher");
	ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("arrayflow", 100);
	ros::Publisher pub = n.advertise<std_msgs::UInt32>("arrayflow", 1);
	ros::Subscriber subGPSPositionGet = n.subscribe("/mavros/global_position/raw/fix", 1, getGpsPositionCallback);//gpsPos
	ros::Subscriber subGPSTimeGet = n.subscribe("/mavros/time_reference", 1, getGpsTimeCallback);//gpsTime

	ros::Subscriber local_position = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPose_cb);

	//ros::Subscriber GPS_raw = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 1, GPSraw_cb);
	//ros::Subscriber GPS_global = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, GPS_cb);
	//ros::Subscriber local_position = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPose_cb);
	//ros::Subscriber current_time = nh.subscribe<sensor_msgs::TimeReference>("/mavros/time_reference", 1, time_reference_cb);

	//ros::spin();
	//END ROS

	//WRITE OUTPUT VIDEO
	int frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
  	int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT); 

	std::string fileNameVIDEO = "outVIDEO_" + GetCurrentTimeForFileName() + ".avi";
	//VideoWriter video(fileNameVIDEO,cv::VideoWriter::fourcc('M','J','P','G'),30, Size(frame_width*scaler,frame_height*scaler));

	cout << "S_WIDTH = " << frame_width << endl;
	cout << "S_H ==== " << frame_height << endl;

	//VideoWriter video(fileNameVIDEO,cv::VideoWriter::fourcc('M','J','P','G'),30, Size(1280,720));
	VideoWriter video(fileNameVIDEO,cv::VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height));
	//VideoWriter video;//("outcpp.avi",CV_FOURCC('M','J','P','G'),30, Size(frame_width*scaler,frame_height*scaler));
	cout << "frame_width:" << frame_width << " frame_height:" << frame_height << endl;
	
	assert(cap.isOpened());

	Mat cur, cur_grey;
	Mat prev, prev_grey;

	//cap >> prev;//get the first frame.ch
	
	Mat temp1; cap >> temp1;
	cout << temp1.rows << " rows" << endl;
	if(scaler != 1){
		cv::resize(temp1, temp1, cv::Size(), scaler, scaler);
	}	
	temp1.copyTo(prev);

	cvtColor(prev, prev_grey, COLOR_BGR2GRAY);	
	
	int k=1;
	int max_frames = cap.get(CAP_PROP_FRAME_COUNT);

	//NASOS
	//int frameID=0;	
	double prevTime = (double)getTickCount();
	const bool useGpuFeaturesTrack = true;

	//MOSSE
	//string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"}; 
    	// Create a tracker
    	//string trackerType = trackerTypes[6];
	//Ptr<Tracker> tracker;
 	
        //if (trackerType == "MOSSE"){
        //    	tracker = cv::TrackerCSRT::create();	    	
	//}
        //if (trackerType == "CSRT"){
        //    	tracker = cv::TrackerCSRT::create();	    	
	//}







	//YOLO 4
	// List of tracker types in OpenCV 3.4.1
	string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
	
	// Create a tracker
	string trackerType = trackerTypes[6]; //2
 
	Ptr<Tracker> tracker;
	//Ptr<Tracker> tracker2;	

	cout <<  CV_MINOR_VERSION << "," <<CV_MAJOR_VERSION << endl;

	#if (CV_MINOR_VERSION < 3)
	{
		tracker = Tracker::create(trackerType);
		//tracker2 = Tracker::create(trackerType);
	}
	#else
	{	
		tracker = trackerUsed::create();//Ptr<Tracker> tracker;
		//tracker2 = trackerUsed::create();
	}
	#endif   











	tracker->init(prev, bbox2);
	bool okB = false;

	Mat homog;

//./track ./RESULTS1/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 0 1 2 1
//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 40 1 0 1 0 1 1

	
	bool runExperiment = false; //false , true
	bool plotHomography = true; 	bool plotHomographyTEXT = false;
	bool plotAllPredictedFlowVectors = true;
	bool displayall = true;

	bool logGPS = false;
	bool showFeedPreview = true; 
	bool plotFromFile = false;
	bool plotFromVikon = false;
//	bool plotDebug = true; //true //false
	bool plotDebugTracking = true;
	//bool offlinePTZ = true; //enable when reading PTZ from file to cut back propellers, replaced with usePTZ
	//bool cutPropellers = true; //cut propellers regions from calculations, offline or online

	if(plotDebugEnabled == 0){
		showFeedPreview = false; 		
		plotDebug = false; 
		plotDebugTracking = false;
	}

	//bool plotDebugTrackingFUSION = true;


//READ FILE FOR DEBUG PUROSES
	//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 0 1
	//READ WINDOW COORDINATES FROM TEXT FILE
	//ifstream inputfile("videos/in1a.log");
	//ifstream inputfile("RESULTS/Tracking_Result_2020-01-14_14-48-38.csv"); //2372
		
	string get_file_name = argv[1];

	size_t lastindex = get_file_name.find_last_of("."); 
	string rawname = get_file_name.substr(0, lastindex); 

	rawname = rawname + ".csv";

	cout << "opening file:" << rawname << endl;

	ifstream inputfile(rawname); //2372
	vector<double> pointsCSVx;//mosse
	vector<double> pointsCSVy;
	vector<double> pointsCSVw;
	vector<double> pointsCSVh;

	vector<double> pointsCSVxM;//measurement
	vector<double> pointsCSVyM;
	vector<double> pointsCSVwM;
	vector<double> pointsCSVhM;

	vector<int> cameraPan;
	vector<int> cameraTilt;
	vector<int> cameraZoom;

	vector<double> rollRates;
	vector<double> pitchRates;
	vector<double> yawRates;
		
	vector<double> FClinearVelocityXs;
	vector<double> FClinearVelocityYs;
	vector<double> FClinearVelocityZs;

	//vector<double> frameTimes;
	int countReads= 0;

	if(!usePTZ){//if(plotFromFile || plotFromVikon){
		cout << "read file ok" << endl;

		string current_line;
		// vector allows you to add data without knowing the exact size beforehand
		vector< vector<int> > all_data;
		// Start reading lines as long as there are lines in the file
		while(getline(inputfile, current_line)){
		   // Now inside each line we need to seperate the cols
		   vector<int> values;
		   stringstream temp(current_line);
		   string single_value;
		   while(getline(temp,single_value,';')){
			// convert the string element to a integer value
			values.push_back(atoi(single_value.c_str()));
		   }
		   // add the row to the complete data vector
		   all_data.push_back(values);
	//	   cout << "OUT=" << values[5] << endl; //5,6,7,8

		   //0 id, mosse 1-2-3-4, measure 5-6-7-8, pan-tilt-zoom

		   int frameID = values[0];
 		   double timeRecorded = 0;//values[1];

			bool beforeAddTime = true;//false;
			if(beforeAddTime){
				   // timeRecorded = values[1];
				   cameraPan.push_back(values[9]);
				   cameraTilt.push_back(values[10]);
				   cameraZoom.push_back(values[11]);

			  	   pointsCSVx.push_back(values[1]);
				   pointsCSVy.push_back(values[2]);
				   pointsCSVw.push_back(values[3]);
				   pointsCSVh.push_back(values[4]);

				   pointsCSVxM.push_back(values[5]);
				   pointsCSVyM.push_back(values[6]);
				   pointsCSVwM.push_back(values[7]);
				   pointsCSVhM.push_back(values[8]);
			}else{
				   timeRecorded = values[1];
				   cameraPan.push_back(values[10]);
				   cameraTilt.push_back(values[11]);
				   cameraZoom.push_back(values[12]);

			  	   pointsCSVx.push_back(values[2]);
				   pointsCSVy.push_back(values[3]);
				   pointsCSVw.push_back(values[4]);
				   pointsCSVh.push_back(values[5]);

				   pointsCSVxM.push_back(values[6]);
				   pointsCSVyM.push_back(values[7]);
				   pointsCSVwM.push_back(values[8]);
				   pointsCSVhM.push_back(values[9]);
			}
		  
		   //IF ROLL-PITCH-YAW DATA EXIST
		   //	12 - Yaw
		   //	13 - Pitch
		   //	14 - Roll
		   //YawRate =;
		   //pitchRate =;
		   //rollRate =;

		//./track ./vids2/2/outVIDEO_2020-04-02_15-21-03.avi 1 1 0 1 0 1 1 0 0
		//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 0 1 0 0 1
		//./track ./vids3/outVIDEO_2020-04-06_17-12-32.avi 1 1 0 1 0 1 1 0 1

		   //IF ROLL-PITCH-YAW DATA NOT EXIST calculate		  
		   if(1==0 && countReads > 0){
			float timeDelta = 1.0/30.0;//assume 30fps since if no time data saved

			float currentYaw = cameraPan[cameraPan.size()-1];
			float prevYaw = cameraPan[cameraPan.size()-2];			
			float YawRate = (currentYaw-prevYaw) / timeDelta;
		   	yawRates.push_back(YawRate);

			float currentPitch = cameraTilt[cameraTilt.size()-1];
			float prevPitch = cameraTilt[cameraTilt.size()-2];			
			float PitchRate = (currentPitch-prevPitch) / timeDelta;
		   	pitchRates.push_back(PitchRate);

			cout << "YawRate=" << YawRate << ", PitchRate=" << PitchRate << endl;
		   }else{
			//if !usePTZ, provide rates by saved file, if usePTZ pass from subscribed topic by flight conroller
			yawRates.push_back(values[13]);
			pitchRates.push_back(values[14]);
			rollRates.push_back(values[15]);

			bool has_recorded_flight_Controller_data = false;
			if(has_recorded_flight_Controller_data){
				FClinearVelocityXs.push_back(values[16]);
				FClinearVelocityYs.push_back(values[17]);
				FClinearVelocityZs.push_back(values[18]);
			}
		   }

		   countReads++;
			
		}	
		//END READ WINDOW COORDINATES FROM TEXT FILE
	}


	//PLOT VIKON DATA
	//vector<Mat> VulkanRot;
	//vector<Mat> MavicRot;
	vector<Point2f> MavicProjectedPos;
	vector<int> frameIDsVikon;
	int frames_back_sync =  17;//-1;//-19; //synch to real video due to lag
	if(plotFromVikon && !usePTZ){

		//AXIS

		//VULKAN
		//X:Forward
		//Y:Left
		//Z:Up

		//MAVIC PRO
		//X:
		//Y:
		//Z:

		//CAMERA BASE TRANSLATION
		//X:4cm
		//Y:0cm
		//Z:16cm

		//cout << "test 00" << endl;
		//ifstream inputfileV("vikon/Vicon_y20m1d14h14min35.csv");//Vicon_y20m1d14h14min48
		//ifstream inputfileV("vikon/Vicon_y20m1d14h14min48.csv"); //2368
	//ifstream inputfileV("vikon/myfile1FLIGHT_CONTROLLER_and_Vicon_DATA_EXP2_FUSION.csv"); //2368
		ifstream inputfileV("vikon/Vicon_y20m1d14h14min48.csv"); //2368
		
		//cout << "test 0" << endl;
		string current_lineV;
		// vector allows you to add data without knowing the exact size beforehand
		vector< vector<int> > all_data;
		// Start reading lines as long as there are lines in the file

		int RealFrameCounter = -1; //count frames covered
		
		while(getline(inputfileV, current_lineV)){
			// Now inside each line we need to seperate the cols
			vector<float> values;
			stringstream temp(current_lineV);
			string single_value;
			//cout << "test 1" << endl;
			int countPlace = 0;
			while(getline(temp,single_value,',')){ //';'
				// convert the string element to a integer value
				values.push_back(atof(single_value.c_str()));
				if(1==0){
					if(countPlace==0){
						cout << "Time:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==1){
						cout << "Vulkan X:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==2){
						cout << "Vulkan Y:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==3){
						cout << "Vulkan Z:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==4){
						cout << "Vulkan QW:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==5){
						cout << "Vulkan QX:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==6){
						cout << "Vulkan QY:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==7){
						cout << "Vulkan QZ:" << atof(single_value.c_str()) << endl;
					}

					if(countPlace==8){
						cout << "Mavic Pro X:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==9){
						cout << "Mavic Pro Y:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==10){
						cout << "Mavic Pro Z:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==11){
						cout << "Mavic Pro QW:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==12){
						cout << "Mavic Pro QX:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==13){
						cout << "Mavic Pro QY:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==14){
						cout << "Mavic Pro QZ:" << atof(single_value.c_str()) << endl;
					}
					if(countPlace==15){
						cout << "Frame ID:" << atof(single_value.c_str()) << endl;
					}
				}
				countPlace++;
			}
			//cout << "test 11" << endl;
			//take time, vulkan x,y,z, rotation, Mavic x,y,z and corresponding frameID
			// Calculate rotation about z axis
			//Mat R_z = (Mat_<float>(3,3) <<
			//	       cos(theta[2]),    -sin(theta[2]),      0,
			//	       sin(theta[2]),    cos(theta[2]),       0,
			//	       0,               0,                  1); 	     
			// Combined rotation matrix
			//Mat R = R_z * R_y * R_x;
			//Vec3f Eulers;
			//Eulers[0] = Eulers[0] * CV_PI / 180;
			//Eulers[1] = Eulers[1] * CV_PI / 180;
			//Eulers[2] = Eulers[2] * CV_PI / 180;
			//Mat EulerRotationMatrix = eulerAnglesToRotationMatrix(Eulers);
			//std::cout << "Euler Rotation Matrix:" << EulerRotationMatrix << std::endl;
			//Vec3f GravityVec;
			//GravityVec[0] = 0;
			//GravityVec[1] = 0;
			//GravityVec[2] = 0.981;
			//GravityPerAxis =Mat(GravityVec).t() * EulerRotationMatrix ;//GravityPerAxis = EulerRotationMatrix * Mat(GravityVec);
			//Vec3f GravityPerAxisV = (Vec3f)GravityPerAxis;
			//float magnitude = sqrt(GravityPerAxisV[0]*GravityPerAxisV[0] + GravityPerAxisV[1]*GravityPerAxisV[1] + GravityPerAxisV[2]*GravityPerAxisV[2]);
	
			//Mat eulerAnglesToRotationMatrix(Vec3f &theta)
			//void multiplyQuaternion(const Mat& q1,const Mat& q2, Mat& q)
			//void rotate_vector_by_quaternion(const Vec3f& v, const Mat& q, Vec3f& vprime)

			//cout << "test 111" << endl;

			int frameIDPosition = 1; //new files
			frameIDPosition = 15;// OLD FILE


		   //ID CHECK - get first frame and if later have same ID discard
		   //if(frameIDsVikon.size() == 0 || frameIDsVikon[frameIDsVikon.size()-1] != values[1]){ 
		   //if(values[1] == RealFrameCounter){
		   if(frameIDsVikon.size() == 0 || frameIDsVikon[frameIDsVikon.size()-1] != values[frameIDPosition]){ 

			int valuesDiff = values[frameIDPosition] - RealFrameCounter;
			if(1==1 && valuesDiff > 1){ // values[1] != RealFrameCounter){				
				for (int i = 0 ;i < int(valuesDiff) - 1; i++){
					//IF ID not found add previous (or inerpolate TO DO)
					if(frameIDsVikon.size() > 0){			
						MavicProjectedPos.push_back(MavicProjectedPos[MavicProjectedPos.size()-1]);
						FClinearVelocityXs.push_back(FClinearVelocityXs[FClinearVelocityXs.size()-1]); //Vulkan Y( left ) to camera -X (left, +X is right)
						FClinearVelocityYs.push_back(FClinearVelocityXs[FClinearVelocityYs.size()-1]); //Vulkan Z(up) to camera -Y (up - +Y is down)
						FClinearVelocityZs.push_back(FClinearVelocityXs[FClinearVelocityZs.size()-1]); //Vulkan X(forward) to camera Z (forward)
					}else{
						MavicProjectedPos.push_back(Point2f(0,0));
						FClinearVelocityXs.push_back(0); 
						FClinearVelocityYs.push_back(0); 
						FClinearVelocityZs.push_back(0);
					}
					RealFrameCounter++;
					frameIDsVikon.push_back(RealFrameCounter);
					
				}
			}

			RealFrameCounter++;
			frameIDsVikon.push_back(RealFrameCounter);
						

			//frameIDsVikon.push_back(values[1]);
			float diff = 0;
			if(frameIDsVikon.size() > 0){
				diff = values[frameIDPosition] - frameIDsVikon[frameIDsVikon.size()-2];
			}
	//		cout << "Frame ID:" <<  frameIDsVikon[frameIDsVikon.size()-1] << " diff=" << diff << endl;


			//LOAD LINEAR VELOCITIES
			FClinearVelocityXs.push_back(-values[24]); //Vulkan Y( left ) to camera -X (left, +X is right)
			FClinearVelocityYs.push_back(-values[25]); //Vulkan Z(up) to camera -Y (up - +Y is down)
			FClinearVelocityZs.push_back(values[23]);  //Vulkan X(forward) to camera Z (forward)
			

		    	// Vulkan quaternion quatVulkan (x1 y1 z1 r1)
		    	//Mat quatVulkan;
		    	//quatVulkan.at<float>(0) = values[5];
		    	//quatVulkan.at<float>(1)	= values[6];
		    	//quatVulkan.at<float>(2)	= values[7];
		    	//quatVulkan.at<float>(3)	= values[4];
			// Mavic quaternion quatMavic (x1 y1 z1 r1)
		    	//Mat quatMavic;
		    	//quatMavic.at<float>(0)	= values[12];
		    	//quatMavic.at<float>(1)	= values[13];
		    	//quatMavic.at<float>(2)	= values[14];
		    	//quatMavic.at<float>(3)	= values[11];

			//Vec3f posVulkan(q.at<float>(0), q.at<float>(1), q.at<float>(2)); //x,y,z,w = 0,1,2,3
			//Vec3f posVulkan(values[1],values[2],values[3]); //x,y,z,w = 0,1,2,3
			//Vec3f posMavic(values[8],values[9],values[10]); 

			//ROTATE - TRANSLATE based on Vulkan position and Rotation, then translate to camera axis, then project based on pan-tilt-zoom values 
			//MatrixXd posVulkan(1,3);
			//posVulkan(0,0) = values[1];
			//posVulkan(0,1) = values[2];
			//posVulkan(0,2) = values[3];	
			//MatrixXd posMavic(1,3);
			//posMavic(0,0) = values[8];
			//posMavic(0,1) = values[9];
			//posMavic(0,2) = values[10];

			//cout << "test 1111" << endl;				
			
			Eigen::Vector3f posVulkan(values[1],values[2],values[3]); 	
			Eigen::Vector3f posMavic(values[8],values[9],values[10]);	

//			Eigen::Vector3f posVulkan(values[2],values[3],values[4]); 	
//			Eigen::Vector3f posMavic(values[9],values[10],values[11]);			

			//cout << "test Vector " << posMavic << endl;

			//Quaternion (const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z)
			Eigen::Quaternionf VulkanRot(values[4],values[5],values[6],values[7]);
			Eigen::Quaternionf MavicRot(values[11],values[12],values[13],values[14]);
//			Eigen::Quaternionf VulkanRot(values[5],values[6],values[7],values[8]);
//			Eigen::Quaternionf MavicRot(values[12],values[13],values[14],values[15]);
			Eigen::Matrix3f VulkanRotM = VulkanRot.toRotationMatrix();
			Eigen::Matrix3f MavicRotM = MavicRot.toRotationMatrix();

			Eigen::Vector3f testRotation;
			testRotation = VulkanRotM * posVulkan;
			//	cout << "test Vector " << testRotation << endl;
			//MatrixXd testRotation;
			//testRotation = VulkanRot * posVulkan;
			//cout << "testRotation" << testRotation << endl;

			/*
			Transform t;

			Translation	
			t.translate(Vector_(tx,ty,..));
			t.pretranslate(Vector_(tx,ty,..));
	
			t *= Translation_(tx,ty,..);
			t = Translation_(tx,ty,..) * t;

			Rotation
			In 2D and for the procedural API, any_rotation can also be an angle in radian	

			t.rotate(any_rotation);
			t.prerotate(any_rotation);
	
			t *= any_rotation;
			t = any_rotation * t;
			*/

			//TO CAMERA from Vulkan axis
			//CAMERA BASE TRANSLATION
			//X:4cm
			//Y:0cm
			//Z:16cm
			Eigen::Vector3f posCamera(0.04,0.0,0.16);//0.16
			//Eigen::Quaternionf toCameraRotA(values[4],values[5],values[6],values[7]);//-90 around the Z up axis of Vulkan
			//Eigen::Quaternionf toCameraRotB(values[4],values[5],values[6],values[7]);//-90 around the new X axis
			float angleRot = -90 * (CV_PI/180);
			Eigen::Matrix3f toCameraRotA;
			toCameraRotA << cos(angleRot), -sin(angleRot), 0,
			     		sin(angleRot),  cos(angleRot), 0,
			     		0, 0, 1;
		//	std::cout << "toCameraRotA = " << toCameraRotA << endl;
		//	std::cout << endl;
			Eigen::Matrix3f toCameraRotB;
			toCameraRotB << 1, 0, 0,
					0, cos(angleRot), -sin(angleRot),
			     		0, sin(angleRot),  cos(angleRot);
		//	std::cout << "toCameraRotB = " << toCameraRotB << endl;

			//PTZ
			float angleZOOM = cameraZoom[frameIDsVikon[frameIDsVikon.size()-1]];
			//float maxZoom = 16384;
			float correctPanTiltByZoomF = angleZOOM/maxZoom;
			float adjustedF = 0.00442 + correctPanTiltByZoomF * (0.0885-0.00442) * 0.1; //Zoom is 0 to 16384, resize F by the zoom factor
			//corrected with non linear curve
			correctPanTiltByZoomF = 1.06 + 0.12 * exp(angleZOOM * 3.18 * pow(10,-4));// 3x zoom means focal lenght max / min = 3	
			adjustedF = correctPanTiltByZoomF * (0.00442/1.18);
 			//adjustedF = 0.00442;
			//Pan is 0 at home. Right is positive, max 2448. Left ranges from full left 63088 to 65535 before home.
			//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully depressed at 65104 to 65535 before home. 
			//Horizontal angle of view PTZ 3.36 to 60.7 degrees
			//Vertical angle of view PTZ 1.89 to 34.1 degrees
			//Pan range +-170 degrees
			//Tilt range  -30 to +90 degrees

			Eigen::Matrix3f toCameraRotPAN;//right pan is positive, rotation aound Y axis
			float anglePAN = cameraPan[frameIDsVikon[frameIDsVikon.size()-1]];// 0;//FIND PAN CORRESPONDING TO HIS FRAME ID FROM OUR LOGS !!!!! 	//cameraPan
			if(anglePAN >= 0 && anglePAN <= 2448){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				anglePAN = 170*(anglePAN/2448);
			}  
			if(anglePAN >= 63088 && anglePAN <= 65535){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				anglePAN = -1*(170-170*( (anglePAN-63088)/ (65535-63088) ));
			} 
			//anglePAN = -45;  
			//convert to rad
			anglePAN = anglePAN * (CV_PI/180);
			//anglePAN = 0;
			toCameraRotPAN << cos(anglePAN), 0, sin(anglePAN),
					0, 1, 0,
			     		-sin(anglePAN), 0, cos(anglePAN);
		//	std::cout << "toCameraRotPAN = " << toCameraRotPAN << endl;

			Eigen::Matrix3f toCameraRotTILT;//up tilt is positive, rotation aound X axis
			float angleTILT = cameraTilt[frameIDsVikon[frameIDsVikon.size()-1]];//0;//FIND TILT CORRESPONDING TO HIS FRAME ID FROM OUR LOGS !!!!!	//cameraTilt
			if(angleTILT >= 0 && angleTILT <= 1296){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				angleTILT = 90*(angleTILT/1296);
			}  
			if(angleTILT >= 65104 && angleTILT <= 65535){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				angleTILT = -(30-30*( (angleTILT-65104)/ (65535-65104) ));
			} 
			//convert to rad
			angleTILT = angleTILT * (CV_PI/180);
			//angleTILT = 0;
			toCameraRotTILT << 1, 0, 0,
					0, cos(angleTILT), -sin(angleTILT),
			     		0, sin(angleTILT),  cos(angleTILT);
		//	std::cout << "toCameraRotTILT = " << toCameraRotTILT << endl;

			//Create tranformation matrix
			//Transform transformToCameraAxis = AngleAxis(0, Vector3f(1,1,1));
			//transformToCameraAxis *=Translation_(1,2,3);
			//transformToCameraAxis.translate(posVulkan);
			//Transform<float,3,Affine> transformToCameraAxis = Translation3f(posVulkan);//* AngleAxisf(a,axis) * Scaling(s);

			//Eigen::Translation<float,3> ToposVulkan = Eigen::Translation<float,3>(1, 1, 1);
			//cout << endl;cout << endl;
			//Transform<float,3,0> transformToCameraAxis;
			//transformToCameraAxis.translate(posVulkan);
			//Eigen::Vector3f posMavicCameraSpace = transformToCameraAxis * posMavic;
			//cout << "posMavicCameraSpace:" << posMavicCameraSpace << endl;
			//cout << endl;cout << endl;
		//	cout << endl;
			//Eigen::Vector3f posMavicCameraSpace;
			//posMavicCameraSpace = VulkanRotM * posMavic;
			//posMavicCameraSpace = posMavic;
			//posMavicCameraSpace += posVulkan;
			//posMavicCameraSpace = VulkanRotM * posMavic;
			//cout << "posMavicCameraSpace = " << posMavicCameraSpace << endl;
			//cout << endl;
			//Eigen::TransformTraits traits=Eigen::Affine;
			
			//https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
			//Transform<float,3,Eigen::Isometry>  transformToCameraAxis; //Isometry, Affine
			//transformToCameraAxis.translate(posVulkan);
			//cout << "transformToCameraAxis = " << transformToCameraAxis << endl;
			//Eigen::Vector3f posMavicCameraSpace = transformToCameraAxis * posMavic;
			//Eigen::Vector3f posMavicCameraSpace = transformToCameraAxis.inverse() * posMavic;
			//cout << "posMavicCameraSpace = " << posMavicCameraSpace << endl;

			Eigen::Matrix3f IdentityM;
			IdentityM << 1, 0, 0,
			     	     0, 1, 0,
			     	     0, 0, 1;

			//Eigen::Matrix3f R;
			// Find your Rotation Matrix
			//Eigen::Vector3f T;
			// Find your translation Vector
			Eigen::Matrix4f TransToVulkan; // Your Transformation Matrix
			TransToVulkan.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
			TransToVulkan.block<3,3>(0,0) = VulkanRotM;// VulkanRotM;
			TransToVulkan.block<3,1>(0,3) = posVulkan;

			Eigen::Matrix4f TransToCamera; // Your Transformation Matrix
			TransToCamera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
			TransToCamera.block<3,3>(0,0) = toCameraRotA*toCameraRotB;
			TransToCamera.block<3,1>(0,3) = posCamera;

			Eigen::Matrix4f RotPanTiltCamera; // Your Transformation Matrix
			RotPanTiltCamera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
			RotPanTiltCamera.block<3,3>(0,0) = toCameraRotPAN * toCameraRotTILT;//toCameraRotTILT*toCameraRotPAN;
			Eigen::Vector3f posCameraZERO(0.0,0.0,0.0);
			RotPanTiltCamera.block<3,1>(0,3) = posCameraZERO;
			
			//ADD ZOOM as translation across z camera axis //cameraZoom

			//FULL TRANSFORMATION
			Eigen::Vector4f posMavic4Dims(posMavic(0), posMavic(1), posMavic(2), 1);//Eigen::Vector4f posMavic4Dims(values[8],values[9],values[10],1);
			Eigen::Matrix4f TransToPoint = TransToVulkan * TransToCamera * RotPanTiltCamera;//* RotPanTiltCamera; 
			//Eigen::Vector4f posMavicCameraSpace = TransToPoint * posMavic4Dims;
			Eigen::Vector4f posMavicCameraSpace = TransToPoint.inverse() * posMavic4Dims;
		//	cout << "posMavicCameraSpace = " << posMavicCameraSpace << endl;

			
			//X' = X * (adjustedF/Z)
			//Y' = Y * (adjustedF/Z)
			//estX = estX + ((0.001 * 5.37) / 2);
			//estY = estY + ((0.001 * 4.04) / 2);					
			//float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
			//float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);	
			//vector<Point> MavicProjectedPos;
			Point2f addMavicPoint((posMavicCameraSpace(0) * adjustedF) / posMavicCameraSpace(2) , (posMavicCameraSpace(1) * adjustedF) / posMavicCameraSpace(2)  );
			addMavicPoint.x = addMavicPoint.x + ((0.001 * 5.37) / 2);
			addMavicPoint.y = addMavicPoint.y + ((0.001 * 4.04) / 2);
			addMavicPoint.x = addMavicPoint.x * prev.cols * scaler / (0.001 * 5.37);
			addMavicPoint.y = addMavicPoint.y * prev.rows * scaler  / (0.001 * 4.04);
			MavicProjectedPos.push_back(addMavicPoint);

			if(diff > 1){
				//for(int i=0;i< int(diff)-1 ;i++)
				//MavicProjectedPos.push_back(addMavicPoint);
			}

		//	cout << endl;

		    }//END ID CHECK
		  //}else{
			//IF ID not found add previous (or inerpolate TO DO)			
			//MavicProjectedPos.push_back(MavicProjectedPos[MavicProjectedPos.size()-1]);

			//FClinearVelocityXs.push_back(FClinearVelocityXs[FClinearVelocityXs.size()-1]); //Vulkan Y( left ) to camera -X (left, +X is right)
			//FClinearVelocityYs.push_back(FClinearVelocityXs[FClinearVelocityYs.size()-1]); //Vulkan Z(up) to camera -Y (up - +Y is down)
			//FClinearVelocityZs.push_back(FClinearVelocityXs[FClinearVelocityZs.size()-1]); //Vulkan X(forward) to camera Z (forward)
		  //}
			////// Create polygons to exclude for left and right propellers - motors
			
			
	
			//////

		}//END WHILE LOOP
		cout << "POINTS VIKON COUNT = " << MavicProjectedPos.size() << endl;
	}
	///END PLOT VIKON DATA

//
	//Video1,2,3.avi
	bool use2ondBackLayer = use2ondBackLayerINPUT;//true;
	int ignorePixelsNearEdge = 20;
	int cornersMax = 480 * scaler; //300
	float angleMin = 14;//5;	//109; use high for POV videos
	float angleMax = 360-angleMin;//355;	//300;
	float minDistPTZ_FLOW = 1;
	float minDistPTZ_FLOW2 = 2;
	float maxDistPTZ_FLOW = 32; //42
	float diffEstimatesToPrevMAX = 27;//14
	float diffEstimatesToPrevMIN = 1;
	float minVelocityMagnitude = 1;
	float minDistInitMOSSE = 85 * scaler; //40 Init mosse if previous windows centers wintin this distance
	float minDisttoCurrentMOSSE = 80 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
	float minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
	
	float lowerCutoff = lowerCutoffPixels * scaler;// 150 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
	int maxContours = 56; //if too many contours, do not try find best one
	float dotPlotScaler = 1;

	float vectorLength = 4; //5
	float minFeatureDistance = 5* scaler; //10
	float cornerQuality = 0.03;//0.04;

	//RUN MULTI TARGET SAMPLE
	//./track ./vids/Tube1.mp4 1 0.95 1 0 0 2

	//RUN REAL TIME PTZ
	//./track ./vids/video1.avi 1 1 0 1 1 2

	// ./track ./vids/video6b.mp4 1 0.45 0 1 0 1 0
	// ./track ./vids/Tube1.mp4 1 0.95 1 0 0 1 0
	// ./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 0 1 1

	//IMPROVE ON 2ond EXPERIMENT
	//./track ./outVIDEO_2020-02-10_15-47-38.avi 40 1 0 1 0 1 0
	//./track ./outVIDEO_2020-02-10_15-42-29.avi 40 1 0 1 0 1 0
	//./track ./outVIDEO_2020-02-10_15-37-52.avi 800 1 0 1 0 1 0
	//./track ./outVIDEO_2020-02-10_15-28-31.avi 400 1 0 1 0 1 0
	//./track ./outVIDEO_2020-02-10_14-53-25.avi 911 1 0 1 0 1 0
	
	//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 1 0 1 1 1 1 0 1
	
	//./track ./vids/Tube1.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube2.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube3.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube4.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube5.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube6.mp4 1 0.95 1 0 0 1 0 0 1
	//./track ./vids/Tube7.mp4 1 0.95 1 0 0 1 0 0 1

	//./track ./vids/video1.avi 1 0.45 0 1 0 1 0 150 1
	//./track ./vids/video2.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video3.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video4a.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video4b.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video4c.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video5.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video5a.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video5b.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video6.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video6a.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/video6b.mp4 1 0.45 0 1 0 1 0 0 1

	//./track ./vids/v7a.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/v7b.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/v7c.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/v7e.mp4 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/v7g.mp4 1 0.45 0 1 0 1 0 0 1

	//./track ./vids/DJI_0218.MP4 1 0.7 0 1 0 1 0 0 1
	//./track ./vids/DJI_0219.MP4 1 0.7 0 1 0 1 0 0 1
	//./track ./vids/DJI_0220.MP4 1 0.7 0 1 0 1 0 0 1
	//./track ./vids/DJI_0221.MP4 1 0.7 0 1 0 1 0 0 1

	//./track ./vids/exp2a1.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/exp2a2.avi 1 0.45 0 1 0 1 0 0 1
	//./track ./vids/exp2a3.avi 1 0.45 0 1 0 1 0 0 1
	
	//./track ./RESULTS/outVIDEO_2020-01-14_14-35-09.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-35-34.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-35-58.avi 0 1 0 0 0 1 1 0 1 GOOD
	//./track ./RESULTS/outVIDEO_2020-01-14_14-39-45.avi 0 1 0 0 0 1 1 0 1 GOOD
	//./track ./RESULTS/outVIDEO_2020-01-14_14-41-27.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-42-09.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-43-04.avi 0 1 0 0 0 1 1 0 1 GOOD
	//./track ./RESULTS/outVIDEO_2020-01-14_14-43-52.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-48-21.avi 0 1 0 0 0 1 1 0 1
	//./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 0 1 0 0 0 1 1 0 1 BEST

	//FUSION GLOBALS
	//bool PTZ_HOMOGRAPHY_FUSION = true;
	//bool plotAllPredictedFlowVectors = false;
	float scaleFactorLK1 = 1;//0.65;//0.5;//0.34;//0.72;//0.5; //SCALE MATCH POINT METHOD INPUT /0.55
	float minDistanceToIgnore = 0.35;//0.16; //0.26 /0.24 //0.36 //0.46 //0.56;  /// 0.36 - bst for half res
	float minNormalizedDistanceToIgnore = 0.0001;
	Mat cur2;


//YOLO 4
	//ROTATED BOUNDING BOX
	if(1==1){
		//use2ondBackLayer = false;
		cornersMax = 1880 * scaler; //300
		angleMin = 9;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1; //1
		minDistPTZ_FLOW2 = 2; //2
		maxDistPTZ_FLOW = 28; //42 //32
		diffEstimatesToPrevMAX = 25;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.64;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 80 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 8;
		dotPlotScaler = 0.5;
	}
//YOLO 4
	//ROTATED BOUNDING BOX
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 1880 * scaler; //300
		angleMin = 9;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.6; //1
		minDistPTZ_FLOW2 = 1; //2
		maxDistPTZ_FLOW = 28; //42 //32
		diffEstimatesToPrevMAX = 25;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.44;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 80 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 2;
		dotPlotScaler = 0.5;
	}


	//YOLO 4
	//ROTATED BOUNDING BOX
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 1380 * scaler; //300
		angleMin = 1;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.03; //1
		minDistPTZ_FLOW2 = 0.075; //2
		maxDistPTZ_FLOW = 38; //42 //32
		diffEstimatesToPrevMAX = 25;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.005;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 80 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 8;
		dotPlotScaler = 0.7;
	}


	//ROTATED BOUNDING BOX
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 5;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.5; //1
		minDistPTZ_FLOW2 = 1.2; //2
		maxDistPTZ_FLOW = 28; //42 //32
		diffEstimatesToPrevMAX = 25;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.1;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 80 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 8;
		dotPlotScaler = 0.7;
	}

	//FUSION3 - double PTZ scale (2)
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 8;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1.2; //1
		minDistPTZ_FLOW2 = 2.5; //2
		maxDistPTZ_FLOW = 28; //42 //32
		diffEstimatesToPrevMAX = 25;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.1;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 80 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 8;
		dotPlotScaler = 0.7;
	}

	//FUSION2 - double PTZ scale (2)
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 5;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.8; //1
		minDistPTZ_FLOW2 = 1; //2
		maxDistPTZ_FLOW = 35; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.12;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 40 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 4;
		dotPlotScaler = 0.5;
	}

	//FUSION1 - half PTZ scale B
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 6;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1; //1
		minDistPTZ_FLOW2 = 2; //2
		maxDistPTZ_FLOW = 42; //42 //32
		diffEstimatesToPrevMAX = 37;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.25;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 40 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 15;
		dotPlotScaler = 0.5;

		minDistanceToIgnore = 0.45;
	}
	//FUSION1 - half PTZ scale A
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 2;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.6; //1
		minDistPTZ_FLOW2 = 1; //2
		maxDistPTZ_FLOW = 42; //42 //32
		diffEstimatesToPrevMAX = 37;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.05;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 40 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 15;
		dotPlotScaler = 0.5;
	}

	//BEFORE FUSION
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 18;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.8; //1
		minDistPTZ_FLOW2 = 2.9; //2
		maxDistPTZ_FLOW = 32; //42 //32
		diffEstimatesToPrevMAX = 23;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.45;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 40 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.5;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 5;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.4; //1
		minDistPTZ_FLOW2 = 0.8; //2
		maxDistPTZ_FLOW = 37; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.15;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 140 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 40 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.5;
	}

	//
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 7;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.5; //1
		minDistPTZ_FLOW2 = 5; //2
		maxDistPTZ_FLOW = 27; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.21;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 90 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32; //12
		dotPlotScaler = 0.42;
		minFeatureDistance = 1 * scaler;
		cornerQuality = 0.01;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 12;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1.5; //1
		minDistPTZ_FLOW2 = 5; //2
		maxDistPTZ_FLOW = 17; //42 //32
		diffEstimatesToPrevMAX = 19;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.42;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 90 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.42;
		minFeatureDistance = 1 * scaler;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 12;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1.5; //1
		minDistPTZ_FLOW2 = 5; //2
		maxDistPTZ_FLOW = 17; //42 //32
		diffEstimatesToPrevMAX = 19;//14 //27
 		diffEstimatesToPrevMIN = 0.1;//1.2
		minVelocityMagnitude = 0.42;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 90 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.42;
		minFeatureDistance = 1 * scaler;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2320 * scaler; //300
		angleMin = 3;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.9; //1
		minDistPTZ_FLOW2 =1.5; //2
		maxDistPTZ_FLOW = 36; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.6;//1.2
		minVelocityMagnitude = 0.17;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 90 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.5;

		 minFeatureDistance = 5 * scaler; //10
		 cornerQuality = 0.04;
	}

	//USED IN 2oND EXPERIMENT !!!!
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 7;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.9; //1
		minDistPTZ_FLOW2 = 1.5; //2
		maxDistPTZ_FLOW = 37; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.6;//1.2
		minVelocityMagnitude = 0.86;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 40 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.5;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 7;//5;	//109; use high for POV videos 	
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1; //1
		minDistPTZ_FLOW2 = 2; //2
		maxDistPTZ_FLOW = 37; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.9;//1.2
		minVelocityMagnitude = 1.25;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 40 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.5;
	}
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 5;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.8; //1
		minDistPTZ_FLOW2 = 1; //2
		maxDistPTZ_FLOW = 37; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 0.2;//1.2
		minVelocityMagnitude = 0.15;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 40 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 32;
		dotPlotScaler = 0.5;
	}


	//PTZ  --- Tube1,4.mp4 with more points
	// ./track ./vids/Tube1.mp4 1 0.85 1 0 0 1
	// ./track ./RESULTS/outVIDEO_2020-01-14_14-48-37.avi 1 0.9 0 1 0 1
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 14;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 0.5; //1
		minDistPTZ_FLOW2 = 1; //2
		maxDistPTZ_FLOW = 37; //42 //32
		diffEstimatesToPrevMAX = 29;//14 //27
 		diffEstimatesToPrevMIN = 1.1;//1.2
		minVelocityMagnitude = 0.2;//0.6
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 40 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.5;
	}


	//./track ./RESULTS1/outVIDEO_2020-01-14_14-48-37.avi 1 0.7 0 0 0 1
	//PTZ 5 - reduce noise further
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 21;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 3;
		maxDistPTZ_FLOW = 38; //42
		diffEstimatesToPrevMAX = 32;//14
 		diffEstimatesToPrevMIN = 0.9;
		minVelocityMagnitude = 0.15;
		minDistInitMOSSE = 70 * scaler; //40 Init mosse if previous windows centers wintin this distance 180
		minDisttoCurrentMOSSE = 105 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow) 20
		minDisttoPrevWindow = 206 * scaler; //200 if distance of curent predicted track window to previous less than this number do init 70
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.62; //0.6

		minFeatureDistance = 5 * scaler; //10
		cornerQuality = 0.05;//0.04; //higher removes more points
	}


	//./track ./RESULTS1/outVIDEO_2020-01-14_14-48-37.avi 1 0.7 0 0 0 1
	//PTZ 4
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 2380 * scaler; //300
		angleMin = 19;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 2;
		maxDistPTZ_FLOW = 42; //42
		diffEstimatesToPrevMAX = 37;//14
 		diffEstimatesToPrevMIN = 0.8;
		minVelocityMagnitude = 0.1;
		minDistInitMOSSE = 290 * scaler; //40 Init mosse if previous windows centers wintin this distance 180
		minDisttoCurrentMOSSE = 5 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow) 20
		minDisttoPrevWindow = 226 * scaler; //200 if distance of curent predicted track window to previous less than this number do init 70
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.62; //0.6

		minFeatureDistance = 3 * scaler; //10
		cornerQuality = 0.02;//0.04;
	}


	//PTZ 3
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 480 * scaler; //300
		angleMin = 12;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 2;
		maxDistPTZ_FLOW = 23; //42
		diffEstimatesToPrevMAX = 27;//14
 		diffEstimatesToPrevMIN = 0.85;
		minVelocityMagnitude = 0.45;
		minDistInitMOSSE = 190 * scaler; //40 Init mosse if previous windows centers wintin this distance 180
		minDisttoCurrentMOSSE = 15 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow) 20
		minDisttoPrevWindow = 60 * scaler; //200 if distance of curent predicted track window to previous less than this number do init 70
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.62; //0.6

		minFeatureDistance = 6* scaler; //10
		cornerQuality = 0.01;//0.04;
	}

	//PTZ 2 - EXPERIEMNTS USED
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 480 * scaler; //300
		angleMin = 8;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 1;
		maxDistPTZ_FLOW = 23; //42
		diffEstimatesToPrevMAX = 39;//14
 		diffEstimatesToPrevMIN = 0.3;
		minVelocityMagnitude = 0.3;
		minDistInitMOSSE = 190 * scaler; //40 Init mosse if previous windows centers wintin this distance 180
		minDisttoCurrentMOSSE = 15 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow) 20
		minDisttoPrevWindow = 60 * scaler; //200 if distance of curent predicted track window to previous less than this number do init 70
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.65; //0.6

		minFeatureDistance = 6* scaler; //10
		cornerQuality = 0.01;//0.04;
	}

	//PTZ
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 480 * scaler; //300
		angleMin = 5;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 3;
		maxDistPTZ_FLOW = 62; //42
		diffEstimatesToPrevMAX = 37;//14
 		diffEstimatesToPrevMIN = 0.2;
		minVelocityMagnitude = 0.2;
		minDistInitMOSSE = 180 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 20 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 70 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.6;

		minFeatureDistance = 5* scaler; //10
		cornerQuality = 0.01;//0.04;
	}

	//Tube1,4.mp4
	if(1==0){
		//use2ondBackLayer = false;
		cornersMax = 380 * scaler; //300
		angleMin = 14;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 2;
		maxDistPTZ_FLOW = 32; //42
		diffEstimatesToPrevMAX = 27;//14
 		diffEstimatesToPrevMIN = 1.2;
		minVelocityMagnitude = 0.6;
		minDistInitMOSSE = 120 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 40 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 100 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 65;
		dotPlotScaler = 0.5;
	}
	//Tube5,6.mp4
	if(1==0){
		//use2ondBackLayer = true;
		cornersMax = 480 * scaler; //300
		angleMin = 110;//5;	//109; use high for POV videos
		angleMax = 360-angleMin;//355;	//300;
		minDistPTZ_FLOW = 1;
		minDistPTZ_FLOW2 = 1;
		maxDistPTZ_FLOW = 52; //42
		diffEstimatesToPrevMAX = 37;//14
 		diffEstimatesToPrevMIN = 4;
		minVelocityMagnitude = 1;
		minDistInitMOSSE = 70 * scaler; //40 Init mosse if previous windows centers wintin this distance
		minDisttoCurrentMOSSE = 100 * scaler; //200 if MOSSE distance above this to curent predicted track window // (distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
		minDisttoPrevWindow = 200 * scaler; //200 if distance of curent predicted track window to previous less than this number do init
		//plotDebug = false;
		//lowerCutoff = 0 * scaler; //pixels to ignore in the lower image part - feature points removed based on this value
		maxContours = 55;
		dotPlotScaler = 0.5;
	}

	//PTZ LOG FILE
	//rectangle(plotFrame, bbox2, Scalar( 255, 255, 255 ), 2, 1 );
	std::string fileName = "Tracking_Result_" + GetCurrentTimeForFileName() + ".csv";// to_string(static_cast<int>(HorAngle)) +".csv";		
	bool logData = true;
	if(logData){
			outfile.open(fileName, std::ios::out | std::ios::app);
			if (outfile.fail()){
				throw std::ios_base::failure(std::strerror(errno));
			}
			//make sure write fails with exception if something is wrong
			outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);

			string addVikonPosition2D = "";
			if(plotFromVikon && !usePTZ){
				addVikonPosition2D = "ViconScreenPosX;ViconScreenPosY;";
			}


			//WRITE HEADER

			//outfile << frameID << ";" << start << ";"	
			//		<< bbox2.x * (1/scaleFrame) << ";"
			//		<< bbox2.y * (1/scaleFrame)  << ";"
			//		<< bbox2.width * (1/scaleFrame) << ";" 
			//		<< bbox2.height * (1/scaleFrame) << ";" 
			//		<< currentYoloRect.x << ";" 
			//		<< currentYoloRect.y << ";" 
			//		<< currentYoloRect.width << ";" 
			//		<< currentYoloRect.height << ";" 
			//		<< std::endl;				

				outfile << "Video Frame ID" << ";" << "Time" << ";"
					<< "MOSSE Corner X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
					<< "YOLO Corner X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
					<< "Drone %" << ";"
					<< "trackFrameEstimateKALMAN X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";" 				
					<< "YoloA_enter_and_found" << ";" 
					<< "YoloB_enter_and_found" << ";" 
					<< "Mosse_enter_and_found" << ";" 
					<< "Homog_enter_and_found" << ";" 
					<< "currentPan" << ";" << "currentTilt" << ";" << "currentZoom" << ";" //<< "YawRate" << ";" << "pitchRate" << ";"<< "rollRate" << ";" << "zoomRate" << ";"					
					<< std::endl;	
				
	}
	//END PTZ LOG FILE 

	//PTZ ENABLES 
	//bool enableTestPanTilt = true; //must disable enablePIDracking to use !!!!
	//bool enableTestZoom = true; //must disable enablePIDracking to use !!!!
	//bool enablePIDracking = true;
	//bool enablePIDZoom = true;
	float PID_P = 0.9;//0.85f;//0.75f; //1.2 //0.92	
	float maxPanSpeed = 9.0;//8
	int maxPTZ_Zoom = 12000;//7000; ///// Zoom is 0 to 16384 (1x - 20x) //4800 //6500
	//END PTZ ENABLES
	

	//keep mosse history
	vector<Rect> bboxes;



//////////////////////////////////////////////////  KALMAN FILTER INIT //////////////////////////////////////////////////	

	    // >>>> Kalman Filter
	    int stateSize = 8;//6
	    int measSize = 6; //4
	    int contrSize = 0;

	    unsigned int type = CV_32F;
	    //KF (DP,MP,CP,type)
	    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

	    cv::Mat state(stateSize, 1, type);  // [x, y, v_x, v_y, a_x, a_y,  w, h]
	    cv::Mat meas(measSize, 1, type);    // [z_x, z_y, z_vx, z_vy, z_w,  z_h]
	    //cv::Mat procNoise(stateSize, 1, type)
	    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	    // Transition State Matrix A
	    // Note: set dT at each processing step!
	    // [ 1 0 dT 0  (1/2)*dT*dT 	0		0 0 ]
	    // [ 0 1 0  dT 0   		(1/2)*dT*dT	0 0 ]
	    // [ 0 0 1  0  dT 		0		0 0 ]
	    // [ 0 0 0  1  0 		dT 		0 0 ]
	    // [ 0 0 0  0  1		0		0 0 ]
	    // [ 0 0 0  0  0		1  		0 0 ]
	    // [ 0 0 0  0  0 		0  		1 0 ]
	    // [ 0 0 0  0  0 		0  		0 1 ]
	    cv::setIdentity(kf.transitionMatrix);

	    // Measure Matrix H - MP x DP
	    // [ 1 0 0 0 0 0 ]
	    // [ 0 1 0 0 0 0 ]
	    // [ 0 0 0 0 1 0 ]
	    // [ 0 0 0 0 0 1 ]
	    
       	    // Measure Matrix H - MP x DP ACCEL
  	    // [ 1 0 0 0 0 0 0 0]	//measurement of x position
	    // [ 0 1 0 0 0 0 0 0]	//measurement of y position
	    // [ 0 0 1 0 0 0 0 0]	//measurement of velocity x
	    // [ 0 0 0 1 0 0 0 0] 	//measurement of velocity y
	    // [ 0 0 0 0 0 0 1 0]	//measurements of width, height
	    // [ 0 0 0 0 0 0 0 1]
	    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	 
	    kf.measurementMatrix.at<float>(0)  = 1.0f;
	    kf.measurementMatrix.at<float>(9)  = 1.0f;
	    kf.measurementMatrix.at<float>(18) = 1.0f;
	    kf.measurementMatrix.at<float>(27) = 1.0f;
  	    kf.measurementMatrix.at<float>(38) = 1.0f;
	    kf.measurementMatrix.at<float>(47) = 1.0f;

	    // Process Noise Covariance Matrix Q - DP x DP
	    // [ Ex   0   0     0     0    0  ]
	    // [ 0    Ey  0     0     0    0  ]
	    // [ 0    0   Ev_x  0     0    0  ]
	    // [ 0    0   0     Ev_y  0    0  ]
	    // [ 0    0   0     0     Ew   0  ]
	    // [ 0    0   0     0     0    Eh ]

	    // Process Noise Covariance Matrix Q - DP x DP
	    // [ Ex   0   0     0     0    0  	0    0]
	    // [ 0    Ey  0     0     0    0  	0    0]
	    // [ 0    0   Ev_x  0     0    0  	0    0]
	    // [ 0    0   0     Ev_y  0    0  	0    0]
	    // [ 0    0   0     0     Ea_x 0  	0    0]
	    // [ 0    0   0     0     0    Ea_y 0    0]
	    // [ 0    0   0     0     0    0  	Ew   0]
	    // [ 0    0   0     0     0    0 	0    Eh]

	    //ACCELERATION
	    kf.processNoiseCov.at<float>(0) = 4.0;//6
	    kf.processNoiseCov.at<float>(9) = 4.0;
	    kf.processNoiseCov.at<float>(18) = 18;//20.01f; //18
	    kf.processNoiseCov.at<float>(27) = 18;//20.01f;
	    kf.processNoiseCov.at<float>(36) = 14.0;
	    kf.processNoiseCov.at<float>(45) = 14.0;//accel
	    kf.processNoiseCov.at<float>(54) = 4.0;
	    kf.processNoiseCov.at<float>(63) = 4.0;


	    //// Measures Noise Covariance Matrix R - 4x4 - MP x MP - measSize x measSize
	    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(350));//cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1)); //50
	  
	    //ACCEL
	    kf.measurementNoiseCov.at<float>(0) = 2700;//12150;//1e-5;
	    kf.measurementNoiseCov.at<float>(7) = 2700;//12150;
	    kf.measurementNoiseCov.at<float>(14) = 270*270;//
	    kf.measurementNoiseCov.at<float>(21) = 270*270;//velocity cov
 	    kf.measurementNoiseCov.at<float>(28) = 22500.0;//150*150
	    kf.measurementNoiseCov.at<float>(35) = 22500.0;

	    // <<<< Kalman Filter
	   
	    double ticks = 0;
	    bool found = false;
	    int notFoundCount = 0;
	    cv::Rect predRect;
	    cv::Point center;
	    vector<Rect> predRects;
	    vector<Rect> predRects_Kalman_Window;
	    vector<Point> trackFrameEstimateCenters;
	////////////////////////////////////////////////// END KALMAN INIT //////////////////////////////////////////////////




int toggleCommandrate = 0;
bool zoomingOut = true;


cout << "Initializaion finished ... starting camera loop !!!" << endl;



	//////////////// PTZ FLOW ///////////////
	//SPARKFUN SENSOR
		useSparkfun = false;

		if(!usePTZ){
			useSparkfun = false;
		}

		if(useSparkfun){
			initComSparkFun();		
		}
		std::thread t3(task3, "SparkFun");
	//END SPARKFUN SENSOR
	//////////////// END PTZ FLOW //////////////


double timerSTART = (double)getTickCount();
time_t startTime = time(0);



		YoloA_enter_and_found1 = 0; //if enter become 1, if found become 2
		YoloB_enter_and_found1 = 0;
		Mosse_enter_and_found1 = 0;
		Homog_enter_and_found1 = 0;
		YoloA_enter_and_found2 = 0; //if enter become 1, if found become 2
		YoloB_enter_and_found2 = 0;
		Mosse_enter_and_found2 = 0;
		Homog_enter_and_found2 = 0;


		int sizing = 224;


///////////////////////////////////////////////////// MAIN LOOP //////////////////////////////////////////////////////////////
	while(ros::ok() && true){




		//NEW1 - reset here
		YoloA_enter_and_found = 0; //if enter become 1, if found become 2
		YoloB_enter_and_found = 0;
		Mosse_enter_and_found = 0;
		Homog_enter_and_found = 0;

	
	



		//PTZ
		float panDiff = 0; float tiltDiff = 0; float zoomDiff = 0;
		//END PTZ


		if(plotDebugEnabled == 0){
			showFeedPreview = false; 		
			plotDebug = false; 
			plotDebugTracking = false;
		}
		if(plotDebugEnabled == 1){
			showFeedPreview = true; 		
			plotDebug = true; 
			plotDebugTracking = true;
		}
		

		if(runExperiment){
			showFeedPreview = false; 		
			plotDebug = false; 
			plotDebugTracking = false;

			plotDebugTrackingFUSION = false;
			
			PTZ_HOMOGRAPHY_FUSION = true;
			enablePIDracking = true;
			//enablePIDZoom = true;
			//bool enableTestZoom = false;
			//bool enableTestPanTilt = false;
		}

		//Provide pan-tilt-zoom by reading the file associated with the video
		if(!usePTZ && cameraPan.size() > frameID+1){
			currentPan  = cameraPan[frameID+1];//first line is he labels (0)
			currentTilt = cameraTilt[frameID+1];
			currentZoom = cameraZoom[frameID+1];

			//cout << "currentPan:" << currentPan << endl;
			//PLOT MAVIC PRO POSITION FROM VIKON - MavicProjectedPos
			//circle(canvas, MavicProjectedPos[frameID], 2,  CV_RGB(42/112, 2/112, 220/121), -1);			
		}

		vector<Point2f> LeftPropellerContourHull;
		vector<Point2f> RightPropellerContourHull;

		vector<Point2f> upperLeftPropellerContour;
		vector<Point2f> lowerLeftPropellerContour;
		vector<Point2f> upperRightPropellerContour;
		vector<Point2f> lowerRightPropellerContour;
		bool allULout = false;
		bool allLLout = false;
		bool allURout = false;
		bool allLRout = false;
		int ULout = 0;  int LLout = 0; int URout = 0; int LRout = 0;
		//float maxZoom = 16384;
		float correctPanTiltByZoomF = currentZoom/maxZoom;
		float adjustedF = 0.00442 + correctPanTiltByZoomF * (0.0885-0.00442) * 0.1; //Zoom is 0 to 16384, resize F by the zoom factor
		//corrected with non linear curve
		correctPanTiltByZoomF = 1.06 + 0.12 * exp(currentZoom * 3.18 * pow(10,-4));// 3x zoom means focal lenght max / min = 3	
		adjustedF = correctPanTiltByZoomF * (0.00442/1.18);
		if(cutPropellers){
//			cout << "currentPan:" << currentPan << ", currentTilt:" << currentTilt << ", currentZoom:" << currentZoom  <<  endl;
			//PTZ 
//			float maxZoom = 16384;
//			float correctPanTiltByZoomF = currentZoom/maxZoom;
//			float adjustedF = 0.00442 + correctPanTiltByZoomF * (0.0885-0.00442) * 0.1; //Zoom is 0 to 16384, resize F by the zoom factor
			//corrected with non linear curve
//			correctPanTiltByZoomF = 1.06 + 0.12 * exp(currentZoom * 3.18 * pow(10,-4));// 3x zoom means focal lenght max / min = 3	
//			adjustedF = correctPanTiltByZoomF * (0.00442/1.18);
 			//adjustedF = 0.00442;
			//Pan is 0 at home. Right is positive, max 2448. Left ranges from full left 63088 to 65535 before home.
			//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully depressed at 65104 to 65535 before home. 
			//Horizontal angle of view PTZ 3.36 to 60.7 degrees
			//Vertical angle of view PTZ 1.89 to 34.1 degrees
			//Pan range +-170 degrees
			//Tilt range  -30 to +90 degrees

			Eigen::Matrix3f toCameraRotPAN;//right pan is positive, rotation aound Y axis
			float anglePAN = currentPan;// 0;//FIND PAN CORRESPONDING TO HIS FRAME ID FROM OUR LOGS !!!!! 	//cameraPan
			if(anglePAN >= 0 && anglePAN <= 2448){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				anglePAN = 170*(anglePAN/2448);
			}  
			if(anglePAN >= 63088 && anglePAN <= 65535){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				anglePAN = -1*(170-170*( (anglePAN-63088)/ (65535-63088) ));
			} 
			//anglePAN = -45;  
			//convert to rad
			anglePAN = anglePAN * (CV_PI/180);
			//anglePAN = 0;
			toCameraRotPAN << cos(anglePAN), 0, sin(anglePAN),
					0, 1, 0,
			     		-sin(anglePAN), 0, cos(anglePAN);
			//	std::cout << "toCameraRotPAN = " << toCameraRotPAN << endl;

			Eigen::Matrix3f toCameraRotTILT;//up tilt is positive, rotation aound X axis
			float angleTILT = currentTilt;//0;//FIND TILT CORRESPONDING TO HIS FRAME ID FROM OUR LOGS !!!!!	//cameraTilt
			if(angleTILT >= 0 && angleTILT <= 1296){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				angleTILT = 90*(angleTILT/1296);
			}  
			if(angleTILT >= 65104 && angleTILT <= 65535){ //LOOKING TO THE RIGHT SIDE 0 to 170 degrees
				angleTILT = -(30-30*( (angleTILT-65104)/ (65535-65104) ));
			} 
			//convert to rad
			angleTILT = angleTILT * (CV_PI/180);
			//angleTILT = 0;
			toCameraRotTILT << 1, 0, 0,
					0, cos(angleTILT), -sin(angleTILT),
			     		0, sin(angleTILT),  cos(angleTILT);

			//CREATE CIRCLE POINTS AROUND CAMERA AXIS CENTER
			vector<Point3f> propellerUpperLeft; //level with drone center height
			int polygonRes = 90;
			float angleIncrease = 0;
			float propellerRadius = 0.42;//0.36;
			for (int i=0;i<polygonRes;i++){

				angleIncrease = i*(360/polygonRes) + 2*frameID;
				angleIncrease = angleIncrease * (CV_PI/180);
				Eigen::Vector4f pointOnCircle(sin(angleIncrease)*propellerRadius, 0, cos(angleIncrease)*propellerRadius, 1);	
				 //angleIncrease = angleIncrease + i*(360/polygonRes);	
			
				//TRANSFORM CIRCLE CENTER AND POINTS TO PROPELLER PLACEMENT RELATIVE TO THE CAMERA
				Eigen::Vector3f toVulkanCenterP(0, 0.16, -0.04);	
				Eigen::Vector3f toVulkanPropellerCenterP(-0.53, 0, 0.574);
				Eigen::Matrix3f IdentityM;
				IdentityM << 1, 0, 0,
				     	     0, 1, 0,
				     	     0, 0, 1;
				//Eigen::Matrix3f R;
				// Find your Rotation Matrix
				//Eigen::Vector3f T;
				// Find your translation Vector
				Eigen::Matrix4f toVulkanCenter; // Your Transformation Matrix
				toVulkanCenter.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
				toVulkanCenter.block<3,3>(0,0) = IdentityM;// no rotation
				toVulkanCenter.block<3,1>(0,3) = toVulkanCenterP;

				Eigen::Matrix4f toVulkanPropellerCenter; // Your Transformation Matrix
				toVulkanPropellerCenter.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
				toVulkanPropellerCenter.block<3,3>(0,0) = IdentityM;// no rotation
				toVulkanPropellerCenter.block<3,1>(0,3) = toVulkanPropellerCenterP;

				Eigen::Matrix4f RotPanTiltCamera; // Your Transformation Matrix
				RotPanTiltCamera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
				RotPanTiltCamera.block<3,3>(0,0) = toCameraRotPAN*toCameraRotTILT;
				Eigen::Vector3f posCameraZERO(0.0,0.0,0.0);
				RotPanTiltCamera.block<3,1>(0,3) = posCameraZERO;

				//UPPER LEFT
				Eigen::Vector3f toVulkanPropellerCenterPUP(0, 0.03, 0); //GO DOWN in Y (positive down in camera) 3cm up
				Eigen::Matrix4f toVulkanPropellerCenterUP; // Your Transformation Matrix
				toVulkanPropellerCenterUP.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
				toVulkanPropellerCenterUP.block<3,3>(0,0) = IdentityM;// no rotation
				toVulkanPropellerCenterUP.block<3,1>(0,3) = toVulkanPropellerCenterPUP;

				//Project points and find contour (vector<Point>)
				Eigen::Matrix4f TransToPoint = toVulkanCenter * toVulkanPropellerCenter * toVulkanPropellerCenterUP * RotPanTiltCamera;//* RotPanTiltCamera; 
				//Eigen::Vector4f posMavicCameraSpace = TransToPoint * posMavic4Dims;
				Eigen::Vector4f posPointsCameraSpace = TransToPoint.inverse() * pointOnCircle;
				//	cout << "posMavicCameraSpace = " << posMavicCameraSpace << endl;
			
				//X' = X * (adjustedF/Z)
				//Y' = Y * (adjustedF/Z)
				//estX = estX + ((0.001 * 5.37) / 2);
				//estY = estY + ((0.001 * 4.04) / 2);					
				//float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
				//float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);	
				//vector<Point> MavicProjectedPos;
				Point2f addMavicPoint((posPointsCameraSpace(0) * adjustedF) / posPointsCameraSpace(2) , (posPointsCameraSpace(1) * adjustedF) / posPointsCameraSpace(2)  );
				addMavicPoint.x = addMavicPoint.x + ((0.001 * 5.37) / 2);
				addMavicPoint.y = addMavicPoint.y + ((0.001 * 4.04) / 2);
				addMavicPoint.x = addMavicPoint.x * prev.cols * 1 / (0.001 * 5.37);
				addMavicPoint.y = addMavicPoint.y * prev.rows * 1  / (0.001 * 4.04);

				if(addMavicPoint.x > 0 && addMavicPoint.x < prev.cols * 1 &&  addMavicPoint.y > 0 && addMavicPoint.y < prev.rows * 1){
					//upperLeftPropellerContour.push_back(addMavicPoint);
					if(upperLeftPropellerContour.size() > 0 && abs(addMavicPoint.x - upperLeftPropellerContour[upperLeftPropellerContour.size()-1].x) > 50 ){
						ULout++;
					}
				}else{
					ULout++;
				}
				upperLeftPropellerContour.push_back(addMavicPoint);

				//UPPER RIGHT - modify final translation
				toVulkanPropellerCenterPUP(0) = 0.53*2; //move from left propeller to the right one
				toVulkanPropellerCenterUP.block<3,1>(0,3) = toVulkanPropellerCenterPUP;//load new transformation
				TransToPoint = toVulkanCenter * toVulkanPropellerCenter * toVulkanPropellerCenterUP * RotPanTiltCamera;				
				posPointsCameraSpace = TransToPoint.inverse() * pointOnCircle;				
				Point2f addMavicPointUPR((posPointsCameraSpace(0) * adjustedF) / posPointsCameraSpace(2) , (posPointsCameraSpace(1) * adjustedF) / posPointsCameraSpace(2)  );
				addMavicPointUPR.x = addMavicPointUPR.x + ((0.001 * 5.37) / 2);
				addMavicPointUPR.y = addMavicPointUPR.y + ((0.001 * 4.04) / 2);
				addMavicPointUPR.x = addMavicPointUPR.x * prev.cols * 1 / (0.001 * 5.37);
				addMavicPointUPR.y = addMavicPointUPR.y * prev.rows * 1  / (0.001 * 4.04);

				if(addMavicPointUPR.x > 0 && addMavicPointUPR.x < prev.cols * 1 &&  addMavicPointUPR.y > 0 && addMavicPointUPR.y < prev.rows * 1){
					//upperRightPropellerContour.push_back(addMavicPointUPR);
					if(upperRightPropellerContour.size() > 0 && abs(addMavicPointUPR.x - upperRightPropellerContour[upperRightPropellerContour.size()-1].x) > 50 ){
						URout++;
					}
				}else{
					URout++;
				}
				upperRightPropellerContour.push_back(addMavicPointUPR);

				//LOWER LEFT
				Eigen::Vector3f toVulkanPropellerCenterPLL(0, 0.172, 0); //GO DOWN in Y (positive down in camera) 14.85cm 0.152
				Eigen::Matrix4f toVulkanPropellerCenterLL; // Your Transformation Matrix
				toVulkanPropellerCenterLL.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
				toVulkanPropellerCenterLL.block<3,3>(0,0) = IdentityM;// no rotation
				toVulkanPropellerCenterLL.block<3,1>(0,3) = toVulkanPropellerCenterPLL;

				TransToPoint = toVulkanCenter * toVulkanPropellerCenter * toVulkanPropellerCenterLL * RotPanTiltCamera;
				posPointsCameraSpace = TransToPoint.inverse() * pointOnCircle;				
				Point2f addMavicPointLL((posPointsCameraSpace(0) * adjustedF) / posPointsCameraSpace(2) , (posPointsCameraSpace(1) * adjustedF) / posPointsCameraSpace(2)  );
				addMavicPointLL.x = addMavicPointLL.x + ((0.001 * 5.37) / 2);
				addMavicPointLL.y = addMavicPointLL.y + ((0.001 * 4.04) / 2);
				addMavicPointLL.x = addMavicPointLL.x * prev.cols * 1 / (0.001 * 5.37);
				addMavicPointLL.y = addMavicPointLL.y * prev.rows * 1  / (0.001 * 4.04);

				if(addMavicPointLL.x > 0 && addMavicPointLL.x < prev.cols * 1 &&  addMavicPointLL.y > 0 && addMavicPointLL.y < prev.rows * 1){
					//lowerLeftPropellerContour.push_back(addMavicPointLL);
					if(lowerLeftPropellerContour.size() > 0 && abs(addMavicPointLL.x - lowerLeftPropellerContour[lowerLeftPropellerContour.size()-1].x) > 50 ){
						LLout++;
					}
				}else{
					LLout++;
				}
				lowerLeftPropellerContour.push_back(addMavicPointLL);

				//LOWER RIGHT
				//UPPER RIGHT - modify final translation
				toVulkanPropellerCenterPLL(0) = 0.53*2; //move from left propeller to the right one
				toVulkanPropellerCenterLL.block<3,1>(0,3) = toVulkanPropellerCenterPLL;//load new transformation
				TransToPoint = toVulkanCenter * toVulkanPropellerCenter * toVulkanPropellerCenterLL * RotPanTiltCamera;
				posPointsCameraSpace = TransToPoint.inverse() * pointOnCircle;			
				Point2f addMavicPointLR((posPointsCameraSpace(0) * adjustedF) / posPointsCameraSpace(2) , (posPointsCameraSpace(1) * adjustedF) / posPointsCameraSpace(2)  );
				addMavicPointLR.x = addMavicPointLR.x + ((0.001 * 5.37) / 2);
				addMavicPointLR.y = addMavicPointLR.y + ((0.001 * 4.04) / 2);
				addMavicPointLR.x = addMavicPointLR.x * prev.cols * 1 / (0.001 * 5.37);
				addMavicPointLR.y = addMavicPointLR.y * prev.rows * 1  / (0.001 * 4.04);

				if(addMavicPointLR.x > 0 && addMavicPointLR.x < prev.cols * 1 &&  addMavicPointLR.y > 0 && addMavicPointLR.y < prev.rows * 1){
					//lowerRightPropellerContour.push_back(addMavicPointLR);
					if(lowerRightPropellerContour.size() > 0 && abs(addMavicPointLR.x - lowerRightPropellerContour[lowerRightPropellerContour.size()-1].x) > 50 ){
						LRout++;
					}
				}else{
					LRout++;
				}
				lowerRightPropellerContour.push_back(addMavicPointLR);

				//EXTRA CHECKS
				
				
				
				
				
			}
			//upperLeftPropellerContour.push_back(upperLeftPropellerContour[0]);
			//lowerLeftPropellerContour.push_back(lowerLeftPropellerContour[0]);
			//upperRightPropellerContour.push_back(upperRightPropellerContour[0]);
			//lowerRightPropellerContour.push_back(lowerRightPropellerContour[0]);

			if(upperLeftPropellerContour.size() - ULout < 20){//if(ULout == upperLeftPropellerContour.size() ){
				allULout = true;
			}
			if(lowerLeftPropellerContour.size() - LLout < 20){//if(LLout == lowerLeftPropellerContour.size() ){
				allLLout = true;
			}
			if(upperRightPropellerContour.size() - URout < 20){//if(URout == upperRightPropellerContour.size() ){
				allURout = true;
			}
			if(lowerRightPropellerContour.size() - LRout < 20){//if(LRout == lowerRightPropellerContour.size() ){
				allLRout = true;
			}
				
			//CHECK IF AT LEAST FEW INSIDE SCREEN
			//for(int i=0;i< lowerRightPropellerContour.size();i++){
			//	circle(canvas, lowerRightPropellerContour[i], 5,  CV_RGB(142/1, 2/112, 220/121), -1);
			//	if(i<lowerRightPropellerContour.size()-1){
			//		line(canvas,lowerRightPropellerContour[i], lowerRightPropellerContour[i+1],CV_RGB(255, 255, 255) , 1, 8,0); 
			//	}
			//}

			//CONVEX HULL
			if(upperLeftPropellerContour.size() > 4 && lowerLeftPropellerContour.size() > 4){
				//vector<vector<Point2f>> LeftPropellerContours;
				//LeftPropellerContours.push_back(upperLeftPropellerContour);
				//LeftPropellerContours.push_back(lowerLeftPropellerContour);
				vector<Point2f> LeftPropellerContours;
				for (int i=0; i<upperLeftPropellerContour.size(); i++){
					LeftPropellerContours.push_back(upperLeftPropellerContour[i]);
				}
				for (int i=0; i<lowerLeftPropellerContour.size(); i++){
					LeftPropellerContours.push_back(lowerLeftPropellerContour[i]);
				}
				convexHull(LeftPropellerContours, LeftPropellerContourHull);//LeftPropellerContourHull = convexHull(LeftPropellerContours);
				
			}
			if(upperRightPropellerContour.size() > 4 && lowerRightPropellerContour.size() > 4){
				//vector<vector<Point2f>> RightPropellerContours;
				//RightPropellerContours.push_back(upperRightPropellerContour);
				//RightPropellerContours.push_back(lowerRightPropellerContour);			
				//convexHull(RightPropellerContours, RightPropellerContourHull);//RightPropellerContourHull = convexHull(RightPropellerContours);
				vector<Point2f> RightPropellerContours;
				for (int i=0; i<upperRightPropellerContour.size(); i++){
					RightPropellerContours.push_back(upperRightPropellerContour[i]);
				}
				for (int i=0; i<lowerRightPropellerContour.size(); i++){
					RightPropellerContours.push_back(lowerRightPropellerContour[i]);
				}
				convexHull(RightPropellerContours, RightPropellerContourHull);
			}
			////////
		}

		

		if(useGpuFeaturesTrack){
			cv::ocl::setUseOpenCL(useGpuFeaturesTrack);
		}

		// Start timer
		double timer = (double)getTickCount();

		//NASOS
		frameID++;		

		Mat temp; cap >> temp;
		//cap >> frame;
		temp.copyTo(frame);



		//int sizing = 224;//124;//224;//416;
		//if(sizing == 224){
		//	sizing = 416;		
		//}else if(sizing == 416){
		//	sizing = 800;
		//}else{
		//	sizing = 224;
		//}



		bool loadDSLRImages = false;
		if(1==0 && loadDSLRImages){
			if(frameID == 1){
				temp = frame = imread("DLSR/DSCN0015.JPG", IMREAD_COLOR);
						}
			if(frameID == 2){
				temp = frame = imread("DLSR/DSCN0016.JPG", IMREAD_COLOR);
						}
			if(frameID == 3){
							temp = frame = imread("DLSR/DSCN0017.JPG", IMREAD_COLOR);
						}
			if(frameID == 4){
							temp = frame = imread("DLSR/DSCN0018.JPG", IMREAD_COLOR);
						}
			if(frameID == 5){
							temp = frame = imread("DLSR/DSCN0020.JPG", IMREAD_COLOR);
						}
			if(frameID == 6){
							temp = frame = imread("DLSR/DSCN0021.JPG", IMREAD_COLOR);
						}
			if(frameID == 7){
							temp = frame = imread("DLSR/DSCN0022.JPG", IMREAD_COLOR);
						}
			if(frameID == 8){
							temp = frame = imread("DLSR/DSCN0023.JPG", IMREAD_COLOR);
						}
			if(frameID == 9){
							temp = frame = imread("DLSR/DSCN0024.JPG", IMREAD_COLOR);
						}
			if(frameID == 10){
							temp = frame = imread("DLSR/DSCN0025.JPG", IMREAD_COLOR);
						}
			if(frameID == 11){
							temp = frame = imread("DLSR/DSCN0026.JPG", IMREAD_COLOR);
						}
			if(frameID == 12){
							temp = frame = imread("DLSR/DSCN0027.JPG", IMREAD_COLOR);
						}
			if(frameID == 13){
							temp = frame = imread("DLSR/DSCN0028.JPG", IMREAD_COLOR);
						}

			if(frameID == 14){
							temp = frame = imread("DLSR/DSCN0029.JPG", IMREAD_COLOR);
						}
			if(frameID == 15){
							temp = frame = imread("DLSR/DSCN0030.JPG", IMREAD_COLOR);
						}
			if(frameID == 16){
							temp = frame = imread("DLSR/DSCN0031.JPG", IMREAD_COLOR);
						}
			if(frameID == 17){
							temp = frame = imread("DLSR/DSCN0032.JPG", IMREAD_COLOR);
						}
			if(frameID == 18){
							temp = frame = imread("DLSR/DSCN0033.JPG", IMREAD_COLOR);
						}
			if(frameID == 19){
							temp = frame = imread("DLSR/DSCN0034.JPG", IMREAD_COLOR);
						}
			if(frameID == 20){
							temp = frame = imread("DLSR/DSCN0035.JPG", IMREAD_COLOR);
						}
			if(frameID == 21){
							temp = frame = imread("DLSR/DSCN0036.JPG", IMREAD_COLOR);
						}
			if(frameID == 22){
							temp = frame = imread("DLSR/DSCN0037.JPG", IMREAD_COLOR);
						}
			if(frameID == 23){
							temp = frame = imread("DLSR/DSCN0038.JPG", IMREAD_COLOR);frameID=0;
						}

		}



	int frameOffset = 6;
	if(loadDSLRImages){
		if(frameID > 22){
			//frameID =0; 
			temp = frame = imread("DLSR/DSCN0015.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 1){	
			temp = frame = imread("DLSR/DSCN0016.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 2){	
			temp = frame = imread("DLSR/DSCN0017.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 3){	
			temp = frame = imread("DLSR/DSCN0018.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 4){	
			temp = frame = imread("DLSR/DSCN0020.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 5){	
			temp = frame = imread("DLSR/DSCN0021.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 6){	
			temp = frame = imread("DLSR/DSCN0022.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 7){	
			temp = frame = imread("DLSR/DSCN0023.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 8){	
			temp = frame = imread("DLSR/DSCN0024.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 9){	
			temp = frame = imread("DLSR/DSCN0025.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 10){	
			temp = frame = imread("DLSR/DSCN0026.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 11){	
			temp = frame = imread("DLSR/DSCN0027.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 12){	
			temp = frame = imread("DLSR/DSCN0028.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 13){	
			temp = frame = imread("DLSR/DSCN0029.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 14){	
			temp = frame = imread("DLSR/DSCN0030.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 15){	
			temp = frame = imread("DLSR/DSCN0031.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 16){	
			temp = frame = imread("DLSR/DSCN0032.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 17){	
			temp = frame = imread("DLSR/DSCN0033.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 18){	
			temp = frame = imread("DLSR/DSCN0034.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 19){	
			temp = frame = imread("DLSR/DSCN0035.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 20){	
			temp = frame = imread("DLSR/DSCN0036.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 21){	
			temp = frame = imread("DLSR/DSCN0037.JPG", IMREAD_COLOR);
		}
		if(frameID > 22+frameOffset * 22){	
			temp = frame = imread("DLSR/DSCN0038.JPG", IMREAD_COLOR);
		}
	}
	for ( int i = 1; i < 6; i = i + 2 )
	{ 
		//GaussianBlur( temp, temp, Size( i, i ), 0, 0 );
		//GaussianBlur( frame, frame, Size( i, i ), 0, 0 );
	}
	//cout << "fid = " << frameID << endl;


		//////////////////// YOLO 4
		//while(1==0 && !stop)
		//{
			//cout<<frame.size<<endl;
			//cap >> frame;
			//if (!capture.read(frame))
			//{
			//    printf("fail to read.\n");
			//    return 0;
			//}

		      	double before = get_time_point();		       
		  
	
			//int sizing = 224;//124;//224;//416;
			int sizingA = 416;//800;
			sizing = 224;
			

			if(plotDebug){
				putText(frame, to_string(sizingA) + " , " + to_string(sizing) , Point(850,50), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(250, 60, 60),2);	
			}


			Mat copyFrame;
			frame.copyTo(copyFrame);
			Mat copyBlob;
			blob.copyTo(copyBlob);	


			prevRectFixed.x = prevRect.x;
			prevRectFixed.y = prevRect.y;
			prevRectFixed.width = prevRect.width;
			prevRectFixed.height = prevRect.height;


			//cout << "prevRect X=" << prevRect.x << endl;
			if((!foundWindow && !foundWindowInCropped)){// || !foundWindow)){//if(prevRect.x == 0 || !fullOptimal){
				infereDrone(frame, blob, sizingA, classNamesVec, thresh, nms, classes,net,output_names, detections, 0);// 0 == LOWER RESOLUTION //v1.3 416
				//cout << "INININI 111" << endl;
				currentYoloRect = prevRect;
				cout << "FULL RES " << " foundWindow = "<< foundWindow <<   endl;

				if(!foundWindow){
					foundWindowInCropped = true;//
					prevRect.x   = 0;
					prevRect.y  = 0;
					prevRect.width = frame.cols;
					prevRect.height = frame.rows;
					tryDifferentRes = true;
				}

			}else


			if(1==1 || foundWindow){// || (prevRect.width > 1 && fullOptimal)){ //if(!foundWindow || (prevRect.x > 0 && fullOptimal)){
				cout << "LOW RES" << endl;
				//infereDrone(frame, blob, 224, classNamesVec, thresh, nms, classes,net,output_names, detections, 0);// 0 == LOWER RESOLUTION

				if(1==1){

						//cout << "Window not found, using cropped image around previous window" << endl;
						//cout << "prevRect=" << prevRect << endl;

						int ax = prevRect.x  - 1.0*prevRect.width;
						int ay = prevRect.y  - 1.0*prevRect.height;
						int aw = prevRect.width  * 3;
						int ah = prevRect.height * 3;
						if(ax < 0){
							ax = 0; 
						}
						if(ay < 0){
							ay = 0; 
						}
						if(ax + aw > frame.cols){
							aw = frame.cols - ax;
							//prevRectFixed.width = aw;
						}
						if(ay + ah > frame.rows){
							ah = frame.rows - ay;
							//prevRectFixed.height = ah;
						}				
						
						prevRectFixed.x = ax;
						prevRectFixed.y = ay;

						//cout << "ax:" << ax << " ay:" << ay << " aw:" << aw << " ah:" << ah << endl;
						Rect cropRect = Rect(ax, ay, aw, ah);

						cout << "cropRect = " << cropRect << endl;

						if(tryDifferentRes){//TRY different Yolo res instead of cropped, if main yolo did not find window
							cropRect.x = 0;
							cropRect.y  = 0;
							cropRect.width = frame.cols;
							cropRect.height = frame.rows;
							tryDifferentRes = false;	

							infereDrone(frame, blob, sizing, classNamesVec, thresh, nms, classes,net3,output_names3, detections, 0);// 0 == LOWER RESOLUTION //v1.3 416
							//cout << "INININI 111" << endl;
							currentYoloRect = prevRect;					
						}else{
				
							//cout << "w:" << frame.cols << " h:" << frame.rows << " cropRect:" << cropRect << endl;
							detections.clear();
							Mat cropedImage = frame(cropRect);
							Mat copiedIMG; cropedImage.copyTo(copiedIMG);
							infereDrone(copiedIMG, blob, sizing, classNamesVec, thresh, nms, classes,net3,output_names3, detections, 1); //1 == CROPPED
							if(plotDebug){
								imshow("video cropped",copiedIMG);
							}
						}
						

						float red = 255*get_color(2,1,80);
						float green = 255*get_color(1,1,80);
						float blue = 255*get_color(0,1,80);
						const auto color = Scalar(blue, green, red);
						if(plotDebug){
							//cv::rectangle(frame, cv::Point(prevRect.x + 1.5*prevRect.width, prevRect.y + 1.0*prevRect.height), cv::Point(prevRect.x + prevRect.width/1, prevRect.y + prevRect.height/1), color/2, 3);
							cv::rectangle(frame, cv::Point(prevRect.x, prevRect.y), cv::Point(prevRect.x + prevRect.width, prevRect.y + prevRect.height), color/2, 3);
						}
						currentYoloRect = prevRect;

						//LABELS
						std::ostringstream label_ss;
						label_ss << classNamesVec[0] << ": " << std::fixed << std::setprecision(2) << currentScore;
						auto label = label_ss.str();
						int baseline;
						auto label_bg_sz = getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
						if(plotDebug){
							rectangle(frame, Point(prevRect.x, prevRect.y - label_bg_sz.height - baseline - 10), cv::Point(prevRect.x + label_bg_sz.width, prevRect.y), color, cv::FILLED);
							putText(frame, label.c_str(), Point(prevRect.x, prevRect.y - baseline - 5), FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
						}
				}
				if(!foundWindow){
						//cout << "Window not found, using higher resolution" << endl;
						//detections.clear();
						//infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 2); //2 == FULL RESOLUTION
				}

				if(1==0){
					if(prevRect.x > 0){
						cout << "Window not found, using cropped image around previous window" << endl;
						infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 1); //1 == CROPPED
					}
					if(!foundWindow){
						cout << "Window not found, using higher resolution" << endl;
						detections.clear();
						//infereDrone(frame, blob, 416, classNamesVec, thresh, nms, classes,net2,output_names2, detections, 2); //2 == FULL RESOLUTION
					}
				}
			}


			if(plotDebug){
				imshow("video",frame);
			}
			//int c=waitKey(1);
			//if((char)c==27)
			//    break;
			//else if(c>=0)
			//    waitKey(0);
			//video.write(frame);

		//}

	//cout << "FOUND WINDOW === " << foundWindow << endl;


	//if(1==1 || !foundWindow || prevRect.x == 0){			
	//if(  !foundWindow || prevRect.x == 0){
	if(  (!foundWindow && !foundWindowInCropped) || prevRect.x == 0){





		//NEW1
		Homog_enter_and_found = 1; Homog_enter_and_found1++;





		if(frameID < startFrame){
			continue;
		}

		//GPU resize	
		if(scaler != 1){
			cv::resize(temp.getUMat(ACCESS_RW), cur, cv::Size(), scaler, scaler);
		}else{
			temp.copyTo(cur);
		}
	
		if(cur.data == NULL) {
			break;
		}	

		//v0.1
		if(frameID < 30){
			frameID++;
			//cout << "show" << endl;
			


			cur.copyTo(prev);

			cvtColor(prev, prev_grey, COLOR_BGR2GRAY);


			prev = cur.clone();
			cvtColor(cur.getUMat(ACCESS_RW), cur_grey, COLOR_BGR2GRAY);
			cur_grey.copyTo(prev_grey);

			//imshow("cur", cur);
			//imshow("prev", prev);

			//imshow("cur_grey", cur_grey);
			//imshow("prev_grey", prev_grey);

			waitKey(30);
			continue;
		}	

		//cout << "Reached 1" << endl;


		//cvtColor(cur, cur_grey, COLOR_BGR2GRAY);
		cvtColor(cur.getUMat(ACCESS_RW), cur_grey, COLOR_BGR2GRAY);

		// vector from prev to cur
		vector <Point2f> prev_corner, cur_corner;
		vector <Point2f> prev_corner2, cur_corner2, ptzEstimatedCorners, ptzBasedFlowCorners;

		vector <float>  ptzBasedFlowCorners_MAG;

		vector <Point2f> prev_corner3, prev_corner4, cur_corner3,cur_corner4; //NASOS
		vector <uchar> status;
		vector <float> err;

		//MATCH POINS	
		//Mat homog;
		bool useORB = false; 
		//compares the points found in two images using same method (than find features and estimate with optical flow) - ADD option to only find with ORB and simate with flow combination
		
		//int cornersMax = 300;//400;//150; //480
		if(useORB){
			// The estimated homography will be stored in h. 
			//Mat imReg, h;			   
			// Align images
			//cout << "Aligning images ..." << endl; 
			//alignImages(cur_grey, prev_grey, homog);

			//FIND DOMINANT KEYPOINS
			alignImages(cur_grey, prev_grey, homog, cur_corner, prev_corner);

			//USE only in first image, next image estimate with flow		
			cur_corner.clear();	
			calcOpticalFlowPyrLK(prev_grey.getUMat(ACCESS_RW), cur_grey.getUMat(ACCESS_RW), prev_corner, cur_corner, status, err);		

			// weed out bad matches
			for(size_t i=0; i < status.size(); i++) {
				if(status[i]) {
					prev_corner2.push_back(prev_corner[i]);
					cur_corner2.push_back(cur_corner[i]);
				}
			}		
			homog = findHomography(prev_corner2, cur_corner2,RANSAC) ;//RANSAC);	//CV_LMEDS		
		}else{	
			
			bool useKaze = false;//false; //true
			Ptr<AKAZE> akaze = AKAZE::create();vector<KeyPoint> kpts1;

			//TEST AKAZE result, if zero use another
			if(useKaze){
				//vector<KeyPoint> kpts1;//, kpts1;
				Mat desc1;//, desc1;
				//Ptr<AKAZE> akaze = AKAZE::create();
				akaze->detectAndCompute(prev_grey, noArray(), kpts1, desc1); //akaze->detectAndCompute(prev_grey.getUMat(ACCESS_RW), noArray(), kpts1, desc1);
				//akaze->detectAndCompute(cur_grey.getUMat(ACCESS_RW), noArray(), kpts2, desc2);
				cout << "Points found in kpts1:" << kpts1.size() << endl;
			}


			bool useNewFeatures = false; //find features in new image and use those for homography than the flow estimated ones
			if(useNewFeatures){
				//if(useGpuFeaturesTrack){						
				//	UMat upointsN;					
				//	cv::goodFeaturesToTrack(cur_grey.getUMat(ACCESS_RW), upointsN, cornersMax, 0.01, 30);					
				//	UMatToVector(upointsN, cur_corner3);					
				//}else{
				//	goodFeaturesToTrack(cur_grey, cur_corner3, cornersMax, 0.01, 30);
				//}				
				//cout << prev_corner2.size() << " ," << cur_corner3.size() << endl;				

				//USE ORB TO MATCH
				alignImages(cur_grey, prev_grey, homog, cur_corner, prev_corner);
				cur_corner.clear();				
				if(prev_corner.size() >3){
					calcOpticalFlowPyrLK(prev_grey.getUMat(ACCESS_RW), cur_grey.getUMat(ACCESS_RW), prev_corner, cur_corner, status, err);
					// weed out bad matches
					for(size_t i=0; i < status.size(); i++) {
						if(status[i]) {
							prev_corner2.push_back(prev_corner[i]);
							cur_corner2.push_back(cur_corner[i]);
						}
					}
				}
			}
			else{	
				if(!useKaze || kpts1.size() == 0){
					if(useGpuFeaturesTrack){						
						UMat points, upoints;			
						cv::goodFeaturesToTrack(prev_grey.getUMat(ACCESS_RW), upoints, cornersMax, cornerQuality, minFeatureDistance, noArray(),4 , false, 0.04); 					
						UMatToVector(upoints, prev_corner);					
					}else{
						goodFeaturesToTrack(prev_grey, prev_corner, cornersMax, 0.01, 30);
					}
				}else{
					//vector<KeyPoint> kpts1;//, kpts1;
					//Mat desc1;//, desc1;
					//Ptr<AKAZE> akaze = AKAZE::create();
					//akaze->detectAndCompute(prev_grey, noArray(), kpts1, desc1);
					//akaze->detectAndCompute(cur_grey.getUMat(ACCESS_RW), noArray(), kpts2, desc2);
					//cout << "Points found in kpts1:" << kpts1.size() << endl;
					for(size_t i=0; i < kpts1.size(); i++) {
						prev_corner.push_back(kpts1[i].pt);
					}
				}
		
		//		cout << "Points found in prev:" << prev_corner.size() << endl;

				calcOpticalFlowPyrLK(prev_grey.getUMat(ACCESS_RW), cur_grey.getUMat(ACCESS_RW), prev_corner, cur_corner, status, err);	
				//calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);			

				// weed out bad matches
				for(size_t i=0; i < status.size(); i++) {
					if(status[i]) {
						prev_corner2.push_back(prev_corner[i]);
						cur_corner2.push_back(cur_corner[i]);
					}
				}
	
				//cout << "prev_corner2.size()" << prev_corner2.size() << endl;
				if(prev_corner2.size() > 3){

					//use flow or new features for homogrpahy matching
					bool useFeatures = false;
					if(useFeatures){

						cur_corner.clear();
						//prev_corner.clear();
						//bool useKaze = false;
						if(!useKaze || kpts1.size() == 0){
							if(useGpuFeaturesTrack){						
								UMat upointsN;					
								cv::goodFeaturesToTrack(cur_grey.getUMat(ACCESS_RW), upointsN, cornersMax, cornerQuality, minFeatureDistance, noArray(),4 , false, 0.04);				
								UMatToVector(upointsN, cur_corner);					
							}else{
								cv::goodFeaturesToTrack(cur_grey.getUMat(ACCESS_RW), cur_corner, cornersMax, cornerQuality, minFeatureDistance, noArray(),4 , false, 0.04);
							}
						}else{
							vector<KeyPoint> kpts2;//, kpts1;
							Mat desc2;//, desc1;
							//Ptr<AKAZE> akaze = AKAZE::create();
							//akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
							akaze->detectAndCompute(cur_grey, noArray(), kpts2, desc2); //akaze->detectAndCompute(cur_grey.getUMat(ACCESS_RW), noArray(), kpts2, desc2);
							for(size_t i=0; i < kpts2.size(); i++) {
								cur_corner.push_back(kpts2[i].pt);
							}
						}

			//			cout << "Points found in current:" << cur_corner.size() << endl;

						// weed out bad matches
						int counterA = prev_corner2.size();
						if(cur_corner.size() < prev_corner2.size())
						{
							counterA = cur_corner.size();
						}
						cur_corner4 = cur_corner2; //save previous flow estimated points, to match them with new sizes below
						prev_corner4 =prev_corner2;
						prev_corner2.clear(); cur_corner2.clear();
						for(size_t i=0; i < counterA; i++) {
							//if(status[i]) {
								prev_corner2.push_back(prev_corner4[i]);
								//cur_corner3.push_back(cur_corner[i]);
								cur_corner2.push_back(cur_corner4[i]);
							//}

							//find closest to above
							int closest_ID = 0;
							for(size_t j=0; j < cur_corner.size(); j++) {
								if( sqrt(  pow(prev_corner4[i].x - cur_corner[j].x,2) + pow(prev_corner4[i].y - cur_corner[j].y,2)  ) <  
									sqrt(  pow(prev_corner4[i].x - cur_corner[closest_ID].x,2) + pow(prev_corner4[i].y - cur_corner[closest_ID].y,2)  )		 
								){
									closest_ID=j;
								}
							}
							cur_corner3.push_back(cur_corner[closest_ID]);
						}

						if(cur_corner3.size() > 5){
							homog = findHomography(prev_corner2, cur_corner3, RANSAC);
						}else{
							homog = findHomography(prev_corner2, cur_corner2, RANSAC);
						}
					}else{
						homog = findHomography(prev_corner2, cur_corner2, RANSAC);//RANSAC);	//CV_LMEDS
					}
				}else{
					//keep same			
				}
			}	
		}

		// Now draw the original and stablised side by side for coolness
		Mat canvas = Mat::zeros(cur.rows, cur.cols, cur.type());
		prev.copyTo(canvas(Range::all(), Range(0, cur.cols)));

		//PREVIEW HOMOGRAPHY RESULT
		//bool plotHomography = true;
		Mat canvasF = Mat::zeros(cur.rows, cur.cols * 2 +10, cur.type());
		if(plotHomography){
			prev.copyTo(canvasF(Range::all(), Range(0, cur.cols)));
			cur.copyTo(canvasF(Range::all(), Range(cur.cols * 1, cur.cols * 2)));
		}

		//PLOT POINTS TO DRAW RECTANGLES AROUND
		Mat plotPoints(cur.rows, cur.cols, CV_8UC1, Scalar(0,0,0)); 	




		//cout << "Reached 2" << endl;



		////////////////// PTZ FLOW ADDITION ///////////////////////////////////////////////////////////////////////

				if(useSparkfun){
					if(pozyxOUTPUT.size() > 0){

						//ADD CHOISE TO BE READ EITHER FROM FLIGHT CONTROLLER OR SPARKFUN
						YawRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (1+3)]);
						pitchRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (2+3)]);
						rollRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (3+3)]);

						accelXRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (6+3)]);
						accelYRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (5+3)]);
						accelZRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (4+3)]);

						//Linear velocity from FLIGHT CONTROLLER
						//FClinearVelocityX = ;
						//FClinearVelocityY = ;
						//FClinearVelocityZ = ;
					}
					//cout << "Yaw Rate = " << YawRate << " Pitch Rate  = " << pitchRate << " Zoom Rate = " << zoomRate << endl;
				}else{
						
					//--------------------- ROTATIONS --------------------------

					//double panDelta = panRate * timeDiff;
					//double panDeltaRadians = panDelta * 3.14159265359f / 180.0f;

					//double titlDelta = tiltRate * timeDiff;
					//double tiltDeltaRadians = titlDelta * 3.14159265359f / 180.0f;


					//ADD CHOISE FOR ROTATIONS TO BE READ EITHER FROM FLIGHT CONTROLLER from VICON SAVED FILE OR OUR SAVED LOG FILE (ALSO CHOOSE FROM SPARKFUN OR FLIGHT CONTROLLER RECORDED BY SUBSCRIBED TOPIC)

					//FROM SPARKFUN from our saved file
					if(yawRates.size() > 0 && frameID < yawRates.size()){
						YawRate = yawRates[frameID];//panRate;// currentPan;
						pitchRate = pitchRates[frameID];//tiltRate;//currentTilt;
						rollRate = rollRates[frameID];
					}
					//rollRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (3+3)]);
					//cout << "panDelta = " << YawRate << "titlDelta  = " << pitchRate <<endl;


					//FROM FLIGH CONTROLLER from our saved file


					//FROM FLIGH CONTROLLER from VICON saved file




					//--------------------- TRANSLATIONS --------------------------

					//ADD CHOISE FOR TRANSLATIONS TO BE READ EITHER FROM FLIGHT CONTROLLER from VICON SAVED FILE OR OUR SAVED LOG FILE (ALSO CHOOSE FROM SPARKFUN AFTER ACCEL & GRAV CALCS OR FLIGHT CONTROLLER RECORDED BY SUBSCRIBED TOPIC)
					if(frameID < FClinearVelocityXs.size()){
						FClinearVelocityX = FClinearVelocityXs[frameID-frames_back_sync];
						FClinearVelocityY = FClinearVelocityYs[frameID-frames_back_sync];
						FClinearVelocityZ = FClinearVelocityZs[frameID-frames_back_sync];
					}
				}

				//PTZ FLOW ESTIMATOR -----
				//PTZ flow estimation - ptzEstimatedCorners
				double timeNow = (double)getTickCount();
				//(double)getTickCount() - timer, panRate
				double timeDiff = (timeNow - prevTime) * (1/getTickFrequency());//double timeDiff = ((double)getTickCount() - timer) * 0.001;//convert millisconds to seconds
				//cout << "timeDiff secs = " << timeDiff << "timeNow  = " << timeNow <<"prevTime  = " << prevTime <<endl;
				

				//cout << "Reached 3_0a = " << PTZ_HOMOGRAPHY_FUSION << endl;

				//  1.  ////////////////////////////////////////////// ACCELERATION INTEGRAL //////////////////////////////////// //v0.1
				Vec3f grav;
				double accelXGrav = 0;// (accelXRate-grav[0]);
				double accelYGrav = 0;// (accelYRate-grav[1]);
				double accelZGrav = 0;// (accelZRate-grav[2]);
				double accelXGravSUM = 0;
				double accelYGravSUM = 0;
				double accelZGravSUM = 0;
				if(1==0){		//NO LONGER  DO HERE, DO IN THREAD !!!		
					
					grav = (Vec3f)GravityPerAxis;
					accelXGrav = (accelXRate-grav[0]);
					accelYGrav = (accelYRate-grav[1]);
					accelZGrav = (accelZRate-grav[2]);
					

					//accelXRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (6+3)]);
					//		accelYRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (5+3)]);
					//		accelZRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (4+3)]);
					//Vec3f grav = (Vec3f)GravityPerAxis;
					//double accelXGrav = (accelXRate-grav[0]);
					//double accelYGrav = (accelYRate-grav[1]);
					//double accelZGrav = (accelZRate-grav[2]);

					//cout << "accel X:" << accelXGrav << endl;
					if(abs(accelXGrav) > 0.125){
						//cout << "accel X:" << accelXGrav << endl;
					}else{ accelXGrav=0; }
					if(abs(accelYGrav) > 0.125){
						cout << "accel Y:" << accelYGrav << endl;
					}else{ accelYGrav=0; }
					if(abs(accelZGrav) > 0.125){
						//cout << "accel Z:" << accelZGrav << endl;
					}else{ accelZGrav=0; }
					//cout << "accel Z:" << accelZGrav << endl;

					//Insert samples in buffer
					bufferAccelX.push_back(accelXGrav);
					bufferAccelY.push_back(accelYGrav);
					bufferAccelZ.push_back(accelZGrav);	

					//double accelXGravSUM = 0;
					//double accelYGravSUM = 0;
					//double accelZGravSUM = 0;
					int samplesCount = 1; //TO DO - define based on data rate
					for (int i=0; i < samplesCount; i++){
						accelXGravSUM += bufferAccelX[bufferAccelX.size()-1-i];
						accelYGravSUM += bufferAccelY[bufferAccelY.size()-1-i];
						accelZGravSUM += bufferAccelZ[bufferAccelZ.size()-1-i];
					}
					accelXGravSUM /= samplesCount;
					accelYGravSUM /= samplesCount;
					accelZGravSUM /= samplesCount;

					if(abs(accelXGravSUM) < 0.0198){
						accelXGravSUM=sign(accelXGravSUM)*0.000001;
						velocityXGrav = 0;
					}
					if(abs(accelYGravSUM) < 0.0198){
						accelYGravSUM=sign(accelYGravSUM)*0.000001;
						velocityYGrav = 0;
					}
					accelZGravSUM = accelZGravSUM - 0.02;
					if(abs(accelZGravSUM) < 0.0198){
						accelZGravSUM=sign(accelZGravSUM)*0.000001;
						velocityZGrav = 0;
					}

					//update - integrate speed
					//double velocityXGrav = (accelXRate+grav[0]);
					//double velocityYGrav = (accelYRate+grav[1]);
					//double velocityZGrav = (accelZRate-grav[2]);
					///velocityXGrav = velocityXGrav + accelXGrav * timeDiff;
					///velocityYGrav = velocityYGrav + accelYGrav * timeDiff;
					///velocityZGrav = velocityZGrav + accelZGrav * timeDiff;

					//cout << "Sign accel X:" << sign(accelXGravSUM) << endl;cout << "Sign accel Y:" << sign(accelYGravSUM) << endl;cout << "Sign accel Z:" << sign(accelZGravSUM) << endl;

					//if(abs(accelXGravSUM) > 0 && abs(accelYGravSUM) > 0 && abs(accelZGravSUM) > 0
					//	&& abs(accelXGravSUM) < 10 && abs(accelYGravSUM) < 10 && abs(accelZGravSUM) < 10
					if(	(abs(accelXGravSUM) > 0 && abs(accelXGravSUM) < 30) ||
						(abs(accelYGravSUM) > 0 && abs(accelYGravSUM) < 30) || 
						(abs(accelZGravSUM) > 0 && abs(accelZGravSUM) < 30)
					){
						velocityXGrav = velocityXGrav + accelXGravSUM * timeDiff;
						velocityYGrav = velocityYGrav + accelYGravSUM * timeDiff;
						velocityZGrav = velocityZGrav + accelZGravSUM * timeDiff;

						double dXGrav = 0;
						double dYGrav = 0;
						double dZGrav = 0;
						dXGrav = velocityXGrav * timeDiff * 111;
						dYGrav = velocityYGrav * timeDiff * 111;
						dZGrav = velocityZGrav * timeDiff * 111;

						TRANSLATION_X_Grav += dXGrav*1;
						TRANSLATION_Y_Grav += dYGrav*1;
						TRANSLATION_Z_Grav += dZGrav*1;
					
						if(dXGrav > 0.0000001){
							cout << "dXGrav = " << dXGrav << endl; 
						}
						if(dYGrav > 0.0000001){
							cout << "dYGrav = " << dYGrav << endl; 
						}
						if(dZGrav > 0.0000001){
							cout << "dZGrav = " << dZGrav << endl; 
						}

						//cout << "accel X:" << accelXGravSUM << endl;cout << "accel Y:" << accelYGravSUM << endl;cout << "accel Z:" << accelZGravSUM << endl;
						//cout << "velocityXGrav = " << velocityXGrav << ", velocityYGrav = " << velocityYGrav  << ", velocityZGrav = " << velocityZGrav << endl; 
						//cout << "dXGrav = " << dXGrav << ", dYGrav = " << dYGrav  << ", dZGrav = " << dZGrav << endl; 
						//cout << "sumDX = " << sumDX << ", sumDY = " << sumDY << endl;
						//cout << "X = " << TRANSLATION_X_Grav << endl;
						//cout << "Y = " << TRANSLATION_Y_Grav << endl;
						//cout << "Z = " << TRANSLATION_Z_Grav << endl;
						//cout << "accel X:" << accelXGrav << endl;cout << "accel Y:" << accelYGrav << endl;cout << "accel Z:" << accelZGrav << endl; cout << endl;
						//sumDX, sumDY
					}
				}//END DX_DY_DZ

				//  1 end.  ////////////////////////////////////////////// END ACCELERATION INTEGRAL ////////////////////////////////////


				double panDelta = panRate * timeDiff;
				double panDeltaRadians = panDelta * 3.14159265359f / 180.0f;

				double titlDelta = tiltRate * timeDiff;
				double tiltDeltaRadians = titlDelta * 3.14159265359f / 180.0f;

				//cout << "panDeltaRadians = " << panDeltaRadians << "tiltDeltaRadians  = " << tiltDeltaRadians <<endl;
				//cout << "panDelta = " << panDelta << "titlDelta  = " << titlDelta <<endl;

				//ptzEstimatedCorners.push_back(Point2f(prev_corner2[i].x - 0.00442 * panDeltaRadians * 0.0025,prev_corner2[i].y));
				//ptzEstimatedCorners.push_back(Point2f(prev_corner2[i].x - 0.00442 * panDeltaRadians * 0.1025,prev_corner2[i].y));
				//ptzEstimatedCorners.push_back(Point2f(prev_corner2[i].x - 0.00442 * panDeltaRadians * 0.000005,prev_corner2[i].y));
				//PTZ Optics x20 camera specs - 20x zoom, f4.42mm (no zoom) to 88.5mm (full zoom); F1.8 to F2.8		

				//pan speed 2, 0.000005
				//pan speed 4, 0.15 and plus sign
				//ptzEstimatedCorners.push_back(Point2f(prev_corner2[i].x + 0.00442 * panDeltaRadians * 0.15,prev_corner2[i].y - 0.00442 * tiltDeltaRadians * 0.15));	

				//ADD ZOOM - currentZoom
				//zoomRate
				float zoomDelta = zoomRate * timeDiff;
				float zoomDeltaRadians = zoomDelta * 3.14f / 180.0f;
				//float maxZoom = 16384;
			//	float correctPanTiltByZoomF = currentZoom/maxZoom;
			//	float adjustedF = 0.00442 + correctPanTiltByZoomF * (0.0885-0.00442) * 0.1; //Zoom is 0 to 16384, resize F by the zoom factor

				//corrected with non linear curve
			//	correctPanTiltByZoomF = 1.06 + 0.12 * exp(currentZoom * 3.18 * pow(10,-4));// 3x zoom means focal lenght max / min = 3
				//adjustedF = correctPanTiltByZoomF * (0.00442/1.18) * 1;//0.00442;
			//	adjustedF = correctPanTiltByZoomF * (0.00442/1.18) * 1;//0.00442;	
						
			//cout << "Reached 3_0b = " << PTZ_HOMOGRAPHY_FUSION << endl;

				Mat frame_grey;
				canvas.copyTo(frame_grey);
				for(size_t i=0; i < prev_corner2.size(); i++) {					

					///add a component based on zoom, radially based on distance from screen center, further points are affected more by zoom
					float zoomComponentX = 0.3 * zoomDeltaRadians * (prev_corner2[i].x - frame_grey.cols/2); //0.0000015 for zoom speed 1, 0.0000008 for zoom speed 3
					float zoomComponentY = 0.3 * zoomDeltaRadians * (prev_corner2[i].y - frame_grey.rows/2);
										

					int esimateSimplificationLevel = 1;//5;
					if(useSparkfun || !usePTZ){ //if use PTZ can get rates from Sparkfun or camera PTZ values, if not use PTZ we assume we have Sparkfun accelearation measurements in the offline recorded data file
 						esimateSimplificationLevel = 5;
					}
					
					if(esimateSimplificationLevel == 2){	
						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 9.4) / frame_grey.cols; //0.001 * 9.4m ccd = frame_grey.cols pixels, convert prev_corner2[i].x pixels to meter
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 9.4) / frame_grey.rows;
						float divider = 1*(convertXtoMeters * (-panDeltaRadians) - convertYtoMeters * (tiltDeltaRadians));
						cout << "convertXtoMeters = " << convertXtoMeters << " ,panDeltaRadians = " << panDeltaRadians << endl;
						cout << "convertYtoMeters = " << convertYtoMeters << " ,tiltDeltaRadians = " << tiltDeltaRadians << endl;
						cout << "adjustedF = " << adjustedF << " ,divider = " << divider << endl;
						///divider = 0.0000000000015*divider; 
						//0.0000000000015*(prev_corner2[i].x * (-panDeltaRadians) - prev_corner2[i].y * (tiltDeltaRadians));			
						divider = divider + adjustedF;
						float estX = adjustedF * ((convertXtoMeters - adjustedF * (-panDeltaRadians) * 0.4 + 1 * 0) / divider);
						float estY = adjustedF * ((convertYtoMeters - adjustedF * (tiltDeltaRadians) * 0.5 + 1 * 0) / divider);
						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 9.4);
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 9.4);
						ptzEstimatedCorners.push_back(Point2f(convertestXtoPixels, convertestYtoPixels));
					}
					if(esimateSimplificationLevel == 1){						

						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 5.37) / frame_grey.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 4.04) / frame_grey.rows;
						convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
						convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
						float Xi = convertXtoMeters;
						float Yi = convertYtoMeters;
						float F = adjustedF * 1;
						float Psi = -panDeltaRadians * 0.2; //PAN 0.2 when original image scale 0.8, 0.3 when scale 0.6
						float Thi = -tiltDeltaRadians * 0.3;//tiltDeltaRadians*1; ///TILT 0.3 when original image scale 0.8, 0.4 when scale 0.6
						float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
						float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
						float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
						float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	//( (Yi - F * tan(Thi)*cos(Psi) + Xi * sin(Psi)*tan(Thi)) / divider2);	
						
						estX = estX + ((0.001 * 5.37) / 2);
						estY = estY + ((0.001 * 4.04) / 2);

						//float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 9.4);
						//float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 9.4);		
						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);					
						//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
						ptzEstimatedCorners.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
					}

					//SPARKFUN SENSOR
					if(esimateSimplificationLevel == 4){	
						
						//roll = pozyxOUTPUT.size() - 3
						//pitch = pozyxOUTPUT.size() - 2
						//yaw = pozyxOUTPUT.size() - 1

						//acceleration X,Y,Z = pozyxOUTPUT.size() - 6,-5,-4
	
						//double YawRate = (pozyxOUTPUT[pozyxOUTPUT.size() - 1]) * 1;
						//double pitchRate = (pozyxOUTPUT[pozyxOUTPUT.size() - 2]) * 1;

						double panDeltaA = YawRate * timeDiff;
						double panDeltaRadiansA = panDeltaA * 3.14159265359f / 180.0f;

						double titlDeltaA = pitchRate * timeDiff;
						double tiltDeltaRadiansA = titlDeltaA * 3.14159265359f / 180.0f;

						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 5.37) / frame_grey.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 4.04) / frame_grey.rows;
						convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
						convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
						float Xi = convertXtoMeters;
						float Yi = convertYtoMeters;
						float F = adjustedF * 1;
						float Psi = -panDeltaRadiansA * 1; //PAN 0.2 when original image scale 0.8, 0.3 when scale 0.6
						float Thi = -tiltDeltaRadiansA * 1;//tiltDeltaRadians*1; ///TILT 0.3 when original image scale 0.8, 0.4 when scale 0.6
						float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
						float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
						float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
						float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	//( (Yi - F * tan(Thi)*cos(Psi) + Xi * sin(Psi)*tan(Thi)) / divider2);	
						
						estX = estX + ((0.001 * 5.37) / 2);
						estY = estY + ((0.001 * 4.04) / 2);

						//float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 9.4);
						//float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 9.4);		
						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);					
						//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
						ptzEstimatedCorners.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
					}
					//END SPARKFUN

					//SPARKFUN SENSOR
					if(esimateSimplificationLevel == 5){						
						
						if(i==0){
							//putText(plotFrame, "Roll Pich-Yaw Rate: " + SSTR(rollRate) +","+ SSTR(pitchRate) +","+ SSTR(YawRate) , 
							//Point(300,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
							//putText(plotFrame, "Acceleration Rates: " + SSTR(accelXRate) +","+ SSTR(accelYRate) +","+ SSTR(accelZRate) , 
							//Point(300,180), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
							//circle(plotFrame, Point(plotFrame.cols/2, plotFrame.rows/2), 55,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)), -1);
						}
						double panDeltaA = YawRate * timeDiff;
						double panDeltaRadiansA = panDeltaA * 3.14159265359f / 180.0f;

						double titlDeltaA = pitchRate * timeDiff;
						double tiltDeltaRadiansA = titlDeltaA * 3.14159265359f / 180.0f;

						//Roll Delta
						double rollDeltaA = rollRate * timeDiff;
						double rollDeltaRadiansA = rollDeltaA * 3.14159265359f / 180.0f;

						//Acceleration deltas - minus GravityPerAxis

						if(usePTZ && 1==0){
							

							double accelXDelta = accelXGrav * timeDiff * timeDiff;
							double accelYDelta = accelYGrav * timeDiff * timeDiff;
							double accelZDelta = accelZGrav * timeDiff * timeDiff;//correct for G
							//accelXDelta = velocityXGrav * timeDiff;
							//accelYDelta = velocityYGrav * timeDiff;
							//accelZDelta = velocityZGrav * timeDiff;//correct for G

							// PLOT ACCEL DATA
							cout << "accel X:" << accelXGrav << endl;cout << "accel Y:" << accelYGrav << endl;cout << "accel Z:" << accelZGrav << endl;
							cout << "accelXRate X:" << accelXRate << endl;cout << "accelXRate Y:" << accelYRate << endl;cout << "accelXRate Z:" << accelZRate << endl;
							cout << "grav[0] X:" << grav[0] << endl;cout << "grav[1] Y:" << grav[1] << endl;cout << "grav[2] Z:" << grav[2] << endl;
							//Eulers[0]
							cout << "Eulers[0]:" << Eulers[0] << endl;cout << "Eulers[1]:" << Eulers[1] << endl;cout << "Eulers[2]:" << Eulers[2] << endl;
							cout << endl;
							//double accelXDelta = (accelXRate-0.03) * timeDiff * timeDiff;
							//double accelYDelta = (accelYRate-0.04) * timeDiff * timeDiff;
							//double accelZDelta = (accelZRate-0.96) * timeDiff * timeDiff;//correct for G
							///double rollDeltaRadiansA = rollDeltaA * 3.14159265359f / 180.0f;
						}

						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 5.37) / frame_grey.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 4.04) / frame_grey.rows;
						convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
						convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
						float Xi = convertXtoMeters;
						float Yi = convertYtoMeters;
						float F = adjustedF;
						float Psi = -panDeltaRadiansA; 
						float Thi = -tiltDeltaRadiansA;
						float Phi = rollDeltaRadiansA;
						float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
						float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
						float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
						float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);
						

						//bool useExactRoll = true;
						if(enableRollCompensation){
								if(1==1 && useExactRoll){
									//use linear velocity including equations !!!!!!!!!!!!!!! dXGrav
									//if(1==1 && (FClinearVelocityX != 0 || FClinearVelocityY != 0 || FClinearVelocityZ != 0 )  ) {
									if(1==1 && (dXGrav > 0 || dYGrav > 0)) {
										//roll embedded in exact equation
										//FINAL_ROT2DDIV_X = 
										//(sin(phi)*(sin(thi)*(cos(psi) + (x*sin(psi))/f) + (y*cos(thi))/f) - cos(phi)*(sin(psi) - (x*cos(psi))/f))/(cos(psi)*(cos(thi) - (y*sin(thi) - x*cos(thi)*sin(psi))/(f*cos(psi))))
										//FINAL_ROT2DDIV_Y = 
										//((sin(phi)*(sin(psi) - (x*cos(psi))/f) + cos(phi)*sin(thi)*(cos(psi) + (x*sin(psi))/f))/cos(thi) + (y*cos(phi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f)
										float x = Xi;
										float y = Yi;
										float f = F;
										float psi = Psi; 
										float thi = Thi;
										float phi = Phi;
										float dx = dXGrav; //1*FClinearVelocityX * timeDiff; 
										float dy = dYGrav; //1*FClinearVelocityY * timeDiff; 
										//estX = F*(sin(phi)*(sin(thi)*(cos(psi) + (x*sin(psi))/f) + (y*cos(thi))/f) - cos(phi)*(sin(psi) - (x*cos(psi))/f))/(cos(psi)*(cos(thi) - (y*sin(thi) - x*cos(thi)*sin(psi))/(f*cos(psi))));
										//estY = F*((sin(phi)*(sin(psi) - (x*cos(psi))/f) + cos(phi)*sin(thi)*(cos(psi) + (x*sin(psi))/f))/cos(thi) + (y*cos(phi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);

										//FINAL_ROT2DDIV_X = 
										estX = -F*(cos(phi)*(sin(psi) + cos(psi)*(dx/f - x/f)) - sin(phi)*(sin(thi)*(cos(psi) - sin(psi)*(dx/f - x/f)) - cos(thi)*(dy/f - y/f)))/(cos(psi)*(cos(thi)
										+ (sin(thi)*(dy/f - y/f) - cos(thi)*sin(psi)*(dx/f - x/f))/cos(psi)));
										//FINAL_ROT2DDIV_Y = 
										estY = F*((sin(phi)*(sin(psi) + cos(psi)*(dx/f - x/f)) + cos(phi)*sin(thi)*(cos(psi) - sin(psi)*(dx/f - x/f)))/cos(thi) - cos(phi)*(dy/f - y/f))/((f*cos(psi) 
										- dx*sin(psi) + x*sin(psi))/f + (dy*sin(thi) - y*sin(thi))/(f*cos(thi)));

										//WITHOUT ROLL TO TEST
										//FINAL_ROT2DDIV_X =
										//-(f*sin(psi) - x*cos(psi))/(f*cos(psi)*cos(thi) - y*sin(thi) + x*cos(thi)*sin(psi))
										//FINAL_ROT2DDIV_Y =
										//(cos(psi)*tan(thi) + (y + x*sin(psi)*tan(thi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f)
										//estX =-F*(f*sin(psi) - x*cos(psi))/(f*cos(psi)*cos(thi) - y*sin(thi) + x*cos(thi)*sin(psi));
										//estY =F*(cos(psi)*tan(thi) + (y + x*sin(psi)*tan(thi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);
										cout << "using linear velocity of: " << dx << " , " << dy << endl;
									}
									else{
										float x = Xi;
										float y = Yi;
										float f = F;
										float psi = Psi; 
										float thi = Thi;
										float phi = Phi;
										estX = F*(sin(phi)*(sin(thi)*(cos(psi) + (x*sin(psi))/f) + (y*cos(thi))/f) - cos(phi)*(sin(psi) - (x*cos(psi))/f))/(cos(psi)*(cos(thi) - (y*sin(thi) - x*cos(thi)*sin(psi))/(f*cos(psi))));
										estY = F*((sin(phi)*(sin(psi) - (x*cos(psi))/f) + cos(phi)*sin(thi)*(cos(psi) + (x*sin(psi))/f))/cos(thi) + (y*cos(phi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);
									}


								}else{
									//Roll -------- ROLL
									//estX = estX - rollDeltaRadiansA * estY * (estX*estX + estY*estY);
									//estY = estY + rollDeltaRadiansA * estX * (estX*estX + estY*estY);
									//estX = estX - (((-rollDeltaRadiansA*0.0000005) * Yi) / (Xi*Xi + Yi*Yi));
									//estY = estY + (((-rollDeltaRadiansA*0.0000005) * Xi) / (Xi*Xi + Yi*Yi));

									//DISPLACE points based on actual roll center in camera 3.5cm Y, 0.5cm X
									float XiDISP = Xi - 0.005;
									float YiDISP = Yi + 0.035;

									float squares = Xi*Xi + Yi*Yi;
									float squaresSPEED = XiDISP*XiDISP + YiDISP*YiDISP;
									float rollX = - Yi / squares;
									float rollY =   Xi / squares;
									float magRoll = sqrt(rollX * rollX + rollY * rollY);
									float distCenter = sqrt(squares);
									float distCenterSPEED = sqrt(squaresSPEED);
									//estX = estX - (((-rollDeltaRadiansA*0.0000005) * rollX));
									//estY = estY + (((-rollDeltaRadiansA*0.0000005) * rollY));
									estX = estX + (((-1.0*distCenter*rollDeltaRadiansA*0.67) * (rollX/magRoll)));
									estY = estY + (((-1.8*distCenter*rollDeltaRadiansA*0.67) * (rollY/magRoll)));
									//estX = estX + (((-0.000001*1*rollDeltaRadiansA) * (rollX/1)));
									//estY = estY + (((-0.000001*1*rollDeltaRadiansA) * (rollY/1)));
									//estX = estX + (((-0.05*distCenter*rollDeltaRadiansA) * (rollX/magRoll)));
									//estY = estY + (((-0.05*distCenter*rollDeltaRadiansA) * (rollY/magRoll)));
								}
						}
						//Accelerations
						//estX = estX + 0.5*accelYDelta; //x is frontal movement - zoom, y is positive left, z is positive down in IMU
						///estY = estY - 0.5*accelZDelta;
						//	estX = estX + 0.015*accelYDelta; //x is frontal movement - zoom, y is positive left, z is positive down in IMU
						//	estY = estY - 0.005*accelZDelta;	
						//float zoomComponentXAccel = 0.01 * accelXDelta * (prev_corner2[i].x - frame_grey.cols/2); 
						//float zoomComponentYAccel = 0.01 * accelXDelta * (prev_corner2[i].y - frame_grey.rows/2);					
						//GravityPerAxis

						estX = estX + ((0.001 * 5.37) / 2);
						estY = estY + ((0.001 * 4.04) / 2);
							
						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);						
						ptzEstimatedCorners.push_back(Point2f(convertestXtoPixels + zoomComponentX, convertestYtoPixels + zoomComponentY));
					}
					//END SPARKFUN

				}

				prevTime = (double)getTickCount();

				//cout << "Reached 3" << endl;

				////////////////////////////////////////////// FIND POINTS OF INTEREST
				
				canvas.copyTo(cur2);

				for(size_t i=0; i < cur_corner2.size(); i++) {

					if(plotDebug && plotDebugTracking){
						//circle(canvas, prev_corner2[i], 6,  CV_RGB(42/112, 2/112, 220/121), -1);
					}

					float distEstimatedPTZtoFLOW = 0.075 * sqrt(pow((cur_corner2[i].x - ptzEstimatedCorners[i].x),2) + pow((cur_corner2[i].y - ptzEstimatedCorners[i].y),2));

					float normalizePTZ = sqrt(pow((ptzEstimatedCorners[i].x),2) + pow((ptzEstimatedCorners[i].y),2));
					float normalizeFLOW = sqrt(pow((cur_corner2[i].x),2) + pow((cur_corner2[i].y),2));
					float anglePTZtoFLOW = sqrt(pow(((cur_corner2[i].x/normalizeFLOW) - (ptzEstimatedCorners[i].x/normalizePTZ)),2) + pow(((cur_corner2[i].y/normalizeFLOW) - (ptzEstimatedCorners[i].y/normalizePTZ)),2));


					//PLOT ROLL PITCH YAW
					if(i==0 && plotDebug  && plotDebugTracking && 1==0){
						putText(canvas, "Roll: " + SSTR(abs(rollRate)) + " (" + SSTR(sign(rollRate)) + ")", 
						Point(10,25), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0,11,250), 2);
						putText(canvas, "Pitc: " + SSTR(abs(pitchRate)) + " (" + SSTR(sign(pitchRate)) + ")", 
						Point(10,45), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0,11,250), 2);
						putText(canvas, "Yaw: " + SSTR(abs(YawRate)) + " (" + SSTR(sign(YawRate)) + ")", 
						Point(10,65), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0,11,250), 2);

						putText(canvas, "AcX: " + SSTR(100*abs(accelXRate-0.03)) + " (" + SSTR(sign(accelXRate-0.03)) + ")", 
						Point(10,105), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(220,211,250), 2);
						putText(canvas, "AcY: " + SSTR(100*abs(accelYRate-0.03)) + " (" + SSTR(sign(accelYRate-0.03)) + ")", 
						Point(10,125), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(220,211,250), 2);
						putText(canvas, "AcZ: " + SSTR(100*abs(accelZRate-0.96)) + " (" + SSTR(sign(accelZRate-0.96)) + ")", 
						Point(10,145), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(220,211,250), 2);
						putText(canvas, "Time Delta: " + SSTR(abs(timeDiff)), Point(10,165), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(120,111,250), 2);

						//PLOT TESTING STATE
						//useSparkfun
						//enableTestPanTilt
						//enableTestZoom 
						//enablePIDracking 
						//enablePIDZoom 
						putText(canvas, "Sensor: " + SSTR(useSparkfun) + ", PT:"+ SSTR(enableTestPanTilt)+ ", Z:"+ SSTR(enableTestZoom)+ ", PID:"+ 
						SSTR(enablePIDracking)+ ", PIDZ:"+ SSTR(enablePIDZoom), Point(10,185), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(120,111,250), 2);
						//circle(plotFrame, Point(plotFrame.cols/2, plotFrame.rows/2), 55,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)), -1);
					}

					bool scaleImportant = false;
					
					if(scaleImportant){						
						float directionX = 12 * (cur_corner2[i].x - ptzEstimatedCorners[i].x);
						float directionY = 12 * (cur_corner2[i].y - ptzEstimatedCorners[i].y);
						line(canvas, Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y),Point(cur_corner2[i].x + cur2.cols+10 + directionX, cur_corner2[i].y + directionY), 
						CV_RGB(distEstimatedPTZtoFLOW * 100, distEstimatedPTZtoFLOW * 100, 0));
						circle(canvas, Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), 9 * distEstimatedPTZtoFLOW,  CV_RGB(242, 2/112, 220/121), -1);
						circle(canvas, Point(ptzEstimatedCorners[i].x + cur2.cols+10, ptzEstimatedCorners[i].y), 6 * distEstimatedPTZtoFLOW,  CV_RGB(1, 212, 220), -1);//ptzEstimatedCorners
						//line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y),Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), CV_RGB(255, 255, 0));
					}else{
						if(plotAllPredictedFlowVectors || (distEstimatedPTZtoFLOW > minDistanceToIgnore && anglePTZtoFLOW > minNormalizedDistanceToIgnore))//if(> 0.4) //0.36 works with speed 2 in drone emulator, but not with 1 -- 0.16
						{ //0.8){
							float directionX = 4 * (cur_corner2[i].x - ptzEstimatedCorners[i].x);
							float directionY = 4 * (cur_corner2[i].y - ptzEstimatedCorners[i].y);
							float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;
							//line(canvas, Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y),Point(cur_corner2[i].x + cur2.cols+10 + directionX, cur_corner2[i].y + directionY), 
							//CV_RGB(distEstimatedPTZtoFLOW * 100, distEstimatedPTZtoFLOW * 100, 0));
							//circle(canvas, Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), 9,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
							//circle(canvas, Point(ptzEstimatedCorners[i].x + cur2.cols+10, ptzEstimatedCorners[i].y), 6, (CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/magnitude), -1);

							if(!PTZ_HOMOGRAPHY_FUSION && plotDebug && plotDebugTracking){
								//circle(canvas, Point(cur_corner2[i].x, cur_corner2[i].y), 9,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
								//circle(canvas, Point(ptzEstimatedCorners[i].x, ptzEstimatedCorners[i].y), 6, (CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/magnitude), -1);
							}

							if(distEstimatedPTZtoFLOW > minDistanceToIgnore  && anglePTZtoFLOW > minNormalizedDistanceToIgnore){
								//circle(plotPoints, Point(cur_corner2[i].x, cur_corner2[i].y), 35,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
								ptzBasedFlowCorners.push_back(cur_corner2[i]);
								ptzBasedFlowCorners_MAG.push_back(magnitude);
							}

							///plot two flow esimates
							float directionX1 = 5 * (ptzEstimatedCorners[i].x-prev_corner2[i].x);
							float directionY1 = 5 * (ptzEstimatedCorners[i].y-prev_corner2[i].y);
							float magnitude1 = sqrt(pow(directionX1,2)+pow(directionY1,2))*0.9;
							float directionX2 = 5 * (cur_corner2[i].x-prev_corner2[i].x);
							float directionY2 = 5 * (cur_corner2[i].y-prev_corner2[i].y);
							float magnitude2 = sqrt(pow(directionX2,2)+pow(directionY2,2))*0.9;
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX1, prev_corner2[i].y + directionY1), 
							//CV_RGB(distEstimatedPTZtoFLOW * 100, distEstimatedPTZtoFLOW * 100, 0));
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX2, prev_corner2[i].y + directionY2), 
							//CV_RGB(distEstimatedPTZtoFLOW * 200, distEstimatedPTZtoFLOW * 200, 220));
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX1, prev_corner2[i].y + directionY1), 
							//CV_RGB(1 * 0, 1 * 250, 0));
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX2, prev_corner2[i].y + directionY2), 
							//CV_RGB(250, 1 * 0, 0));

							if(!PTZ_HOMOGRAPHY_FUSION && plotDebug && plotDebugTracking){
								//line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y), Point(prev_corner2[i].x + directionX1, prev_corner2[i].y + directionY1), 
								//CV_RGB(1 * 0, 1 * 250, 0));
								//line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y), Point(prev_corner2[i].x + directionX2, prev_corner2[i].y + directionY2), 
								//CV_RGB(250, 1 * 0, 0));
							}
						}
					}
					//line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y),Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), CV_RGB(215, 215, 215) / distEstimatedPTZtoFLOW);
				}

				////////////////////////////////////////////// END FIND POINTS OF INTEREST

		////////////////// END PTZ FLOW ADDITION ///////////////







		//cout << "Reached 4" << endl;




		//v0.1
		//ESTIMATION of DX, DY, DZ, by the fusion of accelerometer, comparisson of the the homography between image flow to rotated by sensors (pan, tilt, zoom, roll).  
		//TRY FIND HOMOG BETWEN ROTATED and FLOW, assuming the diffeence in homography marix will be only the displacement !!! - TRY with all points first, then only those classiied as background by any method
		Mat homogFLOW_PTZ = homog;//findHomography(ptzEstimatedCorners, cur_corner2, RANSAC);
		//Mat homogFLOW_PTZ = findHomography(cur_corner2, ptzEstimatedCorners,  RANSAC);
		//homog = findHomography(prev_corner2, cur_corner2, RANSAC); //ptzEstimatedCorners
		//DEBUG HOMOG DIRECTIONALITY
		if(1==0 && !homogFLOW_PTZ.empty()){
					double scaler1 =  (homogFLOW_PTZ.at<double>(2,0) * prev_corner2[i].x + homogFLOW_PTZ.at<double>(2,1) * prev_corner2[i].y + homogFLOW_PTZ.at<double>(2,2));
					//double scaleFactor = 500000 * (homogFLOW_PTZ.at<double>(2,0) / pow(prev_corner2[i].x-cur_corner2[i].x,6)  +  homogFLOW_PTZ.at<double>(2,1) / pow(prev_corner2[i].y-cur_corner2[i].y,6) );// + homogFLOW_PTZ.at<double>(2,2);
					//double scaleFactor = 10 * ( (homogFLOW_PTZ.at<double>(2,0) / (homogFLOW_PTZ.at<double>(0,2)*0.01)) + (homogFLOW_PTZ.at<double>(2,1) / (homogFLOW_PTZ.at<double>(1,2)*0.01))  );

					//MOVE FORWARD - OR BACK - dZ
					double scaleFactor_dZx = abs(homogFLOW_PTZ.at<double>(0,2) * 10);//prev_corner2[i].x;
					double scaleFactor_dZy = abs(homogFLOW_PTZ.at<double>(1,2) * 10);//prev_corner2[i].y;
					double scaleFactor = 100000 * (homogFLOW_PTZ.at<double>(2,0) + homogFLOW_PTZ.at<double>(2,1) );
					//double scaleFactor = 100000 * (homogFLOW_PTZ.at<double>(2,0) / scaleFactor_dZx + homogFLOW_PTZ.at<double>(2,1)  / scaleFactor_dZy);

					circle(canvas, 
							Point2f(canvas.cols/2, canvas.rows/2) + 15*Point2f(homogFLOW_PTZ.at<double>(0,2), homogFLOW_PTZ.at<double>(1,2))  / scaler1, 5, CV_RGB(0, 255, 255), -1);				
					line(canvas, Point2f(canvas.cols/2, canvas.rows/2), 
							Point2f(canvas.cols/2, canvas.rows/2) + 15*Point2f(homogFLOW_PTZ.at<double>(0,2), homogFLOW_PTZ.at<double>(1,2))  / scaler1,    CV_RGB(0, 255, 255) , 1, 8,0); 
					//plot depth
					cout << endl;
					//cout << "homogFLOW_PTZ.at<double>(0,2) =" << homogFLOW_PTZ.at<double>(0,2) << "  homogFLOW_PTZ.at<double>(1,2) =" << homogFLOW_PTZ.at<double>(1,2) << endl;
					//cout << "homogFLOW_PTZ.at<double>(2,0) =" << homogFLOW_PTZ.at<double>(2,0) << "  homogFLOW_PTZ.at<double>(2,1) =" << homogFLOW_PTZ.at<double>(2,1) << "  homogFLOW_PTZ.at<double>(2,2) =" << homogFLOW_PTZ.at<double>(2,2) << endl;

 
					circle(canvas, Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, scaleFactor) * 50 / 1, 5, (CV_RGB(255, 0, 0)), -1);				
					line(canvas, Point2f(canvas.cols/2, canvas.rows/2), Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, scaleFactor)* 50 / 1, CV_RGB(255, 0, 0) , 1, 8,0); 


					if( abs(homogFLOW_PTZ.at<double>(2,0)) > 3.2*pow(10,-6) && abs(homogFLOW_PTZ.at<double>(2,1)) > 3.2*pow(10,-6)  ){
						if(homogFLOW_PTZ.at<double>(2,0) > 0 && homogFLOW_PTZ.at<double>(2,1) > 0){
							cout << "Camera moving BACK" << endl;
						}
						if(homogFLOW_PTZ.at<double>(2,0) < 0 && homogFLOW_PTZ.at<double>(2,1) < 0){
							cout << "Camera moving FORWARD" << endl;
						}
					}
					if(1==0){
						if( abs(homogFLOW_PTZ.at<double>(0,2)) > 0.95){
							if(homogFLOW_PTZ.at<double>(0,2) > 0){
								cout << "Camera moving UP" << endl;
							}
							if(homogFLOW_PTZ.at<double>(0,2) < 0){
								cout << "Camera moving DOWN" << endl;
							}
						}
						if( abs(homogFLOW_PTZ.at<double>(1,2)) > 0.95){
							if(homogFLOW_PTZ.at<double>(1,2) > 0){
								cout << "Camera moving LEFT" << endl;
							}
							if(homogFLOW_PTZ.at<double>(1,2) < 0){
								cout << "Camera moving RIGHT" << endl;
							}
						}
					}
		}



		
		vector<Point2f> dronePoints; vector<Point2f> dronePointsFINAL;  vector<float> dronePointsFINAL_MAG;
		vector<Point2f> homogedPoints;
		vector<Point2f> dronePointsPrev;
		//Mat plotPoints(cur.rows, cur.cols, CV_8UC1, Scalar(0,0,0)); 	
		//int maxcountHits = cornersMax / 8;
		//int countHits = 0;

		//v0.1
		//float sumDX = 0; 
		//float sumDY = 0;
		sumDX = 0; 
		sumDY = 0;
		int sumNUMBER=0;

		if(!homog.empty()){


			//DECOMPOSE HOMOGRAPHY 0 //v0.1
			bool homogDecompose0 = false;
			if(homogDecompose0){

				Mat rvec_decomp_Final;
				Mat tvec_decomp_Final;

				vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
				cv::Matx33d cameraMatrix(adjustedF, 	0.0, 		canvas.cols/2,
					       		 0.0, 		adjustedF, 	canvas.rows/2,
					       		 0.0, 		0.0, 		1.0 );
				//[ 9.7269073125437069e+02, 0., 6.1601520799583318e+02, 0.,
      				// 9.6893330177266228e+02, 5.3277120288250023e+02, 0., 0., 1. ]
				//cv::Matx33d cameraMatrix2(9.7269073125437069e+02, 	0.0, 		6.1601520799583318e+02, //canvas.cols/2,
				//	       		 0.0, 		 9.6893330177266228e+02, 	5.3277120288250023e+02, //canvas.rows/2,
				//	       		 0.0, 		0.0, 		1.0);
				double norm = sqrt(homog.at<double>(0,0)*homog.at<double>(0,0) +
				homog.at<double>(1,0)*homog.at<double>(1,0) +
				homog.at<double>(2,0)*homog.at<double>(2,0));
		    		Mat HomogNormalized = homog / norm;
				int solutions = decomposeHomographyMat(HomogNormalized, cameraMatrix, Rs_decomp, ts_decomp, normals_decomp);
				if(1==0){
					cout << "Decompose homography matrix estimated by findHomography():" << HomogNormalized << " HOMOG:" << homog << endl << endl;
					cout << "With camera matrix():" << cameraMatrix << endl << endl;
				}
				for (int i = 0; i < solutions; i++)
				{
					Mat origin(3, 1, CV_64F, Scalar(0));
					Mat origin1 = Rs_decomp[i]*origin + ts_decomp[i];
					double d_inv1 = 1.0 / normals_decomp[i].dot(origin1);

					double factor_d1 = 1.0 / d_inv1;
				      	Mat rvec_decomp;
				      	Rodrigues(Rs_decomp[i], rvec_decomp);
					
					if(1==0){
					      	cout << "Solution " << i << ":" << endl;
					      	cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
					      	//cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
					      	cout << "tvec from homography decomposition: " << ts_decomp[i].t() << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
					      	//cout << "tvec from camera displacement: " << t_1to2.t() << endl;
					      	cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << endl;
					      	//cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
					}

					if( ts_decomp[i].at<double>(0,2) > 0.1){
						rvec_decomp_Final = rvec_decomp;
						tvec_decomp_Final = ts_decomp[i];
						cout << "Solution " << i << ":" << endl;
					      	cout << "rvec from homography decomposition: " << rvec_decomp_Final.t() << endl;					     
					      	cout << "tvec from homography decomposition: " << tvec_decomp_Final.t() << endl;
						cout << endl;
						
						circle(canvas, Point2f(canvas.cols/2, canvas.rows/2) + Point2f( tvec_decomp_Final.at<double>(0,0) , tvec_decomp_Final.at<double>(0,1)) * 5, 5, (CV_RGB(0, 255, 255)), -1);				
						line(canvas, Point2f(canvas.cols/2, canvas.rows/2),  Point2f(canvas.cols/2, canvas.rows/2)+Point2f( tvec_decomp_Final.at<double>(0,0) , tvec_decomp_Final.at<double>(0,1)) * 5, CV_RGB(0, 255, 255) , 1, 8,0);

						circle(canvas, Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, tvec_decomp_Final.at<double>(0,2))* 50, 5, (CV_RGB(255, 0, 0)), -1);				
						line(canvas, Point2f(canvas.cols/2, canvas.rows/2), Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, tvec_decomp_Final.at<double>(0,2))* 50, CV_RGB(255, 0, 0) , 1, 8,0);
					}
				}			
			}


			//calculate the rotated points and plot them			
			for(size_t i=0; i < prev_corner2.size(); i++) {				

				//find rotated			
				float rotatedX = prev_corner2[i].x;
				float rotatedY = prev_corner2[i].y;

				rotatedX = (homog.at<double>(0,0) * rotatedX) + (homog.at<double>(0,1) *  rotatedY);
				rotatedY = (homog.at<double>(1,0) * rotatedX) + (homog.at<double>(1,1) *  rotatedY);
				
				rotatedX = rotatedX + 1.0*homog.at<double>(0,2);
				rotatedY = rotatedY + 1.0*homog.at<double>(1,2);				
				
				rotatedX = rotatedX / (homog.at<double>(2,0) * prev_corner2[i].x + homog.at<double>(2,1) * prev_corner2[i].y + homog.at<double>(2,2));
				rotatedY = rotatedY / (homog.at<double>(2,0) * prev_corner2[i].x + homog.at<double>(2,1) * prev_corner2[i].y + homog.at<double>(2,2));


				//DEBUG HOMOG DIRECTIONALITY
				if(1==0){
					double scaler1 =  (homog.at<double>(2,0) * prev_corner2[i].x + homog.at<double>(2,1) * prev_corner2[i].y + homog.at<double>(2,2));
					//double scaleFactor = 500000 * (homog.at<double>(2,0) / pow(prev_corner2[i].x-cur_corner2[i].x,6)  +  homog.at<double>(2,1) / pow(prev_corner2[i].y-cur_corner2[i].y,6) );// + homog.at<double>(2,2);
					double scaleFactor = 10 * ( (homog.at<double>(2,0) / (homog.at<double>(0,2)*0.01)) + (homog.at<double>(2,1) / (homog.at<double>(1,2)*0.01))  );
					circle(canvas, Point2f(canvas.cols/2, canvas.rows/2) + Point2f(homog.at<double>(0,2), homog.at<double>(1,2)) * 50 / 1, 5, (CV_RGB(0, 255, 255)), -1);				
					line(canvas, Point2f(canvas.cols/2, canvas.rows/2), Point2f(canvas.cols/2, canvas.rows/2) + Point2f(homog.at<double>(0,2), homog.at<double>(1,2))* 50 / 1,
					CV_RGB(0, 255, 255) , 1, 8,0); 
					//plot depth
					cout << endl;
					//cout << "homog.at<double>(0,2) =" << homog.at<double>(0,2) << "  homog.at<double>(1,2) =" << homog.at<double>(1,2) << endl;
					//cout << "homog.at<double>(2,0) =" << homog.at<double>(2,0) << "  homog.at<double>(2,1) =" << homog.at<double>(2,1) << "  homog.at<double>(2,2) =" << homog.at<double>(2,2) << endl;
					circle(canvas, Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, scaleFactor) * 50 / 1, 5, (CV_RGB(255, 0, 0)), -1);				
					line(canvas, Point2f(canvas.cols/2, canvas.rows/2), Point2f(canvas.cols/2, canvas.rows/2) + Point2f(0, scaleFactor)* 50 / 1, CV_RGB(255, 0, 0) , 1, 8,0); 
					if( abs(homog.at<double>(2,0)) > 3.2*pow(10,-6) && abs(homog.at<double>(2,1)) > 3.2*pow(10,-6)  ){
						if(homog.at<double>(2,0) > 0 && homog.at<double>(2,1) > 0){
							cout << "Camera moving BACK" << endl;
						}
						if(homog.at<double>(2,0) < 0 && homog.at<double>(2,1) < 0){
							cout << "Camera moving FORWARD" << endl;
						}
					}
					if( abs(homog.at<double>(0,2)) > 0.95){
						if(homog.at<double>(0,2) > 0){
							cout << "Camera moving UP" << endl;
						}
						if(homog.at<double>(0,2) < 0){
							cout << "Camera moving DOWN" << endl;
						}
					}
					if( abs(homog.at<double>(1,2)) > 0.95){
						if(homog.at<double>(1,2) > 0){
							cout << "Camera moving LEFT" << endl;
						}
						if(homog.at<double>(1,2) < 0){
							cout << "Camera moving RIGHT" << endl;
						}
					}
				}


				//float distEstimatedPTZtoFLOW = 0.075 * sqrt(pow((cur_corner2[i].x - rotatedX),2) + pow((cur_corner2[i].y - rotatedY),2));
				float distEstimatedPTZtoFLOW = sqrt(pow((cur_corner2[i].x - rotatedX),2) + pow((cur_corner2[i].y - rotatedY),2));
				
				//if(distEstimatedPTZtoFLOW > 3 && distEstimatedPTZtoFLOW < 9)//if(distEstimatedPTZtoFLOW > 0.45)
				float vectorPower = sqrt( (prev_corner2[i].x-cur_corner2[i].x)*(prev_corner2[i].x-cur_corner2[i].x) 
							+ (prev_corner2[i].y-cur_corner2[i].y)*(prev_corner2[i].y-cur_corner2[i].y)) ;
				float diffEstimatesToPrev = abs(vectorPower
						- sqrt( (prev_corner2[i].x-rotatedX)*(prev_corner2[i].x-rotatedX) + (prev_corner2[i].y-rotatedY)*(prev_corner2[i].y-rotatedY))
						);
				
				Point2f p1 = 1000 * Point2f(cur_corner2[i].x-prev_corner2[i].x, cur_corner2[i].y-prev_corner2[i].y);
				Point2f p2 = 1000 * Point2f(rotatedX-prev_corner2[i].x, rotatedY-prev_corner2[i].y);
				//cout << "p1: " << p1 << endl;
				//cout << "p2: " << p2 << endl;
				float angleDiff = (180 / CVA_PI) * angleBetween(p1, p2);
				//cout << "Angle Difference o: " << angleDiff << endl;

				bool isForeground = false;//v0.1
				
				if(plotHomography && displayall){
					line(canvasF, Point(prev_corner2[i].x, prev_corner2[i].y), Point(rotatedX + cur.cols, rotatedY),CV_RGB(255/2, 255/2, 255/2) , 1, 8,0); 
					if(plotHomographyTEXT){
						putText(canvasF, SSTR(homog.at<double>(1,2)), Point(prev_corner2[i].x, prev_corner2[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
					}
					circle(canvasF, Point(cur_corner2[i].x + cur.cols, cur_corner2[i].y), 6,( CV_RGB(0, 55-112, 55-5)), -1);
					circle(canvasF, Point(prev_corner2[i].x, prev_corner2[i].y), 6,(CV_RGB(50, 1, 1)), -1);
				}

				//bool displayall = false;
				if(displayall ||(distEstimatedPTZtoFLOW > minDistPTZ_FLOW && distEstimatedPTZtoFLOW < maxDistPTZ_FLOW 
				&& (diffEstimatesToPrev > diffEstimatesToPrevMIN && diffEstimatesToPrev < diffEstimatesToPrevMAX) 
				&&  (angleDiff > angleMin && angleDiff < angleMax)    )
				&& vectorPower > minVelocityMagnitude)
				//distEstimatedPTZtoFLOW < 64 && diffEstimatesToPrev < 20 && angleDiff > 4)) //24-12
				//if(distEstimatedPTZtoFLOW > 2 && distEstimatedPTZtoFLOW < 32 && diffEstimatesToPrev < 2)
				//if(distEstimatedPTZtoFLOW > 3 && distEstimatedPTZtoFLOW < 22 && diffEstimatesToPrev < 2)
				{ 

						float directionX = 1 * (cur_corner2[i].x - rotatedX);
						float directionY = 1 * (cur_corner2[i].y - rotatedY);
						float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;
					


					//cut near screen edge points
					bool nearScreenEdge = false;
					int offset = ignorePixelsNearEdge * scaler;
					if(cur_corner2[i].x < offset || cur_corner2[i].x > canvas.cols - offset  //|| cur_corner2[i].x + boundRect[i].width > canvas.cols - offset 
					|| cur_corner2[i].y < offset || cur_corner2[i].y > canvas.rows - offset  //|| cur_corner2[i].y + boundRect[i].height > canvas.rows - offset
					|| prev_corner2[i].x < offset || prev_corner2[i].x > canvas.cols - offset 
					|| prev_corner2[i].y < offset || prev_corner2[i].y > canvas.rows - offset 
					){							
								nearScreenEdge = true;		
					}

					if(!nearScreenEdge && (!use2ondBackLayer || dronePoints.size() <= 5)){
						//countHits++;

						

						if(plotDebugTracking){								
							circle(canvas, Point(cur_corner2[i].x , cur_corner2[i].y), 7,( CV_RGB(255-242/magnitude,255- 50/magnitude,255- 2/magnitude)), -1);
							// cyan - closer to black - optical flow
							circle(canvas, Point(rotatedX, rotatedY), 5, ( CV_RGB( 255-1/magnitude,255- 212/magnitude, 255-220/magnitude)), -1); 
							//roz - kokkino homography					
					
							//flow vectors
							line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y), Point(prev_corner2[i].x, prev_corner2[i].y)
							+vectorLength*Point(cur_corner2[i].x-prev_corner2[i].x, cur_corner2[i].y-prev_corner2[i].y),
							CV_RGB(0, 255, 255) , 1, 8,0); 
							line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y), Point(prev_corner2[i].x, prev_corner2[i].y)
							+vectorLength*Point(rotatedX-prev_corner2[i].x, rotatedY-prev_corner2[i].y),CV_RGB(255, 1, 1) , 1, 8,0);
						}
							
						//plot to get rectangles PLOT DOTS !!!!
						if(distEstimatedPTZtoFLOW > minDistPTZ_FLOW && distEstimatedPTZtoFLOW < maxDistPTZ_FLOW && (diffEstimatesToPrev > diffEstimatesToPrevMIN && diffEstimatesToPrev < diffEstimatesToPrevMAX)  //diffEstimatesToPrev < 24  
						//&& dronePoints.size() > 0 
						&& (angleDiff > angleMin && angleDiff < angleMax)  ){


							

							if(cur_corner2[i].y < plotPoints.rows - lowerCutoff){

								if(!cutPropellers || 
									(cutPropellers && (
										(allULout || pointPolygonTest(upperLeftPropellerContour, Point2f(cur_corner2[i].x,cur_corner2[i].y), false) < 0)
										&& (allLLout || pointPolygonTest(lowerLeftPropellerContour, Point2f(cur_corner2[i].x,cur_corner2[i].y), false) < 0)
										&& (allURout || pointPolygonTest(upperRightPropellerContour, Point2f(cur_corner2[i].x,cur_corner2[i].y), false) < 0)
										&& (allLRout || pointPolygonTest(lowerRightPropellerContour, Point2f(cur_corner2[i].x,cur_corner2[i].y), false) < 0)
										)
								  	) 
								){
									//cout << "plotttted" << endl;
									if(!PTZ_HOMOGRAPHY_FUSION){
										circle(plotPoints, Point(cur_corner2[i].x,cur_corner2[i].y), 30 * scaleFrame * (1/0.85) * dotPlotScaler, 
										(CV_RGB(255-242/magnitude,255- 50/magnitude, 255-2/magnitude)), -1);
									}
									dronePointsFINAL_MAG.push_back(magnitude);
									dronePoints.push_back(Point(cur_corner2[i].x , cur_corner2[i].y));
									dronePointsPrev.push_back(Point(prev_corner2[i].x , prev_corner2[i].y));
									homogedPoints.push_back(Point(rotatedX , rotatedY));
									dronePointsFINAL.push_back(Point(cur_corner2[i].x , cur_corner2[i].y));

									isForeground = true;//v0.1



									//CanvasFULL
									if(plotHomography){
										line(canvasF, Point(prev_corner2[i].x, prev_corner2[i].y), Point(rotatedX + cur.cols, rotatedY),CV_RGB(255, 255, 255) , 1, 8,0); 
										if(plotHomographyTEXT){
											putText(canvasF, SSTR(homog.at<double>(1,2)), Point(prev_corner2[i].x, prev_corner2[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
										}
										circle(canvasF, Point(cur_corner2[i].x + cur.cols, cur_corner2[i].y), 6,(CV_RGB(255-242/magnitude,255- 50/magnitude, 255-2/magnitude)), -1);
										circle(canvasF, Point(prev_corner2[i].x, prev_corner2[i].y), 6,(CV_RGB(255, 1, 1)), -1);
									}
								}
							}
						}

						
					}else{
						//dronePointsFINAL_MAG.push_back(magnitude);
						dronePoints.push_back(Point(cur_corner2[i].x , cur_corner2[i].y));
						dronePointsPrev.push_back(Point(prev_corner2[i].x , prev_corner2[i].y));
						homogedPoints.push_back(Point(rotatedX , rotatedY));
						//dronePointsFINAL.push_back(Point(cur_corner2[i].x , cur_corner2[i].y));
					}
				}

				//v0.1 - find directionality
				if(!isForeground && abs(diffEstimatesToPrev) < 0.01){	//if(!isForeground && abs(diffEstimatesToPrev) < 0.0001){
					//See how background moves based on HOMOGRAPHY
					float directionPX = 1 * (cur_corner2[i].x - prev_corner2[i].x);
					float directionPY = 1 * (cur_corner2[i].y - prev_corner2[i].y);
					//float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;

					float dividerA = (homog.at<double>(2,0) * prev_corner2[i].x + homog.at<double>(2,1) * prev_corner2[i].y + homog.at<double>(2,2));
					float transfX = homog.at<double>(0,2) / dividerA;
					float transfY = homog.at<double>(1,2) / dividerA;				

					sumDX = sumDX + directionPX;
					sumDY = sumDY + directionPY;
					//sumDX = sumDX + pow(transfX,1);
					//sumDY = sumDY + pow(transfY,1);
					sumNUMBER++;

					//v0.2 - remove effect due to the rotations  
					//SPARKFUN SENSOR
					//ADD ZOOM - currentZoom
					//zoomRate
					float zoomDelta = zoomRate * timeDiff;
					float zoomDeltaRadians = zoomDelta * 3.14f / 180.0f;
					///add a component based on zoom, radially based on distance from screen center, further points are affected more by zoom
					float zoomComponentX = 0.3 * zoomDeltaRadians * (prev_corner2[i].x - frame_grey.cols/2); //0.0000015 for zoom speed 1, 0.0000008 for zoom speed 3
					float zoomComponentY = 0.3 * zoomDeltaRadians * (prev_corner2[i].y - frame_grey.rows/2);
					if(5 == 5){ //if(esimateSimplificationLevel == 5){					
						
						double panDeltaA = YawRate * timeDiff;
						double panDeltaRadiansA = panDeltaA * 3.14159265359f / 180.0f;

						double titlDeltaA = pitchRate * timeDiff;
						double tiltDeltaRadiansA = titlDeltaA * 3.14159265359f / 180.0f;

						//Roll Delta
						double rollDeltaA = rollRate * timeDiff;
						double rollDeltaRadiansA = rollDeltaA * 3.14159265359f / 180.0f;					

						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 5.37) / frame_grey.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 4.04) / frame_grey.rows;
						convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
						convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
						float Xi = convertXtoMeters;
						float Yi = convertYtoMeters;
						float F = adjustedF;
						float Psi = -panDeltaRadiansA; 
						float Thi = -tiltDeltaRadiansA;
						float Phi = rollDeltaRadiansA;
						float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
						float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
						float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
						float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);						

						//bool useExactRoll = true;
						if(1==1 || enableRollCompensation){
							if(1==1 || useExactRoll){									
								float x = Xi;
								float y = Yi;
								float f = F;
								float psi = Psi; 
								float thi = Thi;
								float phi = Phi;
								estX = F*(sin(phi)*(sin(thi)*(cos(psi) + (x*sin(psi))/f) + (y*cos(thi))/f) - cos(phi)*(sin(psi) - (x*cos(psi))/f))/(cos(psi)*(cos(thi) - (y*sin(thi) - x*cos(thi)*sin(psi))/(f*cos(psi))));
								estY = F*((sin(phi)*(sin(psi) - (x*cos(psi))/f) + cos(phi)*sin(thi)*(cos(psi) + (x*sin(psi))/f))/cos(thi) + (y*cos(phi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);
							}
						}						

						estX = estX + ((0.001 * 5.37) / 2);
						estY = estY + ((0.001 * 4.04) / 2);
							
						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37) + zoomComponentX;
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04) + zoomComponentY;						
						//ptzEstimatedCorners.push_back(Point2f(convertestXtoPixels + zoomComponentX, convertestYtoPixels + zoomComponentY));
						float directionPTZ_X = 1 * (convertestXtoPixels - prev_corner2[i].x);
						float directionPTZ_Y = 1 * (convertestYtoPixels - prev_corner2[i].y);

						//cout << "directionPTZ_X = " << directionPTZ_X << ", directionPTZ_Y = " << directionPTZ_Y << endl;

						float thres = 2.5 * resMultiplier * 0.5;//1.5;
						if( abs(directionPTZ_X) > thres || abs(directionPTZ_Y) > thres){
							//cout << "Displacement due to rotations X =" << directionPTZ_X << ", Y =" << directionPTZ_Y << endl;
							//sumDX = sumDX - directionPTZ_X;
							//sumDY = sumDY - directionPTZ_Y;
						}
						if( abs(directionPTZ_X) > thres){							
							sumDX = sumDX / (15);	sumDY = sumDY / (4);					
						}
						if( abs(directionPTZ_Y) > thres){							
							sumDX = sumDX / (4);	sumDY = sumDY / (15);							
						}

						//sumDX = sumDX / directionPTZ_X;
						//sumDY = sumDY / directionPTZ_Y;
						//sumDX = sumDX - directionPTZ_X;
						//sumDY = sumDY - directionPTZ_Y;
					}
					//END SPARKFUN
					
					
				}//END v0.1

				if(plotDebugTracking){				
					circle(canvas, Point(prev_corner2[i].x , prev_corner2[i].y), 2,  CV_RGB(42/112, 1, 2), -1);
				}
			}

			

			//v0.1 - show DX, DY esimate
			sumDX = sumDX / (float)sumNUMBER;
			sumDY = sumDY / (float)sumNUMBER;

			if(abs(sumDX) > 60){
				sumDX = 0;
			}
			if(abs(sumDY) > 60){
				sumDY = 0;
			}

			if(plotDebug && (sumDX != 0 || sumDY != 0)){
					//circle(canvas, 
					//Point2f(canvas.cols/2, canvas.rows/2) + 50*Point2f(sumDX, sumDY)  / (float)sumNUMBER, 5, CV_RGB(0, 255, 255), -1);				
					//line(canvas, Point2f(canvas.cols/2, canvas.rows/2), 
					//Point2f(canvas.cols/2, canvas.rows/2) + 50*Point2f(sumDX, sumDY)  / (float)sumNUMBER,    CV_RGB(0, 255, 255) , 1, 8,0); 
					circle(canvas, 
					Point2f(canvas.cols/2, canvas.rows/2) + 50*Point2f(sumDX, sumDY), 5, CV_RGB(0, 255, 255), -1);				
					line(canvas, Point2f(canvas.cols/2, canvas.rows/2), 
					Point2f(canvas.cols/2, canvas.rows/2) + 50*Point2f(sumDX, sumDY),    CV_RGB(0, 255, 255) , 1, 8,0);
			}

			//DECOMPOSE HOMOGRAPHY //v0.1
			bool homogDecompose = false;
			if(homogDecompose){
				vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
				cv::Matx33d cameraMatrix(adjustedF, 	0.0, 		canvas.cols/2,
					       		 0.0, 		adjustedF, 	canvas.rows/2,
					       		 0.0, 		0.0, 		1.0 );
				//[ 9.7269073125437069e+02, 0., 6.1601520799583318e+02, 0.,
      				// 9.6893330177266228e+02, 5.3277120288250023e+02, 0., 0., 1. ]
				//cv::Matx33d cameraMatrix2(9.7269073125437069e+02, 	0.0, 		6.1601520799583318e+02, //canvas.cols/2,
				//	       		 0.0, 		 9.6893330177266228e+02, 	5.3277120288250023e+02, //canvas.rows/2,
				//	       		 0.0, 		0.0, 		1.0);
				double norm = sqrt(homog.at<double>(0,0)*homog.at<double>(0,0) +
				homog.at<double>(1,0)*homog.at<double>(1,0) +
				homog.at<double>(2,0)*homog.at<double>(2,0));
		    		Mat HomogNormalized = homog / norm;
				int solutions = decomposeHomographyMat(HomogNormalized, cameraMatrix, Rs_decomp, ts_decomp, normals_decomp);
				cout << "Decompose homography matrix estimated by findHomography():" << HomogNormalized << " HOMOG:" << homog << endl << endl;
				cout << "With camera matrix():" << cameraMatrix << endl << endl;
				for (int i = 0; i < solutions; i++)
				{
					Mat origin(3, 1, CV_64F, Scalar(0));
					Mat origin1 = Rs_decomp[i]*origin + ts_decomp[i];
					double d_inv1 = 1.0 / normals_decomp[i].dot(origin1);

					double factor_d1 = 1.0 / d_inv1;
				      	Mat rvec_decomp;
				      	Rodrigues(Rs_decomp[i], rvec_decomp);
				      	cout << "Solution " << i << ":" << endl;
				      	cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
				      	//cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
				      	cout << "tvec from homography decomposition: " << ts_decomp[i].t() << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
				      	//cout << "tvec from camera displacement: " << t_1to2.t() << endl;
				      	cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << endl;
				      	//cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
				}			
			}

		}


		
		//IDENTIFY 2ond BACKDROUND  LAYER		
		if(dronePoints.size() > 5 && use2ondBackLayer){

			dronePointsFINAL_MAG.clear();
			dronePointsFINAL.clear();

			//cout << "dronePointsPrev count = " << dronePointsPrev.size() << ", dronePoints count = " << dronePoints.size() << endl; 
			homog = findHomography(dronePointsPrev, dronePoints, RANSAC);//homog = findHomography(dronePoints, homogedPoints, RANSAC); //dronePointsPrev

			//ERASE PREVIOUS
			//plotPoints = cv::Mat::zeros(plotPoints.size(), plotPoints.type());

			//check if homog empty
			if (homog.empty())
			{
				cout << "homog empty, trying more ..." << endl;
				homog = findHomography(dronePointsPrev, dronePoints, LMEDS); //use other method to avoid empty matrix
			}
			if (homog.empty())
			{
				cout << "homog empty, trying more ......" << endl;
				homog = findHomography(dronePointsPrev, dronePoints, 0); //use other method to avoid empty matrix
			}
			
			for(size_t i=0; i < dronePointsPrev.size(); i++) {

				if (homog.empty()){
					break;
				}

				//find rotated			
				float rotatedX = dronePointsPrev[i].x;
				float rotatedY = dronePointsPrev[i].y;

				rotatedX = (homog.at<double>(0,0) * rotatedX) + (homog.at<double>(0,1) *  rotatedY);
				rotatedY = (homog.at<double>(1,0) * rotatedX) + (homog.at<double>(1,1) *  rotatedY);
				
				rotatedX = rotatedX + 1.0*homog.at<double>(0,2);
				rotatedY = rotatedY + 1.0*homog.at<double>(1,2);			
				
				rotatedX = rotatedX / (homog.at<double>(2,0) * dronePointsPrev[i].x + homog.at<double>(2,1) * dronePointsPrev[i].y + homog.at<double>(2,2));
				rotatedY = rotatedY / (homog.at<double>(2,0) * dronePointsPrev[i].x + homog.at<double>(2,1) * dronePointsPrev[i].y + homog.at<double>(2,2));
				
				float distEstimatedPTZtoFLOW = sqrt(pow((dronePoints[i].x - rotatedX),2) + pow((dronePoints[i].y - rotatedY),2));		

				float diffEstimatesToPrev = 	abs(sqrt( (dronePointsPrev[i].x-dronePoints[i].x)*(dronePointsPrev[i].x-dronePoints[i].x) 
				+ (dronePointsPrev[i].y-dronePoints[i].y)*(dronePointsPrev[i].y-dronePoints[i].y)) 
				- sqrt( (dronePointsPrev[i].x-rotatedX)*(dronePointsPrev[i].x-rotatedX) + (dronePointsPrev[i].y-rotatedY)*(dronePointsPrev[i].y-rotatedY))
				);				

				Point2f p1 = 1000 * Point2f(dronePoints[i].x-dronePointsPrev[i].x, dronePoints[i].y-dronePointsPrev[i].y);
				Point2f p2 = 1000 * Point2f(rotatedX-dronePointsPrev[i].x, rotatedY-dronePointsPrev[i].y);
				//cout << "p1: " << p1 << endl;
				//cout << "p2: " << p2 << endl;
				float angleDiff = (180 / CVA_PI) * angleBetween(p1, p2);				



				if(plotHomography && displayall){
					line(canvasF, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(rotatedX + cur.cols, rotatedY),CV_RGB(255/2, 255/2, 255/2) , 1, 8,0); 
					if(plotHomographyTEXT){
						putText(canvasF, SSTR(homog.at<double>(1,2)), Point(dronePointsPrev[i].x, dronePointsPrev[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
					}
					circle(canvasF, Point(dronePoints[i].x + cur.cols, dronePoints[i].y), 6,(CV_RGB(255/5-242, 255/5-2, 255/5-1)), -1);
					circle(canvasF, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), 6,(CV_RGB(255/5, 1, 1)), -1);
				}

				//bool displayall = false;
				if(displayall ||(distEstimatedPTZtoFLOW > minDistPTZ_FLOW2 && distEstimatedPTZtoFLOW < maxDistPTZ_FLOW - 5 
				&& diffEstimatesToPrev < diffEstimatesToPrevMAX+1 && (angleDiff > angleMin && angleDiff < angleMax)  ))
				//74 && diffEstimatesToPrev < 29 && angleDiff > 9))	//24 && diffEstimatesToPrev < 9			
				{ 		

					//cut near screen edge points
					bool nearScreenEdge = false;
					int offset = ignorePixelsNearEdge * scaler;
					if(dronePoints[i].x < offset || dronePoints[i].x > canvas.cols - offset  //|| cur_corner2[i].x + boundRect[i].width > canvas.cols - offset 
					|| dronePoints[i].y < offset || dronePoints[i].y > canvas.rows - offset  //|| cur_corner2[i].y + boundRect[i].height > canvas.rows - offset
					|| dronePointsPrev[i].x < offset || dronePointsPrev[i].x > canvas.cols - offset 
					|| dronePointsPrev[i].y < offset || dronePointsPrev[i].y > canvas.rows - offset 
					){							
								nearScreenEdge = true;		
					}
					
					if(!nearScreenEdge){
						float directionX = 1 * (dronePoints[i].x - rotatedX);
						float directionY = 1 * (dronePoints[i].y - rotatedY);
						float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;
					
						if(plotDebugTracking){								
							circle(canvas, Point(dronePoints[i].x , dronePoints[i].y), 7,( CV_RGB(255-242/magnitude,255- 2/magnitude, 255-2/magnitude)), -1);
							// cyan - closer to black - optical flow
							circle(canvas, Point(rotatedX, rotatedY), 5, ( CV_RGB(255-1/magnitude,255- 212/magnitude,255- 220/magnitude)), -1); 
							//roz - kokkino homography				

							//flow vectors
							line(canvas, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(dronePointsPrev[i].x, dronePointsPrev[i].y)
							+vectorLength*Point(dronePoints[i].x-dronePointsPrev[i].x, dronePoints[i].y-dronePointsPrev[i].y),
							CV_RGB(0, 255, 255) , 1, 8,0); 
							line(canvas, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(dronePointsPrev[i].x, dronePointsPrev[i].y)
							+vectorLength*Point(rotatedX-dronePointsPrev[i].x, rotatedY-dronePointsPrev[i].y),
							CV_RGB(255, 1, 1) , 1, 8,0); 
						}

						//dronePoints.push_back(Point(cur_corner2[i].x , cur_corner2[i].y));
						//homogedPoints.push_back(Point(rotatedX , rotatedY));
							
						//plot to get rectangles PLOT DOTS !!!
						if((distEstimatedPTZtoFLOW > minDistPTZ_FLOW2 && distEstimatedPTZtoFLOW < maxDistPTZ_FLOW - 5 && diffEstimatesToPrev < diffEstimatesToPrevMAX+1))
						//44 && diffEstimatesToPrev < 19))  //< 24 && diffEstimatesToPrev < 9
						{
							if(dronePoints[i].y < plotPoints.rows - lowerCutoff){

								if(!cutPropellers || 
									(cutPropellers && (
										(allULout || pointPolygonTest(upperLeftPropellerContour, Point2f(dronePoints[i].x,dronePoints[i].y), false) < 0)
										&& (allLLout || pointPolygonTest(lowerLeftPropellerContour, Point2f(dronePoints[i].x,dronePoints[i].y), false) < 0)
										&& (allURout || pointPolygonTest(upperRightPropellerContour, Point2f(dronePoints[i].x,dronePoints[i].y), false) < 0)
										&& (allLRout || pointPolygonTest(lowerRightPropellerContour, Point2f(dronePoints[i].x,dronePoints[i].y), false) < 0)
										)
								  	) 
								){
									if(!PTZ_HOMOGRAPHY_FUSION){
										circle(plotPoints, Point(dronePoints[i].x,dronePoints[i].y), 40 * scaleFrame * (1/0.85) * dotPlotScaler, 
										( CV_RGB(255-242,255- 1/magnitude, 255-3/magnitude)), -1);
									}
									dronePointsFINAL_MAG.push_back(magnitude);
									dronePointsFINAL.push_back(Point(dronePoints[i].x , dronePoints[i].y));


									//cout << "Element in 2ond parsing ... " << i << endl;
									//CanvasFULL
									if(plotHomography){
										line(canvasF, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(rotatedX + cur.cols, rotatedY),CV_RGB(255, 255, 255) , 1, 8,0); 
										if(plotHomographyTEXT){
											putText(canvasF, SSTR(homog.at<double>(1,2)), Point(dronePointsPrev[i].x, dronePointsPrev[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
										}
										circle(canvasF, Point(dronePoints[i].x + cur.cols, dronePoints[i].y), 6,(CV_RGB(255-242/magnitude,255- 1/magnitude, 255-3/magnitude)), -1);
										circle(canvasF, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), 6,(CV_RGB(255, 1, 1)), -1);
									}
								}
							}
						}
					
						
					}
				}	
				else if(1==0){
					float directionX = 1 * (dronePoints[i].x - rotatedX);
					float directionY = 1 * (dronePoints[i].y - rotatedY);
					float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;								
					circle(canvas, Point(dronePoints[i].x , dronePoints[i].y), 6,( CV_RGB(255-242/magnitude,255-1/magnitude, 255-2/magnitude)), -1);
					// cyan - closer to black - optical flow
					circle(canvas, Point(rotatedX, rotatedY), 5, (  CV_RGB(455-1/magnitude,255- 212/magnitude, 255-220/magnitude)), -1); //roz - kokkino homography
					
					//flow vectors
					line(canvas, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(dronePointsPrev[i].x, 
					dronePointsPrev[i].y)+0.1*vectorLength*Point(dronePoints[i].x-dronePointsPrev[i].x, dronePoints[i].y-dronePointsPrev[i].y),
					CV_RGB(0, 255, 55) , 1, 8,0); 
					line(canvas, Point(dronePointsPrev[i].x, dronePointsPrev[i].y), Point(dronePointsPrev[i].x, dronePointsPrev[i].y)
					+0.1*vectorLength*Point(rotatedX-dronePointsPrev[i].x, rotatedY-dronePointsPrev[i].y),
					CV_RGB(55, 1, 1) , 1, 8,0); 
				}
				
				if(plotDebugTracking){
					circle(canvas, Point(dronePointsPrev[i].x , dronePointsPrev[i].y), 3,  CV_RGB(220/2, 220/2, 220/2), -1);	
				}			
			}
		}
		


		


		//FUSION PTZ - HOMOGRAPHY
		if(PTZ_HOMOGRAPHY_FUSION && 1==1){

			//FIND FUSED POINTS - vector<Point2f> dronePointsFUSION; dronePointsFINAL_MAG -  - ptzBasedFlowCorners_MAG - ptzBasedFlowCorners
			vector<Point2f> dronePointsFUSION;
			vector<float> dronePointsFUSION_MAG;

			//https://stackoverflow.com/questions/9825959/find-closest-neighbors-opencv
			
			//vector<Point2f> pointsForSearch; //Insert all 2D points to this vector
			//vector<float> pointsForSearch_MAG;

			for(size_t i=0; i < dronePointsFINAL.size(); i++) 
			{							
				for(size_t j=0; j < ptzBasedFlowCorners.size(); j++) {
					float X_diff = dronePointsFINAL[i].x - ptzBasedFlowCorners[j].x;
					float Y_diff = dronePointsFINAL[i].y - ptzBasedFlowCorners[j].y;
					float dist = sqrt( pow(X_diff,2) + pow(Y_diff,2) );
					if(dist < 0.35){ //if found a single point in other group that is close, add and break
						dronePointsFUSION.push_back(dronePointsFINAL[i]);
						dronePointsFUSION_MAG.push_back(dronePointsFINAL_MAG[i]);
						break;
					}
				}
			}
			
			//IF EMPTY, trust HOMOG points
			if( ptzBasedFlowCorners.size() < 5 ){
				for(size_t i=0; i < dronePointsFINAL.size(); i++) 
				{
					dronePointsFUSION.push_back(dronePointsFINAL[i]);
					dronePointsFUSION_MAG.push_back(dronePointsFINAL_MAG[i]);
				}
			}

			for(size_t i=0; i < ptzBasedFlowCorners.size(); i++) 
			{							
				for(size_t j=0; j < dronePointsFINAL.size(); j++) {
					float X_diff = dronePointsFINAL[j].x - ptzBasedFlowCorners[i].x;
					float Y_diff = dronePointsFINAL[j].y - ptzBasedFlowCorners[i].y;
					float dist = sqrt( pow(X_diff,2) + pow(Y_diff,2) );
					if(dist < 0.6){ //if found a single point in other group that is close, add and break
						//dronePointsFUSION.push_back(ptzBasedFlowCorners[i]);
						//dronePointsFUSION_MAG.push_back(ptzBasedFlowCorners_MAG[i]);
						break;
					}
				}
			}	

			if(1==0){
				dronePointsFUSION.clear();
				dronePointsFUSION_MAG.clear();
				for(size_t i=0; i < dronePointsFINAL.size(); i++) 
				{							
					dronePointsFUSION.push_back(dronePointsFINAL[i]);
					dronePointsFUSION_MAG.push_back(dronePointsFINAL_MAG[i]);
				}
				for(size_t i = 0; i < ptzBasedFlowCorners.size(); i++) {				
					dronePointsFUSION.push_back(ptzBasedFlowCorners[i]);
					dronePointsFUSION_MAG.push_back(ptzBasedFlowCorners_MAG[i]);
				}
			}
		
			//dronePointsFINAL.push_back(Point(dronePoints[i].x , dronePoints[i].y));
			//for(size_t i=0; i < dronePointsFINAL.size(); i++) {
			//	circle(plotPoints, Point(dronePointsFINAL[i].x,dronePointsFINAL[i].y), 40 * scaleFrame * (1/0.85) * dotPlotScaler, 
			//	(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/dronePointsFINAL_MAG[i]), -1);
			//}

			//DEBUG INDIVIDUAL METHODS REJECTED BY FUSION
			if(plotDebug && 1==0){
				for(size_t i=0; i < dronePointsFINAL.size(); i++) {
					circle(canvas, Point(dronePointsFINAL[i].x , dronePointsFINAL[i].y), 6, CV_RGB(255, 0, 0), -1); //RED
				}
				for(size_t i=0; i < ptzBasedFlowCorners.size(); i++) {
					circle(canvas, Point(ptzBasedFlowCorners[i].x , ptzBasedFlowCorners[i].y), 5, CV_RGB(0, 255, 0), -1); //GREEN
				}
			}

			for(size_t i=0; i < dronePointsFUSION.size(); i++) {
				circle(plotPoints, Point(dronePointsFUSION[i].x,dronePointsFUSION[i].y), 
				40 * scaleFrame * (1/0.85) * dotPlotScaler, 
				( CV_RGB(255-242/dronePointsFUSION_MAG[i],255- 1/dronePointsFUSION_MAG[i], 255-1/dronePointsFUSION_MAG[i])), -1);
				//40 * 1, 
				//CV_RGB(255, 255, 255), -1);

				if(plotDebugTrackingFUSION && plotDebug && 1==1){
					circle(canvas, Point(dronePointsFUSION[i].x , dronePointsFUSION[i].y), 7,(CV_RGB((255-242)/dronePointsFUSION_MAG[i], 255/dronePointsFUSION_MAG[i], 254/dronePointsFUSION_MAG[i])), -1);
					// cyan - closer to black - optical flow
					//circle(canvas, Point(rotatedX, rotatedY), 5, (CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/dronePointsFUSION_MAG[i]), -1); 
					 
					
				}
			}	
					
		}

		//TEST-DISABLE
		//rectangle(canvas, bbox2.tl(), bbox2.br(), CV_RGB(255, 1, 1), 2, 8, 0);

		if(plotDebug){
				imshow("FUSION RESULT", canvas);
		}
		//for(size_t i=0; i < dronePointsFINAL.size(); i++) {
		//	circle(plotPoints, Point(dronePointsFINAL[i].x,dronePointsFINAL[i].y), 40 , 
		//	(CV_RGB(255, 255, 255) ), -1);
		//}




		bool threeFrameConfidence = false;

		//FIND BOUNDING BOX AROUND INSIDE KALMAN POINTS
		if(PTZ_HOMOGRAPHY_FUSION && plotDebugTrackingFUSION){
			plotDebug = true;
		}
		Rect trackFrameEstimateKALMAN = getTrackWindowAroundPoints(1, plotPoints, canvas, plotDebug, maxContours);//no scaling			
		rectangle(plotPoints, trackFrameEstimateKALMAN.tl() * (1/1), trackFrameEstimateKALMAN.br() * (1/1), CV_RGB(255, 0, 255), 2, 8, 0);
		if(plotDebugTracking){
			rectangle(canvas, trackFrameEstimateKALMAN.tl() * (1/1), trackFrameEstimateKALMAN.br() * (1/1), CV_RGB(255, 0, 255), 2, 8, 0);				
		
			namedWindow("Points", WINDOW_NORMAL);
			cv::resizeWindow("Points", 640,480);
			imshow("Points", plotPoints);
		}

		if(PTZ_HOMOGRAPHY_FUSION && plotDebugTrackingFUSION){
			rectangle(canvas, trackFrameEstimateKALMAN.tl() * (1/1), trackFrameEstimateKALMAN.br() * (1/1), CV_RGB(255, 0, 255), 2, 8, 0);
			rectangle(canvas, bbox2.tl(), bbox2.br(), CV_RGB(255, 1, 1), 2, 8, 0);	
			imshow("FUSION RESULT plotPoints", plotPoints);// imshow("FUSION RESULT", canvas);
		}









		//FIND ROTATED bounding box from chosen resolution
		int margin = 25;
		//https://docs.opencv.org/3.4/df/dee/samples_2cpp_2minarea_8cpp-example.html#a13
		//vector<Point2f> dronePointsINSIDEbbox;
		vector<Point2f> dronePointsINSIDEbboxPREV;	//save previous points found inside new MOSSE box	
		for(size_t i = 0; i < dronePointsINSIDEbbox.size(); i++) 
		{
			if(dronePointsINSIDEbbox[i].x > bbox2.x - margin && dronePointsINSIDEbbox[i].x < bbox2.x +  bbox2.width + margin && dronePointsINSIDEbbox[i].y > bbox2.y - margin && dronePointsINSIDEbbox[i].y < bbox2.y + bbox2.height + margin 
				|| (dronePointsINSIDEbbox[i].x > trackFrameEstimateKALMAN.x - margin && dronePointsINSIDEbbox[i].x < trackFrameEstimateKALMAN.x +  trackFrameEstimateKALMAN.width + margin 
					&& dronePointsINSIDEbbox[i].y > trackFrameEstimateKALMAN.y - margin && dronePointsINSIDEbbox[i].y < trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height + margin)
			){
				dronePointsINSIDEbboxPREV.push_back(dronePointsINSIDEbbox[i]);
			}
		}
		dronePointsINSIDEbbox.clear(); //empty points vector

		//ADD previous
		for(size_t i=0; i < dronePointsINSIDEbboxPREV.size(); i++) 
		{			
			dronePointsINSIDEbbox.push_back(dronePointsINSIDEbboxPREV[i]);			
		}

		//ADD new
		for(size_t i=0; i < dronePointsFINAL.size(); i++) 
		{
			if(dronePointsFINAL[i].x > bbox2.x - margin && dronePointsFINAL[i].x < bbox2.x +  bbox2.width + margin && dronePointsFINAL[i].y > bbox2.y - margin && dronePointsFINAL[i].y < bbox2.y + bbox2.height + margin 
				//|| ( dronePointsFINAL[i].x > trackFrameEstimateKALMAN.x - margin && dronePointsFINAL[i].x < trackFrameEstimateKALMAN.x +  trackFrameEstimateKALMAN.width + margin 
					//&& dronePointsFINAL[i].y > trackFrameEstimateKALMAN.y - margin && dronePointsFINAL[i].y < trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height + margin  )
			){
				dronePointsINSIDEbbox.push_back(dronePointsFINAL[i]);
			}
		}
		
		Point2f vtx[4];
		if(dronePointsINSIDEbbox.size() > 2){
			RotatedRect box = minAreaRect(dronePointsINSIDEbbox);
			box.points(vtx);
			for( i = 0; i < 4; i++ ){
		    		line(canvas, vtx[i], vtx[(i+1)%4], Scalar(0, 255, 0), 4, LINE_AA);
			}
		}

		//OLD METHOD without looking history of points
		if(1==0){
			int margin = 40;
			for(size_t i=0; i < dronePointsFINAL.size(); i++) 
			{
				if(dronePointsFINAL[i].x > bbox2.x - margin && dronePointsFINAL[i].x < bbox2.x +  bbox2.width + margin && dronePointsFINAL[i].y > bbox2.y - margin && dronePointsFINAL[i].y < bbox2.y + bbox2.height + margin ){
					dronePointsINSIDEbbox.push_back(dronePointsFINAL[i]);
				}
			}		
			Point2f vtx[4];
			if(dronePointsINSIDEbbox.size() > 2){
				RotatedRect box = minAreaRect(dronePointsINSIDEbbox);
				box.points(vtx);
				for( i = 0; i < 4; i++ ){
			    		line(canvas, vtx[i], vtx[(i+1)%4], Scalar(0, 255, 0), 4, LINE_AA);
				}
			}		
		}//END 1==0

		//FIND ROTATED bounding box from FULL resolution (resolve problem inside current found window)









		//If drone points < 25 && window same place last frames, init mosse
		if(estimatedWindows.size() > 2){
			//bool threeFrameConfidence = false;
			Point center1(trackFrameEstimateKALMAN.x+trackFrameEstimateKALMAN.width, trackFrameEstimateKALMAN.y+trackFrameEstimateKALMAN.height);
			Point center2(bbox2.x+bbox2.width, bbox2.y+bbox2.height);
			Point center3(estimatedWindows[estimatedWindows.size()-1].x + estimatedWindows[estimatedWindows.size()-1].width, 
			estimatedWindows[estimatedWindows.size()-1].y+estimatedWindows[estimatedWindows.size()-1].height);
			Point center4(estimatedWindows[estimatedWindows.size()-2].x + estimatedWindows[estimatedWindows.size()-2].width, 
			estimatedWindows[estimatedWindows.size()-2].y+estimatedWindows[estimatedWindows.size()-2].height);
			Point center5(estimatedWindows[estimatedWindows.size()-3].x + estimatedWindows[estimatedWindows.size()-3].width, 
			estimatedWindows[estimatedWindows.size()-3].y+estimatedWindows[estimatedWindows.size()-3].height);
			float distCenters = sqrt( pow((center1.x-center3.x),2) + pow((center1.y-center3.y),2) );
			float distCenters2 = sqrt( pow((center4.x-center3.x),2) + pow((center4.y-center3.y),2) );
			float distCenters12 = sqrt( pow((center4.x-center1.x),2) + pow((center4.y-center1.y),2) );
			float distCenters15 = sqrt( pow((center5.x-center1.x),2) + pow((center5.y-center1.y),2) );
			float distMOSSE  = sqrt( pow((center1.x-center2.x),2) + pow((center1.y-center2.y),2) );
			float distMOSSE1 = sqrt( pow((center3.x-center2.x),2) + pow((center3.y-center2.y),2) );
			float distMOSSE2 = sqrt( pow((center4.x-center2.x),2) + pow((center4.y-center2.y),2) );
			float distMOSSE3 = sqrt( pow((center5.x-center2.x),2) + pow((center5.y-center2.y),2) );

			//no confidense, discard
			bool repeatConfidense = true;
			if( distCenters > minDisttoCurrentMOSSE*2 && distCenters12 > minDisttoCurrentMOSSE*2 && distCenters15 > minDisttoCurrentMOSSE*2){
				repeatConfidense = false;
			}

			Point2f windowMOSSE(trackFrameEstimateKALMAN.x, trackFrameEstimateKALMAN.y);
			Point2f sizeMOSSE(trackFrameEstimateKALMAN.width, trackFrameEstimateKALMAN.height);
			bool newWindowBigger = false;
			if(distCenters < minDistInitMOSSE && distCenters2 < minDistInitMOSSE && distCenters12 < minDistInitMOSSE
				&& estimatedWindows[estimatedWindows.size()-1].x != -100 && estimatedWindows[estimatedWindows.size()-2].x != -100 && estimatedWindows[estimatedWindows.size()-3].x != -100
			){
				threeFrameConfidence = true;


				//NEW1
				Homog_enter_and_found = 2; Homog_enter_and_found2++;

				//cout << "3 frame confidense ..." << endl << endl;

				//MOSSE SIZING
				vector<float> Xis;
				Xis.push_back(trackFrameEstimateKALMAN.x);
				Xis.push_back(estimatedWindows[estimatedWindows.size()-1].x);
				Xis.push_back(estimatedWindows[estimatedWindows.size()-2].x);
				vector<float> Yis;
				Yis.push_back(trackFrameEstimateKALMAN.y);
				Yis.push_back(estimatedWindows[estimatedWindows.size()-1].y);
				Yis.push_back(estimatedWindows[estimatedWindows.size()-2].y);
				
				float upperLeftX = *std::min_element(Xis.begin(), Xis.end());
				float upperLeftY = *std::min_element(Yis.begin(), Yis.end());
				windowMOSSE.x = upperLeftX;
				windowMOSSE.y = upperLeftY;

				vector<float> MXis;
				MXis.push_back(trackFrameEstimateKALMAN.x+ trackFrameEstimateKALMAN.width);
				MXis.push_back(estimatedWindows[estimatedWindows.size()-1].x + estimatedWindows[estimatedWindows.size()-1].width);
				MXis.push_back(estimatedWindows[estimatedWindows.size()-2].x+ estimatedWindows[estimatedWindows.size()-2].width);
				vector<float> MYis;
				MYis.push_back(trackFrameEstimateKALMAN.y+ trackFrameEstimateKALMAN.height);
				MYis.push_back(estimatedWindows[estimatedWindows.size()-1].y+ estimatedWindows[estimatedWindows.size()-1].height);
				MYis.push_back(estimatedWindows[estimatedWindows.size()-2].y+ estimatedWindows[estimatedWindows.size()-2].height);
		
				float lowerRightX = *std::min_element(MXis.begin(), MXis.end());
				float lowerRightY = *std::min_element(MYis.begin(), MYis.end());
				sizeMOSSE.x = lowerRightX - upperLeftX;
				sizeMOSSE.y = lowerRightY - upperLeftY;

				//IF new confidence 3 window bigger in size, resize MOSSE
				if( sizeMOSSE.x > bbox2.width && sizeMOSSE.y > bbox2.height  ){
					newWindowBigger = true;
				}
			}
			
			//If confidence (dots found on target - rectangle area) lower than last frame && distance to last MOSSE window above threshold, do not update MOSSE.
			//Make bounding box more expanse based on last 3 frames.

			//if(!okB && dronePoints.size() < 25 && distCenters < 200 && center.x > 0 && center.y > 0 && center.x < cur.cols && center.y < cur.rows){
			if(repeatConfidense && dronePoints.size() < 25 && (center1.x > 0 && center1.y > 0 && center1.x < cur.cols && center1.y < cur.rows) 
				//&& (trackFrameEstimateKALMAN.x > 0 && trackFrameEstimateKALMAN.y > 0 && trackFrameEstimateKALMAN.x+trackFrameEstimateKALMAN.width < cur.cols 
				//&& trackFrameEstimateKALMAN.y+trackFrameEstimateKALMAN.height < cur.rows) 
				&& ( (!okB || threeFrameConfidence) && distMOSSE > minDisttoCurrentMOSSE && distCenters < minDisttoPrevWindow)
				|| (center2.x < 0 || center2.y < 0 || center2.x > cur.cols || center2.y > cur.rows)// || threeFrameConfidence
				//|| newWindowBigger
				//|| (distMOSSE > minDisttoCurrentMOSSE*3 && distMOSSE1 > minDisttoCurrentMOSSE*3 && distMOSSE2 > minDisttoCurrentMOSSE*3)
				|| (newWindowBigger && threeFrameConfidence)
				|| (distMOSSE > 460 && threeFrameConfidence)
			){
				//cout << "Enabling and recentering MOSSE at frame: " << frameID << endl << endl;
				tracker = trackerUsed::create();
				bbox2.x = windowMOSSE.x;
				bbox2.y = windowMOSSE.y;
				bbox2.width = sizeMOSSE.x * 1.0;
				bbox2.height = sizeMOSSE.y * 1.0;

				//id no measurement, use previous position
				if(bbox2.x == -100 && bboxes.size() > 3){
					//bbox2.x = canvas.cols - bbox2.width/2;
					//bbox2.y = canvas.rows - bbox2.height/2;
					//bbox2.width = sizeMOSSE.x * 1.0;
					//bbox2.height = sizeMOSSE.y * 1.0;
					//bbox2.x = canvas.cols/2 - bboxes[bboxes.size()-1].width/2;//bboxes[bboxes.size()-2].x;
					//bbox2.y = canvas.rows/2 - bboxes[bboxes.size()-1].height/2;
					bbox2.x = bboxes[bboxes.size()-1].x;//bboxes[bboxes.size()-2].x;
					bbox2.y = bboxes[bboxes.size()-1].y;
					bbox2.width = bboxes[bboxes.size()-1].width;
					bbox2.height =bboxes[bboxes.size()-1].height;

					 
				}

//NEW1
					Mosse_enter_and_found = 2; Mosse_enter_and_found2++;

				tracker->init(prev, bbox2);
				//Rect bboxTMP;
				okB = tracker->update(cur, bbox2);
			}else{
				//Rect bboxTMP;
				okB = tracker->update(cur, bbox2);

				//NEW1
				Mosse_enter_and_found = 1; Mosse_enter_and_found1++;			
			}
			if(plotDebugTracking){
				rectangle(canvas, bbox2.tl(), bbox2.br(), CV_RGB(255, 1, 1), 2, 8, 0);
			}
		}
		estimatedWindows.push_back(trackFrameEstimateKALMAN);		

		//SHOW HOMOGRAPHY
		if(plotHomography){
			imshow("Homography result", canvasF);
		}

		//if(frameID==329){
		//	cout << bbox2 << endl;
		//}

		//PTZ - ROS - LOGS
		//ROS
		//SSTR(int(counter+start_frame))
		//int frameID = frameID;
	//	std_msgs::Float64MultiArray array;
		//Clear array
	//	array.data.clear();
	//	array.data.push_back(frameID); //insert frame id	

		std_msgs::UInt32 array;//std_msgs::UInt32MultiArray array;
		array.data = frameID;
	
		pub.publish(array);
		//Let the world know
		//ros::spin();
		ros::spinOnce();
		//ROS_INFO("I published something!");		
		//END ROS



	
		//DATA CLEAN UP
		//If no confidense, discard // trackFrameEstimateKALMAN = estimation, bbox2 = MOSSE
		Rect trackFrameEstimateKALMAN_Confidence = trackFrameEstimateKALMAN;
		Rect trackFrameEstimateMOSSE_Confidence = bbox2;
		if(estimatedWindows.size() > 2){
				Point center0(estimatedWindows[estimatedWindows.size()-1].x + estimatedWindows[estimatedWindows.size()-1].width, 
								estimatedWindows[estimatedWindows.size()-1].y+estimatedWindows[estimatedWindows.size()-1].height);
				Point center1(estimatedWindows[estimatedWindows.size()-2].x + estimatedWindows[estimatedWindows.size()-2].width, 
								estimatedWindows[estimatedWindows.size()-2].y+estimatedWindows[estimatedWindows.size()-2].height);
				Point center2(estimatedWindows[estimatedWindows.size()-3].x + estimatedWindows[estimatedWindows.size()-3].width, 
								estimatedWindows[estimatedWindows.size()-3].y+estimatedWindows[estimatedWindows.size()-3].height);
				Point center3(estimatedWindows[estimatedWindows.size()-4].x + estimatedWindows[estimatedWindows.size()-4].width, 
								estimatedWindows[estimatedWindows.size()-4].y+estimatedWindows[estimatedWindows.size()-4].height);	
				float distCenters01 = sqrt( pow((center1.x-center0.x),2) + pow((center1.y-center0.y),2) );
				float distCenters02 = sqrt( pow((center2.x-center0.x),2) + pow((center2.y-center0.y),2) );
				float distCenters03 = sqrt( pow((center3.x-center0.x),2) + pow((center3.y-center0.y),2) );	
				if( distCenters01 > minDisttoCurrentMOSSE*2 && distCenters02 > minDisttoCurrentMOSSE*2 && distCenters03 > minDisttoCurrentMOSSE*2){
					trackFrameEstimateKALMAN_Confidence.x = -100;
					trackFrameEstimateKALMAN_Confidence.y = -100;
					trackFrameEstimateKALMAN_Confidence.width = 10;
					trackFrameEstimateKALMAN_Confidence.height = 10;
				}		
				
				//CLEAN UP MOSSE, case where MOSSE not re-initialized and stuck not tracking, use the cleaned up for Kalman measurement
				Point centerM(bbox2.x+bbox2.width, bbox2.y+bbox2.height);
				float distMOSSECenters01 = sqrt( pow((center0.x-centerM.x),2) + pow((center0.y-centerM.y),2) );
				float distMOSSECenters02 = sqrt( pow((center1.x-centerM.x),2) + pow((center1.y-centerM.y),2) );
				float distMOSSECenters03 = sqrt( pow((center2.x-centerM.x),2) + pow((center2.y-centerM.y),2) );	
				if( distMOSSECenters01 > minDisttoCurrentMOSSE*2 && distMOSSECenters02 > minDisttoCurrentMOSSE*2 && distMOSSECenters03 > minDisttoCurrentMOSSE*2){
					trackFrameEstimateMOSSE_Confidence.x = -100;
					trackFrameEstimateMOSSE_Confidence.y = -100;
					trackFrameEstimateMOSSE_Confidence.width = 10;
					trackFrameEstimateMOSSE_Confidence.height = 10;
				}	
		}



		//bboxes - check if consient, only then do PID tracking
		Rect bbox2DIFF;
		bbox2DIFF.x = bbox2.x;
		bbox2DIFF.y = bbox2.y;
		bbox2DIFF.width = bbox2.width;
		bbox2DIFF.height = bbox2.height;
		bboxes.push_back(bbox2);
		if(1==1 && bboxes.size() > 3){
			//if consistent, move, otherwise stop PID tracker
			Point center01(bboxes[bboxes.size()-1].x + bboxes[bboxes.size()-1].width,bboxes[bboxes.size()-1].y+bboxes[bboxes.size()-1].height);
			Point center02(bboxes[bboxes.size()-2].x + bboxes[bboxes.size()-2].width,bboxes[bboxes.size()-2].y+bboxes[bboxes.size()-2].height);
			Point center03(bboxes[bboxes.size()-3].x + bboxes[bboxes.size()-3].width,bboxes[bboxes.size()-3].y+bboxes[bboxes.size()-3].height);
			Point center04(bboxes[bboxes.size()-4].x + bboxes[bboxes.size()-4].width,bboxes[bboxes.size()-4].y+bboxes[bboxes.size()-4].height);
			float distMOSSECenters001 = sqrt( pow((center02.x-center01.x),2) + pow((center02.y-center01.y),2) );
			float distMOSSECenters002 = sqrt( pow((center03.x-center01.x),2) + pow((center03.y-center01.y),2) );
			float distMOSSECenters003 = sqrt( pow((center04.x-center01.x),2) + pow((center04.y-center01.y),2) );	
			if( distMOSSECenters001 > minDisttoCurrentMOSSE*2 && distMOSSECenters002 > minDisttoCurrentMOSSE*2 && distMOSSECenters003 > minDisttoCurrentMOSSE*2){
				bbox2DIFF.x = canvas.cols/2 - bboxes[bboxes.size()-1].width/2;//bboxes[bboxes.size()-2].x;
				bbox2DIFF.y = canvas.rows/2 - bboxes[bboxes.size()-1].height/2;
				bbox2DIFF.width = bboxes[bboxes.size()-1].width;
				bbox2DIFF.height =bboxes[bboxes.size()-1].height;

				bbox2.x = canvas.cols/2 - bboxes[bboxes.size()-1].width/2;//bboxes[bboxes.size()-2].x;
				bbox2.y = canvas.rows/2 - bboxes[bboxes.size()-1].height/2;
				bbox2.width = bboxes[bboxes.size()-1].width;
				bbox2.height =bboxes[bboxes.size()-1].height;

				tracker->init(prev, bbox2);
				okB = tracker->update(cur, bbox2);
			}	
		}





////////////////////////////// KALMAN FILTER ////////////////////////////////////
		//https://github.com/Myzhar/simple-opencv-kalman-tracker/blob/master/source/opencv-kalman.cpp
		// >>>>> Main loop
			
		Rect trackFrameEstimate(-100, -100, 10, 10);
		trackFrameEstimate.x = trackFrameEstimateKALMAN_Confidence.x;
		trackFrameEstimate.y = trackFrameEstimateKALMAN_Confidence.y;
		trackFrameEstimate.width = trackFrameEstimateKALMAN_Confidence.width;
		trackFrameEstimate.height = trackFrameEstimateKALMAN_Confidence.height;

		if(trackFrameEstimateKALMAN.x == -100 && bboxes.size() > 3){
			//trackFrameEstimate.x = trackFrameEstimateMOSSE_Confidence.x;
			//trackFrameEstimate.y = trackFrameEstimateMOSSE_Confidence.y;
			//trackFrameEstimate.width = trackFrameEstimateMOSSE_Confidence.width;
			//trackFrameEstimate.height = trackFrameEstimateMOSSE_Confidence.height;
			trackFrameEstimate.x =  bboxes[bboxes.size()-1].x;
			trackFrameEstimate.y = bboxes[bboxes.size()-1].y;
			trackFrameEstimate.width = bboxes[bboxes.size()-1].width;
			trackFrameEstimate.height = bboxes[bboxes.size()-1].height;
		}

		//trackFrameEstimate.x = trackFrameEstimateKALMAN_Confidence.x + trackFrameEstimateMOSSE_Confidence.x/2;
		//trackFrameEstimate.y = trackFrameEstimateKALMAN_Confidence.y + trackFrameEstimateMOSSE_Confidence.y/2;
		//trackFrameEstimate.width = trackFrameEstimateKALMAN_Confidence.width + trackFrameEstimateMOSSE_Confidence.width/2;
		//trackFrameEstimate.height = trackFrameEstimateKALMAN_Confidence.height + trackFrameEstimateMOSSE_Confidence.height/2;

		trackFrameEstimateCenters.push_back(Point(trackFrameEstimate.x,trackFrameEstimate.y));

		if(trackFrameEstimateCenters.size() > 1)
		{
			double precTick = ticks;
			ticks = (double) cv::getTickCount();

			double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
			
			if (found)
			{		    

			    kf.transitionMatrix.at<float>(2)  = dT;
			    kf.transitionMatrix.at<float>(4)  = (1/2)*dT*dT;
    			    kf.transitionMatrix.at<float>(11) = dT;
			    kf.transitionMatrix.at<float>(13) = (1/2)*dT*dT;				
			    kf.transitionMatrix.at<float>(20) = dT;
			    kf.transitionMatrix.at<float>(29) = dT;
			  
			    state = kf.predict();
	
 			    predRect.width = state.at<float>(6);
			    predRect.height = state.at<float>(7);
			    predRect.x = state.at<float>(0) - predRect.width / 2;
			    predRect.y = state.at<float>(1) - predRect.height / 2;

			    predRects.push_back(predRect);
		
			    center.x = state.at<float>(0);
			    center.y = state.at<float>(1);
				
			    //cv::circle(plotFrame, center, 2, CV_RGB(255,0,0), -1);	
			    //cv::rectangle(plotFrame, predRect, CV_RGB(255,0,0), 2);
			    
			    Point currentCenter =  Point(trackFrameEstimate.x + trackFrameEstimate.width/2,trackFrameEstimate.y + trackFrameEstimate.height/2);
			    Point estimateToCurrent = center - currentCenter;
			    //cv::line(plotFrame, currentCenter, currentCenter + estimateToCurrent * 4, CV_RGB(212, 112, 0));
 			    //cv::circle(plotFrame, currentCenter + estimateToCurrent * 4, 5, CV_RGB(255,0,0), -1);
			
			    //PLOT PREDICTION WINDOW
			    //cv::Rect predTrackRect;
			    //predTrackRect.width = trackFrameEstimate.width;
			    //predTrackRect.height = trackFrameEstimate.height;
			    //predTrackRect.x = currentCenter.x-estimateToCurrent.x - trackFrameEstimate.width/2;
			    //predTrackRect.y = currentCenter.y-estimateToCurrent.y - trackFrameEstimate.height/2; 			   
			}
			
			if (	((trackFrameEstimate.x + trackFrameEstimate.width/2 < 0 || trackFrameEstimate.y + trackFrameEstimate.height/2 < 0 
				|| trackFrameEstimate.x + trackFrameEstimate.width/2 > canvas.cols || trackFrameEstimate.y + trackFrameEstimate.height/2 > canvas.rows))				
			)				
			{
			    notFoundCount++;
	
			    if( notFoundCount >= 15 )
			    {
				found = false;
			    }
			    /*else
				kf.statePost = state;*/
			}
			else
			{
			    notFoundCount = 0;
			
				//if measurement is not much distance from previous, update, otherwise use previous
				int previousID = trackFrameEstimateCenters.size()-2;
				float distancePrev = sqrt(pow(trackFrameEstimate.x - trackFrameEstimateCenters[previousID].x, 2) + pow(trackFrameEstimate.y - trackFrameEstimateCenters[previousID].y, 2));
				
				//ACCELERATION
				float speedX = -0.005	*	(trackFrameEstimate.x - trackFrameEstimateCenters[previousID].x) / dT; 			
				float speedY = -0.005	*	(trackFrameEstimate.y - trackFrameEstimateCenters[previousID].y) / dT;	

			 	// Measure Matrix H - MP x DP ACCEL
			  	// [ 1 0 0 0 0 0 0 0]	//measurement of x position
				// [ 0 1 0 0 0 0 0 0]	//measurement of y position
				// [ 0 0 1 0 0 0 0 0]	//measurement of velocity x
				// [ 0 0 0 1 0 0 0 0] 	//measurement of velocity y
				// [ 0 0 0 0 0 0 1 0]	//measurements of width, height
				// [ 0 0 0 0 0 0 0 1]

				//cv::Mat meas(measSize, 1, type);    // [z_x, z_y, z_vx, z_vy, z_w,  z_h]

				if(distancePrev < 1250){//if(1 ==1 || distancePrev < 390){ //320
				    meas.at<float>(0) = trackFrameEstimate.x + trackFrameEstimate.width / 2;
				    meas.at<float>(1) = trackFrameEstimate.y + trackFrameEstimate.height / 2;					
					
				    meas.at<float>(2) = (float)speedX;
				    meas.at<float>(3) = (float)speedY;

				    meas.at<float>(4) = (float)trackFrameEstimate.width;
				    meas.at<float>(5) = (float)trackFrameEstimate.height;
					
				}else{
				    meas.at<float>(0) = state.at<float>(0);//   (trackFrameEstimateCenters[previousID].x + state.at<float>(0) )/2;
				    meas.at<float>(1) = state.at<float>(1);//  (trackFrameEstimateCenters[previousID].y + state.at<float>(1) )/2;
 				    meas.at<float>(2) = (float)speedX;
				    meas.at<float>(3) = (float)speedY;
				    meas.at<float>(4) = (float)trackFrameEstimate.width;
				    meas.at<float>(5) = (float)trackFrameEstimate.height;
				}

			    if (!found) // First detection!
			    {
				kf.errorCovPre.at<float>(0) = 1; // px
				kf.errorCovPre.at<float>(9) = 1; // px
				kf.errorCovPre.at<float>(18) = 1;
				kf.errorCovPre.at<float>(27) = 1;
				kf.errorCovPre.at<float>(36) = 1; // px
				kf.errorCovPre.at<float>(45) = 1; // px
				kf.errorCovPre.at<float>(54) = 1; // px
				kf.errorCovPre.at<float>(63) = 1; // px

				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);
				state.at<float>(2) = meas.at<float>(2);
				state.at<float>(3) = meas.at<float>(3);
				state.at<float>(4) = 0;
				state.at<float>(5) = 0;
				state.at<float>(6) = meas.at<float>(4)/2;
				state.at<float>(7) = meas.at<float>(5)/2;
				// <<<< Initialization

				kf.statePost = state;				
				found = true;
			    }
			    else{
				kf.correct(meas); // Kalman Correction
			    }	
			}
			// <<<<< Kalman Update
		}

		if(predRects.size() > 0 && plotDebug){
			rectangle(canvas, predRects[predRects.size()-1].tl(), predRects[predRects.size()-1].br(), CV_RGB(11, 11, 11), 2, 1, 0);
		}
		////////////////////////////// END KALMAN FILTER ////////////////////////////////









		

		//// PTZ CONTROLS
		//ZOOM - PAN TEST SIN !!!!
		if(1==0 &&  !enablePIDracking && usePTZ){	
					cout << "frameCounter="  << frameID  << endl;
					//Pan is 0 at home. Right is positive, max 2448. Left ranges from full
					//left 63088 to 65535 before home.
					//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully
					//depressed at 65104 to 65535 before home.  Zoom is 0 to 16384					
					if(frameID < 50){//if(frameCounter < 1100){
						//go home posiion				
						std::string homePTZ = "81010604FF";
						udpSend(homePTZ, false, false);
						//manual focus				
						std::string focusPTZ = "8101043803FF";//"8101043802FF";//std::string focusPTZ = "8101043803FF";
						udpSend(focusPTZ, false, false);
						//stop motion
						string WW = "00";
						std::string panPTZ = "81010601"+WW+WW+"0303FF";
						//udpSend(panPTZ, false, false);
						//zoom out
						string PP = "2";//zoom speed 0 to 7
						std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
						udpSend(zoomPTZ, false, false);
					}else{
						//ZOOM IN	
						if(enableTestZoom){			
							//string PP = "1";//zoom speed 0 to 7
							int zoomSpeed = 1;
							if(currentZoom > 5800){//if(currentZoom > 4800){
								string PP = "0";//zoom speed 0 to 7
								std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out = 810104073pFF
								udpSend(zoomPTZ, false, false);
								zoomRate = -zoomSpeed * 120 / 7;
							}else if(currentZoom < 5){
								string PP = "0";
								std::string zoomPTZ = "810104072"+PP+"FF"; //zoom in  = 810104072pFF
								udpSend(zoomPTZ, false, false);
								zoomRate = zoomSpeed * 120 / 7;
							}
							//cout << "currentZoom=" << currentZoom << endl;							
						}

						int randomNum = rand() % 4;

						//		cout << "currentPan="  << currentPan  << endl;							
						//HANDLE PAN - TILT combination
						if(enableTestPanTilt){
							string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							string WW = "02";
							int panSpeed = 2;
							int tiltSpeed = 2;
							if(currentTilt > 180 && currentTilt < 50000){  //if(currentTilt > 250 && currentTilt < 50000){							
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //if tilt looks up, go down right
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
							else if(currentTilt < 1 || currentTilt > 65300){
								std::string panPTZ = "81010601"+VV+WW+"0101FF";
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;	
								udpSend(panPTZ, false, false);			
							}											
											
						}	
						cout << "currentTilt="  << currentTilt  << endl;
					}
		}/////END ZOOM - PAN TEST SIN !!!!


		//IMPLEMENT PTZ CONTROLS, 5678 = TCP port
		//if(toggleCommandrate == 0 && enablePIDracking && usePTZ){
		if(1==0 && enablePIDracking && usePTZ){

				toggleCommandrate = 3;

				//const char* homePTZ = "81010604FF";
				//std::string homePTZ = "81010604FF";
				//udpSend(homePTZ, false);
				//float PID_P = 0.75f;//0.75f;
				float ScreenCenterX = canvas.cols/2; 
				float ScreenCenterY = canvas.rows/2; 
				float PID_X_DIFF = (bbox2DIFF.x + (bbox2DIFF.width/2) - ScreenCenterX);
				float PID_Y_DIFF = (bbox2DIFF.y + (bbox2DIFF.height/2) - ScreenCenterY);
				//float PID_Control_Pan = PID_P * PID_X_DIFF;
				//float maxPanSpeed = 8;
				float proximityDivider = 22;
				//cout << "ok:" << ok << " okA:" << okA << " okB:" << okB << endl;

				bool box2Same = false;
				if(bbox2P.x == bbox2.x && bbox2P.y == bbox2.y){
					box2Same = true;
					bbox2P.x = bbox2.x;
					bbox2P.y = bbox2.y;
					bbox2P.width = bbox2.width;
					bbox2P.height = bbox2.height;
				}


				if(zoomingOut && currentZoom == 0){
					zoomingOut = false;
				}

				//if(box2Same || !okB || okB == 0 || abs(PID_X_DIFF) > ScreenCenterX * 0.85f || frameCounter < 110 || abs(PID_Y_DIFF) > ScreenCenterY * 0.85f ){
				//if(box2Same || !okB || okB == 0  || frameID < 50 ){//if(box2Same || !okB || okB == 0  || frameCounter < 50 ){		
				//if(box2Same || frameID < 50 || abs(PID_X_DIFF) > 100 || abs(PID_Y_DIFF) > 100 ){
				//if(box2Same || frameID < 50  || abs(PID_X_DIFF) > ScreenCenterX * 0.85f || abs(PID_Y_DIFF) > ScreenCenterY * 0.85f ||  
				//	(threeFrameConfidence && abs(trackFrameEstimateKALMAN.x - bbox2.x) > 250 &&  abs(trackFrameEstimateKALMAN.y - bbox2.y) > 250 ) 
				//	|| bbox2.width > canvas.cols/5
				//cout << "abs(PID_X_DIFF):" << abs(PID_X_DIFF) << ", " <<  abs(PID_Y_DIFF) << endl;
				bool tiltTest = (currentTilt > 1000 && currentTilt < 60000) || (currentTilt < 65255 && currentTilt >= 65104) ;
				if(	//abs(PID_X_DIFF) > ScreenCenterX * 0.65f && abs(PID_Y_DIFF) > ScreenCenterY * 0.65f 
 					frameID < 50 // || !okB || okB == 0   //
					//|| abs(PID_X_DIFF) > ScreenCenterX * 0.65f || abs(PID_Y_DIFF) > ScreenCenterY * 0.65f 
					//|| (threeFrameConfidence && abs(trackFrameEstimateKALMAN.x - bbox2.x) > 850 &&  abs(trackFrameEstimateKALMAN.y - bbox2.y) > 850 ) 
					//|| bbox2.width > canvas.cols/3 || bbox2.height > canvas.rows/3
					|| tiltTest
					//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully
					//depressed at 65104 to 65535 before home
					//|| box2Same
					|| dronePoints.size() > 160
					|| (zoomingOut && currentZoom > 0)
					//|| 
				){		

//					cout << "Reseting .........................................." << PID_X_DIFF << "," << PID_Y_DIFF << endl;		
					//zoom out
					if(frameID < 50 || dronePoints.size() > 160 || (zoomingOut && currentZoom > 0) ){

						if(!tiltTest && frameID > 50){
							string PP = "7";//zoom speed 0 to 7
							std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
							udpSend(zoomPTZ, false, false);
							//cout << "zooming out ... " << currentZoom << endl;
						}else{
							//go home posiion				
							std::string homePTZA = "81010604FF";
							udpSend(homePTZA, false, false);
							//reset mosse to center
							//bbox2.x = canvas.cols/2;//bboxes[bboxes.size()-2].x;
							//bbox2.y = canvas.rows/2;
							//bbox2.width =bbox2.width;
							//bbox2.height =bbox2.height;
							//tracker->init(prev, bbox2);
							//okB = tracker->update(cur, bbox2);

							
							if(frameID < 50){
								bbox2.x = -100;
								bbox2.y = -100;
								bbox2.width = 10;
								bbox2.height = 10;
								tracker->init(prev, bbox2);
								okB = tracker->update(cur, bbox2);							
							}
						}

						zoomingOut = true;
						
					}else{
					//STOP ZOOM					
						std::string zoomStopPTZ = "8101040700FF"; //stop zoom
						udpSend(zoomStopPTZ, false, false);
						zoomRate=0;
						cout << "zooming stopped at ... " << currentZoom << endl;


						

					}

					

					//cout << "RESETTING" << endl;
					//std::string getPanTiltPTZ = "81090612FF"; //read pan - tilt
					//udpSend(getPanTiltPTZ, true, false);

					///HANDLE PAN						
					//go home posiion				
					std::string homePTZ = "81010604FF";
					if(1==0 && frameID < 50 || tiltTest || dronePoints.size() > 160){
						//udpSend(homePTZ, false, false);


						//reset mosse to center
						bbox2.x = canvas.cols/2;//bboxes[bboxes.size()-2].x;
						bbox2.y = canvas.rows/2;
						bbox2.width =bbox2.width;
						bbox2.height =bbox2.height;
						tracker->init(prev, bbox2);
						okB = tracker->update(cur, bbox2);

					}
					//manual focus				
					std::string focusPTZ = "8101043803FF";//"8101043802FF";//std::string focusPTZ = "8101043803FF";
				//	udpSend(focusPTZ, false, false);
					//stop motion
					string WW = "00";
					std::string panPTZ = "81010601"+WW+WW+"0303FF";
					if(frameID >= 50 && !tiltTest){					
				//		udpSend(panPTZ, false, false);				
					}
					panRate = 0;
					tiltRate = 0;
					zoomRate = 0;									

				}else{
					//RESET TRACK WINDOWS CANDIDATES
					//bboxesESTIMATED_STATIC.clear();
					//bboxESTIMATE_STATIC_frames.clear();											

					//PAN MOTION ENABLE					

					//COMBINED PAN TILT
					//HANDLE PAN - TILT combination
					if(1==1){
							//string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							//string WW = "02";

							float tiltDiffF =PID_P * maxPanSpeed * (abs(PID_Y_DIFF) / ScreenCenterY);					
							int tiltDiff = (int)(tiltDiffF);
							string WW = to_string(tiltDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(tiltDiff < 10){
								//add pading
								WW = "0"+WW;
							}
							//string WW = "01";

							float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
							int panDiff = (int)(panDiffF);
							string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(panDiff < 10){
								//add pading
								VV = "0"+VV;
							}

							int panSpeed = panDiff;
							int tiltSpeed = tiltDiff;
							float diffThreshold = 1;
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - right
								std::string panPTZ = "81010601"+VV+WW+"0201FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - right
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - left
								std::string panPTZ = "81010601"+VV+WW+"0101FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - left
								std::string panPTZ = "81010601"+VV+WW+"0102FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}											
					}	
					//END HANDLE PAN - TILT combination

					//ZOOM MOTION ENABLE
					if(enablePIDZoom && 1==1){
						int zoomSpeed = 2; //3
						//cout << "currentZoom=" << currentZoom << " PID_X_DIFF=" << PID_X_DIFF << " PID_Y_DIFF=" << PID_Y_DIFF << endl;
						//if(abs(PID_X_DIFF) < ScreenCenterX/proximityDivider && abs(PID_Y_DIFF) < ScreenCenterY/proximityDivider){ //when centered, zoom in
						//if(abs(PID_X_DIFF) < 950 && abs(PID_Y_DIFF) < 950 && currentZoom < maxPTZ_Zoom){// && currentZoom < maxPTZ_Zoom){ //when centered, zoom in 1250 and 2400
						if(abs(PID_X_DIFF) < canvas.cols/4 && abs(PID_Y_DIFF) < canvas.rows/4 && currentZoom < maxPTZ_Zoom && bbox2.width < canvas.cols/4){
							//stop motion
							//string WW = "00";
							//std::string panPTZ = "81010601"+WW+WW+"0303FF";
							//udpSend(panPTZ, false, false);
							//ZOOM IN				
							string PP = "2";//zoom speed 0 to 7 //5
							std::string zoomPTZ = "810104072"+PP+"FF"; //zoom out, zoom in = 810104072pFF
							udpSend(zoomPTZ, false, false);

							zoomSpeed = 1;
							zoomRate = zoomSpeed * 120 / 7;
							//cout << "zooming in to ... " << currentZoom << endl;
						}else{					

							//if(abs(PID_X_DIFF) > 1050 || abs(PID_Y_DIFF) > 1050){
							if(abs(PID_X_DIFF) > canvas.rows/4 || abs(PID_Y_DIFF) > canvas.rows/4){
								string PP = "4";//zoom speed 0 to 7 //2
								std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
								//udpSend(zoomPTZ, false, false);

								zoomSpeed = 2;
								zoomRate = -zoomSpeed * 120 / 7;

								//cout << "zooming OUT to ... " << currentZoom << endl;
							}else{
								//STOP ZOOM					
								std::string zoomStopPTZ = "8101040700FF"; //stop zoom
								udpSend(zoomStopPTZ, false, false);
								zoomRate=0;
								//cout << "zooming 2 stopped at ... " << currentZoom << endl;
							}
						}						
					}				
	  			}			
		}
		//END IMPLEMENT PTZ CONTROLS, 5678 = TCP port	
		//// END PTZ CONTOLS


		toggleCommandrate--;


		//Cluster the points
		if(1==0){
			//https://stackoverflow.com/questions/33825249/opencv-euclidean-clustering-vs-findcontours
			if(dronePoints.size() > 0){
				// Define the radius tolerance
				int th_distance = 250; // radius tolerance

				// Apply partition 
				// All pixels within the radius tolerance distance will belong to the same class (same label)
				vector<int> labels;

				// With functor
				//int n_labels = partition(dronePoints, labels, EuclideanDistanceFunctor(th_distance));

				// With lambda function (require C++11)
				int th2 = th_distance * th_distance;
				int n_labels = partition(dronePoints, labels, [th2](const Point& lhs, const Point& rhs) {
					return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2;
				});

				// You can save all points in the same class in a vector (one for each class), just like findContours
				vector<vector<Point>> contours(n_labels);
				for (int i = 0; i < dronePoints.size(); ++i)
				{
					contours[labels[i]].push_back(dronePoints[i]);
				}

				// Draw results

				// Build a vector of random color, one for each class (label)
				vector<Vec3b> colors;
				for (int i = 0; i < n_labels; ++i)
				{
					colors.push_back(Vec3b(rand() & 255, rand() & 255, rand() & 255));
				}

				// Draw the labels
				//Mat3b lbl(canvas.rows, canvas.cols, Vec3b(0, 0, 0));
				for (int i = 0; i < dronePoints.size(); ++i)
				{
					//lbl(dronePoints[i]) = colors[labels[i]];
					circle(canvas, dronePoints[i], 15,  colors[labels[i]], -1); //roz - kokkino
				}
				//imshow("Labels", lbl);
			}
		}

		//CLUSTER WITH DBSCAN https://github.com/james-yoo/DBSCAN
		if(1==0){
			// constructor
			vector<PointCluster> PointsClustered;
			for (int i = 0; i < dronePoints.size(); ++i)
			{				
				PointCluster newPoint;
				newPoint.clusterID = UNCLASSIFIED;
				newPoint.x = dronePoints[i].x;
				newPoint.y = dronePoints[i].y;
				newPoint.z = 0;
				PointsClustered.push_back(newPoint);
			}
			//#define MINIMUM_POINTS 4     // minimum number of cluster
			//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
		    	DBSCAN ds(3, 110*110, PointsClustered);  //DBSCAN ds(MINIMUM_POINTS, 100*100, PointsClustered);

		    	// main loop
		    	ds.run();

		    	// result of DBSCAN algorithm		    	
			for (int i = 0; i < ds.m_points.size(); ++i)
			{					
				//circle(canvas, Point(ds.m_points[i].x, ds.m_points[i].y), 15,  CV_RGB(110 * ds.m_points[i].clusterID, 255, 255), -1); //roz - kokkino
				cout << " cluster ID:" << ds.m_points[i].clusterID  << "  i=" << i  << endl; 
			}

		}//END CLUSTER WITH DBSCAN	


		//READ PTZ DATA THREAD ASYNCH
		if(toggleThread == 0){
			std::thread t1(task1);
			t1.detach();
			toggleThread = 1;
		}else{
			std::thread t2(task2);
			t2.detach();
			toggleThread = 0;
		}
		//END READ PTZ DATA THREAD ASYNCH


		// Display FPS on frame	
		float fps = getTickFrequency() / ((double)getTickCount() - timer);
		if(plotDebug){
			putText(canvas, "FPS : " + SSTR(int(fps)) + " Frame:" + SSTR(frameID) + "/" + SSTR(max_frames), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
		}		
	


		//PLOT FROM FILE FOR DEBUG PUROSES
		if(plotFromFile){
			cout << "pointsCSV[frameID]" << pointsCSVx[frameID] << "," << pointsCSVy[frameID] <<"," << endl;
			Rect2d bboxCSV(pointsCSVx[frameID], pointsCSVy[frameID], pointsCSVw[frameID], pointsCSVh[frameID]);
			rectangle(canvas, bboxCSV.tl(), bboxCSV.br(), CV_RGB(255, 255, 255), 12, 18, 0);
			Rect2d bboxCSVM(pointsCSVxM[frameID], pointsCSVyM[frameID], pointsCSVwM[frameID], pointsCSVhM[frameID]);
			rectangle(canvas, bboxCSVM.tl(), bboxCSVM.br(), CV_RGB(255, 0, 0), 12, 18, 0);
		}
		//END PLOT FROM FILE FOR DEBUG PUROSES	



		
		if(!usePTZ && cameraPan.size() > frameID+1 && cutPropellers && MavicProjectedPos.size() > frameID+frames_back_sync ){			
			//PLOT MAVIC PRO POSITION FROM VIKON - MavicProjectedPos
			circle(canvas, MavicProjectedPos[frameID-frames_back_sync]/scaler, 14*scaler,  CV_RGB(42/112, 2/112, 220/121), -1);
		}



		if(plotDebug && cutPropellers){	//if(!usePTZ && cameraPan.size() > frameID+1){			
			//PLOT MAVIC PRO POSITION FROM VIKON - MavicProjectedPos
			//circle(canvas, MavicProjectedPos[frameID], 14,  CV_RGB(42/112, 2/112, 220/121), -1);

			for(int i=0;i< upperLeftPropellerContour.size();i++){
				circle(canvas, upperLeftPropellerContour[i], 5,  CV_RGB(242/1, 2/112, 220/121), -1);
				if(i<upperLeftPropellerContour.size()-1 && !allULout){
					line(canvas,upperLeftPropellerContour[i], upperLeftPropellerContour[i+1],CV_RGB(255, 255, 255) , 1, 8,0); 
				}
			}
			for(int i=0;i< lowerLeftPropellerContour.size();i++){
				circle(canvas, lowerLeftPropellerContour[i], 5,  CV_RGB(142/1, 2/112, 220/121), -1);
				if(i<lowerLeftPropellerContour.size()-1 && !allLLout){
					line(canvas,lowerLeftPropellerContour[i], lowerLeftPropellerContour[i+1],CV_RGB(255, 255, 255) , 1, 8,0); 
				}
			}

			//lowerRightPropellerContour
			for(int i=0;i< upperRightPropellerContour.size();i++){
				circle(canvas, upperRightPropellerContour[i], 5,  CV_RGB(242/1, 2/112, 220/121), -1);
				if(i<upperRightPropellerContour.size()-1 && !allURout){
					line(canvas,upperRightPropellerContour[i], upperRightPropellerContour[i+1],CV_RGB(255, 255, 255) , 1, 8,0); 
				}
			}
			for(int i=0;i< lowerRightPropellerContour.size();i++){
				circle(canvas, lowerRightPropellerContour[i], 5,  CV_RGB(142/1, 2/112, 220/121), -1);
				if(i<lowerRightPropellerContour.size()-1 && !allLRout){
					line(canvas,lowerRightPropellerContour[i], lowerRightPropellerContour[i+1],CV_RGB(255, 255, 255) , 1, 8,0); 
				}
			}

			//CONVEX HULL LEFT
			for (int i=0; i<LeftPropellerContourHull.size(); i++){
				if(i < LeftPropellerContourHull.size() - 1){
					line(canvas,LeftPropellerContourHull[i], LeftPropellerContourHull[i+1],CV_RGB(255, 1, 1) , 1, 8,0); 
				}
			}
			line(canvas,LeftPropellerContourHull[0], LeftPropellerContourHull[LeftPropellerContourHull.size()-1],CV_RGB(255, 1, 1) , 1, 8,0); 
			//CONVEX HULL RIGHT
			for (int i=0; i<RightPropellerContourHull.size(); i++){
				if(i < RightPropellerContourHull.size() - 1){
					line(canvas,RightPropellerContourHull[i], RightPropellerContourHull[i+1],CV_RGB(255, 1, 1) , 1, 8,0); 
				}
			}
			line(canvas,LeftPropellerContourHull[0], LeftPropellerContourHull[LeftPropellerContourHull.size()-1],CV_RGB(255, 1, 1) , 1, 8,0); 
		}



if(plotDebug){
		if(showFeedPreview){ //
			imshow("before and after", canvas);
		}
		if(PTZ_HOMOGRAPHY_FUSION && plotDebugTrackingFUSION){
			imshow("FUSION RESULT", canvas);
		}
}


		//waitKey(10);
		//int k = waitKey(1);
		//kbhit();
		//KEYBOARD HANDLING
		if(1==0 && kbhit() == 0){
			//return 0;
			outfile.close();
			video.release();
			//videoOutA.release();
			//videoOutB.release();
			//videoOutC.release();
			//out_transform.close();
			//out_trajectory.close();
			//out_smoothed_trajectory.close();
			//out_new_transform.close();
			break;		
		}


		
		prev = cur.clone();
		cur_grey.copyTo(prev_grey);

		//cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
		k++;

		// Write the frame into the file 'outcpp.avi'	
  //  		video.write(canvas);	


		//PLOT TO YOLO 4 window
		float red = 255*get_color(2,1,80);
		float green = 255*get_color(1,1,80);
		float blue = 255*get_color(0,1,80);
		const auto color = Scalar(blue, green, red);
		//cv::rectangle(frame, cv::Point(prevRect.x, prevRect.y), cv::Point(prevRect.x + prevRect.width, prevRect.y + prevRect.height), color/2, 3);
		if(plotDebug){
			rectangle(frame, trackFrameEstimateKALMAN.tl() * (1/scaleFrame), trackFrameEstimateKALMAN.br() * (1/scaleFrame), CV_RGB(255, 0, 255), 2, 8, 0);//MAGENTA PINK RECTANGLE
		}
		//if(1==0 && !foundWindow) /////////////////////////////////////// TO CHECK !!!!!!!!!!!!!!!!!!!!!!!
		if(!foundWindow && !foundWindowInCropped)
		{
			prevRect.x =  trackFrameEstimateKALMAN.x * (1/scaleFrame);// - trackFrameEstimateKALMAN.width* (1/scaleFrame);
			prevRect.y =   trackFrameEstimateKALMAN.y * (1/scaleFrame);// - trackFrameEstimateKALMAN.height* (1/scaleFrame);
			prevRect.width =   trackFrameEstimateKALMAN.width * (1/scaleFrame);// * 3;
			prevRect.height =  trackFrameEstimateKALMAN.height * (1/scaleFrame);// * 3;
			foundWindow = false;
			foundWindowInCropped = false;

			//prevRect.x = 0;// - trackFrameEstimateKALMAN.width* (1/scaleFrame);
			//prevRect.y =   0;// - trackFrameEstimateKALMAN.height* (1/scaleFrame);
			//prevRect.width =   frame.cols;// * 3;
			//prevRect.height =  frame.rows;// * 3;
			foundWindow = false;
			foundWindowInCropped = false;
			
		}

	     }//AND YOLO 4 window found test


		//KEYBOARD HANDLING
		if(kbhit() == 0){
			//return 0;
			outfile.close();
			video.release();			
			break;		
		}


		//MOSSE
		Point centerA(bbox2.x* (1/scaleFrame)+bbox2.width* (1/scaleFrame)*(1/2), bbox2.y * (1/scaleFrame)+bbox2.height* (1/scaleFrame)*(1/2));
		Point centerB(currentYoloRect.x + currentYoloRect.width/2, currentYoloRect.y + currentYoloRect.height/2);
		float distMOSSECenter = sqrt( pow((centerA.x-centerB.x),2) + pow((centerA.y-centerB.y),2) );
		//cout << " distMOSSECenter !!!!!!!!!!!!!!!!!!!!!!!!!! = " << distMOSSECenter << endl;
		okB = tracker->update(cur, bbox2);
		if(!okB || distMOSSECenter > 40){//260
			bbox2.x = currentYoloRect.x * scaleFrame - 10;
			bbox2.y = currentYoloRect.y * scaleFrame - 10;
			bbox2.width =  currentYoloRect.width * scaleFrame + 20;  //300;// currentYoloRect.width * scaleFrame;
			bbox2.height = currentYoloRect.height * scaleFrame + 20; //300;//currentYoloRect.height * scaleFrame;
			tracker->init(cur, bbox2);
			//okB = tracker->update(cur, bbox2);
		}
		//okB = tracker->update(cur, bbox2);			
		if(plotDebug){
			rectangle(frame, bbox2.tl()* (1/scaleFrame), bbox2.br()* (1/scaleFrame), CV_RGB(255, 111, 1), 2, 8, 0);
		}

		//v1.3
		if(prevRect.x == prevPrevRectX || prevRect.x == -200){ //if(!foundWindow){ //if(prevRect.x < -100){
			//prevRect.x = 0;
			//foundWindow = false;
			//foundWindowInCropped = false;
			//LABELS
			float red = 255*get_color(2,1,80);
						float green = 255*get_color(1,1,80);
						float blue = 255*get_color(0,1,80);
						const auto color = Scalar(blue, green, red);
						std::ostringstream label_ss;
						label_ss << classNamesVec[0] << ": " << std::fixed << std::setprecision(2) << currentScore;
						auto label = label_ss.str();
						int baseline;
						auto label_bg_sz = getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
						if(plotDebug){
							rectangle(frame, Point(currentYoloRect.x, currentYoloRect.y - label_bg_sz.height - baseline - 10), cv::Point(currentYoloRect.x + label_bg_sz.width, currentYoloRect.y), color, cv::FILLED);
							putText(frame, label.c_str(), Point(currentYoloRect.x, currentYoloRect.y - baseline - 5), FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
							rectangle(frame, currentYoloRect.tl()* (1/1), currentYoloRect.br()* (1/1), CV_RGB(255, 1, 1), 2, 8, 0);
						}

		}//else{
			prevPrevRectX = prevRect.x;
		//}








//LOG FILE
		//rectangle(plotFrame, bbox2, Scalar( 255, 255, 255 ), 2, 1 );		
		if(logData){
			//int frames_back_sync = 2;//5
			float addVikonPosition2Dx = 0;
			float addVikonPosition2Dy = 0;
			if(plotFromVikon && !usePTZ){
				addVikonPosition2Dx = (MavicProjectedPos[frameID-frames_back_sync].x/scaler);
				addVikonPosition2Dy = (MavicProjectedPos[frameID-frames_back_sync].y/scaler);
			}
			float logGPSPosLat = 0;
			float logGPSPosLon = 0;
			float logGPSPosAlt = 0;
			float logGPSTime = 0;
			if(logGPS){
				logGPSPosLat = GPSPosLat;
				logGPSPosLon = GPSPosLon;
				logGPSPosAlt = GPSPosAlt;
				logGPSTime = GPSTime;
			}
			//WRITE DATA	

			string addVikonPosition2D = "";
			if(plotFromVikon && !usePTZ){
				addVikonPosition2D = "addVikonPosition2Dx;addVikonPosition2Dy;";
			}			
			
			double seconds_since_start = difftime( time(0), startTime);
			double timeDiffNow = ((double)getTickCount() - timerSTART);
			//convert millisconds to seconds
					auto start = ros::Time::now();// std::chrono::system_clock::now();	
								
					outfile << frameID << ";" << start << ";"	
					<< bbox2.x * (1/scaleFrame) << ";"
					<< bbox2.y * (1/scaleFrame)  << ";"
					<< bbox2.width * (1/scaleFrame) << ";" 
					<< bbox2.height * (1/scaleFrame) << ";" 
					<< currentYoloRect.x << ";" 
					<< currentYoloRect.y << ";" 
					<< currentYoloRect.width << ";" 
					<< currentYoloRect.height << ";" 
					<< currentScore << ";"
					<< prevRect.x << ";" 
					<< prevRect.y << ";" 
					<< prevRect.width << ";" 
					<< prevRect.height << ";" 
					<< YoloA_enter_and_found << ";" 
					<< YoloB_enter_and_found << ";" 
					<< Mosse_enter_and_found << ";" 
					<< Homog_enter_and_found << ";" 
					<< currentPan << ";" << currentTilt << ";" << currentZoom << ";" //<< YawRate << ";" << pitchRate << ";"<< rollRate << ";" << zoomRate << ";"
					<< std::endl;
				
		}
		//END LOG FILE
		//END PTZ - ROS - LOGS





		//NEW1 - state recorded plot here
		//cout << "YoloA_enter_and_found =" << YoloA_enter_and_found << " YoloB_enter_and_found =" << YoloB_enter_and_found 
		//<< " Mosse_enter_and_found =" << Mosse_enter_and_found << " Homog_enter_and_found =" << Homog_enter_and_found << endl;

		string counterString1 = "YoloA_enter_and_found =" + to_string(YoloA_enter_and_found) + " YoloB_enter_and_found =" + to_string(YoloB_enter_and_found) 
		+ " Mosse_enter_and_found =" + to_string(Mosse_enter_and_found) + " Homog_enter_and_found =" + to_string(Homog_enter_and_found);

		string counterString2 = "YoloA_enter_and_found1 =" + to_string(YoloA_enter_and_found1) + " YoloB_enter_and_found1 =" + to_string(YoloB_enter_and_found1) 
		+ " Mosse_enter_and_found1 =" + to_string(Mosse_enter_and_found1) + " Homog_enter_and_found1 =" + to_string(Homog_enter_and_found1);

		string counterString3 = "YoloA_enter_and_found2 =" + to_string(YoloA_enter_and_found2) + " YoloB_enter_and_found2 =" + to_string(YoloB_enter_and_found2) 
		+ " Mosse_enter_and_found2 =" + to_string(Mosse_enter_and_found2) + " Homog_enter_and_found2 =" + to_string(Homog_enter_and_found2);
		
		

		//YOLO 4
		double after  = get_time_point();		
		fps = 1000000. / (after - before);
		string fpsString = to_string(fps) + " Frame:" + SSTR(frameID) + "/" + SSTR(max_frames);

	//	if(frameID % 10 == 0) {
			cout << "frame ID: " << SSTR(frameID) << "  Current Pan, Tilt, Zoom = " << currentPan << " ," << currentTilt << " ," << currentZoom 			
			<< endl;
			cout << "MOSSE:" << (int)(bbox2.x * (1/scaleFrame)) << "," << (int)(bbox2.y * (1/scaleFrame))  << "," << (int)(bbox2.width * (1/scaleFrame)) << "," << (int)(bbox2.height * (1/scaleFrame))
			<< endl;
			cout
			<< "YOLO:" << prevRect.x << "," << prevRect.y << "," << prevRect.width << "," << prevRect.height
			<< endl;	
	//	}

		if(plotDebug){
			putText(frame, fpsString, Point(50,50), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(120, 120, 120),2);	

			putText(frame, counterString1, Point(50,70), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 20, 220),2);	
			putText(frame, counterString2, Point(50,100), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 20, 220),2);
			putText(frame, counterString3, Point(50,130), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 20, 220),2);
		

	     		imshow("video",frame);
		}
		int c=waitKey(1);

		//cout << "F_WIDTH = " << frame.cols << endl;
		//cout << "F_H ==== " << frame.rows << endl;
		video.write(frame);

		//foundWindow = false; /////////////////////////////////////// TO CHECK !!!!!!!!!!!!!!!!!!!!!!!



waitKey(5);





		////// YOLO 4 PTZ CONTROLS
		//// PTZ CONTROLS
		//ZOOM - PAN TEST SIN !!!!
		if(1==1 && !enablePIDracking && usePTZ){	
					//cout << "frameCounter="  << frameID  << endl;
					//Pan is 0 at home. Right is positive, max 2448. Left ranges from full
					//left 63088 to 65535 before home.
					//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully
					//depressed at 65104 to 65535 before home.  Zoom is 0 to 16384					
					if(frameID < 50){//if(frameCounter < 1100){
						//go home posiion				
						std::string homePTZ = "81010604FF";
						udpSend(homePTZ, false, false);
						//manual focus				
						std::string focusPTZ = "8101043803FF";//"8101043802FF";//std::string focusPTZ = "8101043803FF";
						udpSend(focusPTZ, false, false);
						//stop motion
						string WW = "00";
						std::string panPTZ = "81010601"+WW+WW+"0303FF";
						//udpSend(panPTZ, false, false);
						//zoom out
						string PP = "2";//zoom speed 0 to 7
						std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
						udpSend(zoomPTZ, false, false);
					}else{
						//ZOOM IN	
						if(1==0 && enableTestZoom){			
							//string PP = "1";//zoom speed 0 to 7
							int zoomSpeed = 1;
							if(currentZoom > 5800){//if(currentZoom > 4800){
								string PP = "0";//zoom speed 0 to 7
								std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out = 810104073pFF
								udpSend(zoomPTZ, false, false);
								zoomRate = -zoomSpeed * 120 / 7;
							}else if(currentZoom < 5){
								string PP = "0";
								std::string zoomPTZ = "810104072"+PP+"FF"; //zoom in  = 810104072pFF
								udpSend(zoomPTZ, false, false);
								zoomRate = zoomSpeed * 120 / 7;
							}
							//cout << "currentZoom=" << currentZoom << endl;							
						}

						int randomNum = rand() % 4;

						//		cout << "currentPan="  << currentPan  << endl;							
						//HANDLE PAN - TILT combination
						if(1==0 && enableTestPanTilt){
							string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							string WW = "02";
							int panSpeed = 2;
							int tiltSpeed = 2;
							if(currentTilt > 180 && currentTilt < 50000){  //if(currentTilt > 250 && currentTilt < 50000){							
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //if tilt looks up, go down right
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
							else if(currentTilt < 1 || currentTilt > 65300){
								std::string panPTZ = "81010601"+VV+WW+"0101FF";
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;	
								udpSend(panPTZ, false, false);			
							}											
											
						}	
						//cout << "currentTilt="  << currentTilt  << endl;
					}
		}/////END ZOOM - PAN TEST SIN !!!!


		int zoomBase = 1500;

		//IMPLEMENT PTZ CONTROLS, 5678 = TCP port
		//if(toggleCommandrate == 0 && enablePIDracking && usePTZ){
		if(enablePIDracking && usePTZ){


				if (YoloA_enter_and_found == 2 || YoloB_enter_and_found == 2 || Homog_enter_and_found == 2){			
								
								not_found_long_time = 0;			
				}


				toggleCommandrate = 3;

				//const char* homePTZ = "81010604FF";
				//std::string homePTZ = "81010604FF";
				//udpSend(homePTZ, false);
				//float PID_P = 0.75f;//0.75f;
				float ScreenCenterX = frame.cols/2; 
				float ScreenCenterY = frame.rows/2; 
				//float PID_X_DIFF = (bbox2DIFF.x + (bbox2DIFF.width/2) - ScreenCenterX);
				//float PID_Y_DIFF = (bbox2DIFF.y + (bbox2DIFF.height/2) - ScreenCenterY);
				float PID_X_DIFF = (bbox2.x* (1/scaleFrame) + ( (1/scaleFrame)*bbox2.width/2) - ScreenCenterX);
				float PID_Y_DIFF = (bbox2.y* (1/scaleFrame) + ((1/scaleFrame)*bbox2.height/2) - ScreenCenterY);
				//float PID_Control_Pan = PID_P * PID_X_DIFF;
				//float PID_Control_Pan = PID_P * PID_X_DIFF;
				//float maxPanSpeed = 8;
				float proximityDivider = 22;
				//cout << "ok:" << ok << " okA:" << okA << " okB:" << okB << endl;

				bool box2Same = false;
				if(bbox2P.x == bbox2.x && bbox2P.y == bbox2.y){
					box2Same = true;
					bbox2P.x = bbox2.x;
					bbox2P.y = bbox2.y;
					bbox2P.width = bbox2.width;
					bbox2P.height = bbox2.height;
				}


				if(zoomingOut && currentZoom <= zoomBase){ //currentZoom == zoomBase){
					zoomingOut = false;
				}

				//if(box2Same || !okB || okB == 0 || abs(PID_X_DIFF) > ScreenCenterX * 0.85f || frameCounter < 110 || abs(PID_Y_DIFF) > ScreenCenterY * 0.85f ){
				//if(box2Same || !okB || okB == 0  || frameID < 50 ){//if(box2Same || !okB || okB == 0  || frameCounter < 50 ){		
				//if(box2Same || frameID < 50 || abs(PID_X_DIFF) > 100 || abs(PID_Y_DIFF) > 100 ){
				//if(box2Same || frameID < 50  || abs(PID_X_DIFF) > ScreenCenterX * 0.85f || abs(PID_Y_DIFF) > ScreenCenterY * 0.85f ||  
				//	(threeFrameConfidence && abs(trackFrameEstimateKALMAN.x - bbox2.x) > 250 &&  abs(trackFrameEstimateKALMAN.y - bbox2.y) > 250 ) 
				//	|| bbox2.width > canvas.cols/5
				//cout << "abs(PID_X_DIFF):" << abs(PID_X_DIFF) << ", " <<  abs(PID_Y_DIFF) << endl;
				bool tiltTest = (currentTilt > 1000 && currentTilt < 60000) || (currentTilt < 65255 && currentTilt >= 65104) ;
				if(	//abs(PID_X_DIFF) > ScreenCenterX * 0.65f && abs(PID_Y_DIFF) > ScreenCenterY * 0.65f 
 					frameID < 50 // || !okB || okB == 0   //
					//|| abs(PID_X_DIFF) > ScreenCenterX * 0.65f || abs(PID_Y_DIFF) > ScreenCenterY * 0.65f 
					//|| (threeFrameConfidence && abs(trackFrameEstimateKALMAN.x - bbox2.x) > 850 &&  abs(trackFrameEstimateKALMAN.y - bbox2.y) > 850 ) 
					//|| bbox2.width > canvas.cols/3 || bbox2.height > canvas.rows/3
					|| tiltTest
					//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully
					//depressed at 65104 to 65535 before home
					//|| box2Same
					////////////////////////////////////////////////|| dronePoints.size() > 160
					|| (zoomingOut && currentZoom > zoomBase)
					//|| 
					|| (YoloA_enter_and_found != 2 && YoloB_enter_and_found != 2 && Homog_enter_and_found != 2)
				){		

//					cout << "Reseting .........................................." << PID_X_DIFF << "," << PID_Y_DIFF << endl;		
					//zoom out
					//if(frameID < 50 || dronePoints.size() > 160 || (zoomingOut && currentZoom > 0) ){
					if(frameID < 50 || (zoomingOut && currentZoom > zoomBase) ){

						if(!tiltTest && frameID > 50){
							string PP = "7";//zoom speed 0 to 7
							std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
		//					udpSend(zoomPTZ, false, false);
							//cout << "zooming out ... " << currentZoom << endl;
						}else{
							//go home posiion				
							std::string homePTZA = "81010604FF";
		//					udpSend(homePTZA, false, false);
							//reset mosse to center
							//bbox2.x = canvas.cols/2;//bboxes[bboxes.size()-2].x;
							//bbox2.y = canvas.rows/2;
							//bbox2.width =bbox2.width;
							//bbox2.height =bbox2.height;
							//tracker->init(prev, bbox2);
							//okB = tracker->update(cur, bbox2);

							
							if(frameID < 50){
								bbox2.x = -100;
								bbox2.y = -100;
								bbox2.width = 10;
								bbox2.height = 10;
								tracker->init(prev, bbox2);
								okB = tracker->update(cur, bbox2);							
							}
						}

						zoomingOut = true;
						
					}else{
					//STOP ZOOM					
						std::string zoomStopPTZ = "8101040700FF"; //stop zoom
						//udpSend(zoomStopPTZ, false, false);
						zoomRate=0;

						if(frameID % 10 == 0) {
							cout << "zooming stopped at ... " << currentZoom << endl;
						}

						string PP = "1";//zoom speed 0 to 7 //2
								std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF		//// ZOOM OUT
				//				udpSend(zoomPTZ, false, false);
//
								//zoomSpeed = 2;
								//zoomRate = -zoomSpeed * 120 / 7;

								//std::string homePTZ = "81010604FF";
								//udpSend(homePTZ, false, false);
					}					

					//cout << "RESETTING" << endl;
					//std::string getPanTiltPTZ = "81090612FF"; //read pan - tilt
					//udpSend(getPanTiltPTZ, true, false);

					///HANDLE PAN						
					//go home posiion				
					std::string homePTZ = "81010604FF";
					if(1==0 && frameID < 50 || tiltTest){// || dronePoints.size() > 160){
						//udpSend(homePTZ, false, false);


						//reset mosse to center
						bbox2.x = frame.cols/2;//bboxes[bboxes.size()-2].x;
						bbox2.y = frame.rows/2;
						bbox2.width =bbox2.width;
						bbox2.height =bbox2.height;
						tracker->init(prev, bbox2);
						okB = tracker->update(cur, bbox2);

					}
					//manual focus				
					std::string focusPTZ = "8101043803FF";//"8101043802FF";//std::string focusPTZ = "8101043803FF";
					//	udpSend(focusPTZ, false, false);


					//HANDLE PAN - TILT combination to GO HOMING POSITION
					if(1==0 && frameID >= 50 && !tiltTest){
						if (YoloA_enter_and_found != 2 && YoloB_enter_and_found != 2 && Homog_enter_and_found != 2){	
							//string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							//string WW = "02";

							float HOME_X_DIFF = currentPan;
							float HOME_Y_DIFF = currentTilt;//(bbox2.y* (1/scaleFrame) + ((1/scaleFrame)*bbox2.height/2) - ScreenCenterY);

							float tiltDiffF =PID_P * maxPanSpeed * (abs(HOME_Y_DIFF) / 65000);					
							int tiltDiff = (int)(tiltDiffF);
							string WW = to_string(tiltDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(tiltDiff < 10){
								//add pading
								WW = "0"+WW;
							}
							//string WW = "01";

							float panDiffF = PID_P * maxPanSpeed * (abs(HOME_X_DIFF) / 65000);
							int panDiff = (int)(panDiffF);
							string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(panDiff < 10){
								//add pading
								VV = "0"+VV;
							}

							int panSpeed = panDiff;
							int tiltSpeed = tiltDiff;
							float diffThreshold = 1;
							if(HOME_X_DIFF > diffThreshold && HOME_Y_DIFF <= diffThreshold){ //go up - right
								std::string panPTZ = "81010601"+VV+WW+"0201FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(HOME_X_DIFF > diffThreshold && HOME_Y_DIFF > diffThreshold){ //go down - right
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(HOME_X_DIFF <= diffThreshold && HOME_Y_DIFF <= diffThreshold){ //go up - left
								std::string panPTZ = "81010601"+VV+WW+"0101FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(HOME_X_DIFF <= diffThreshold && HOME_Y_DIFF > diffThreshold){ //go down - left
								std::string panPTZ = "81010601"+VV+WW+"0102FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
						}											
					}	
					//END HANDLE PAN - TILT combination, also check if has lost tracking for long time
					else{

						//stop motion
						string WW = "00";
						std::string panPTZ = "81010601"+WW+WW+"0303FF";
						if(frameID >= 50 && !tiltTest){	
							if (YoloA_enter_and_found != 2 && YoloB_enter_and_found != 2 && Homog_enter_and_found != 2){				
								udpSend(panPTZ, false, false);	
								not_found_long_time++;			
							}

							

							//go home posiion
							if (not_found_long_time == 50){				
								std::string homePTZA = "81010604FF";
								udpSend(homePTZA, false, false);
								not_found_long_time =0;
							}
						}
						panRate = 0;
						tiltRate = 0;
						zoomRate = 0;

					}
				}else{
					//RESET TRACK WINDOWS CANDIDATES
					//bboxesESTIMATED_STATIC.clear();
					//bboxESTIMATE_STATIC_frames.clear();											

					//PAN MOTION ENABLE					

					//COMBINED PAN TILT
					//HANDLE PAN - TILT combination
					if(1==1){
							//string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							//string WW = "02";

							float tiltDiffF =PID_P * maxPanSpeed * (abs(PID_Y_DIFF) / ScreenCenterY);					
							int tiltDiff = (int)(tiltDiffF);
							string WW = to_string(tiltDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(tiltDiff < 10){
								//add pading
								WW = "0"+WW;
							}
							//string WW = "01";

							float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
							int panDiff = (int)(panDiffF);
							string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							if(panDiff < 10){
								//add pading
								VV = "0"+VV;
							}

							int panSpeed = panDiff;
							int tiltSpeed = tiltDiff;
							float diffThreshold = 1;
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - right
								std::string panPTZ = "81010601"+VV+WW+"0201FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - right
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - left
								std::string panPTZ = "81010601"+VV+WW+"0101FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}else
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - left
								std::string panPTZ = "81010601"+VV+WW+"0102FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}											
					}	
					//END HANDLE PAN - TILT combination

					//ZOOM MOTION ENABLE
					if(enablePIDZoom && 1==1){
						int zoomSpeed = 2; //3
						//cout << "currentZoom=" << currentZoom << " PID_X_DIFF=" << PID_X_DIFF << " PID_Y_DIFF=" << PID_Y_DIFF << endl;
						//if(abs(PID_X_DIFF) < ScreenCenterX/proximityDivider && abs(PID_Y_DIFF) < ScreenCenterY/proximityDivider){ //when centered, zoom in
						//if(abs(PID_X_DIFF) < 950 && abs(PID_Y_DIFF) < 950 && currentZoom < maxPTZ_Zoom){// && currentZoom < maxPTZ_Zoom){ //when centered, zoom in 1250 and 2400
						if(abs(PID_X_DIFF) < frame.cols/4 && abs(PID_Y_DIFF) < frame.rows/4 && currentZoom < maxPTZ_Zoom && bbox2.width* (1/scaleFrame) < frame.cols/4
							&& bbox2.x * (1/scaleFrame) > 0 && bbox2.y * (1/scaleFrame) > 0 && bbox2.x * (1/scaleFrame) < frame.cols && bbox2.y * (1/scaleFrame) < frame.rows
							&& (YoloA_enter_and_found == 2 || YoloB_enter_and_found == 2 || Homog_enter_and_found == 2)	 						
						){
							//stop motion
							//string WW = "00";
							//std::string panPTZ = "81010601"+WW+WW+"0303FF";
							//udpSend(panPTZ, false, false);
							//ZOOM IN				
							string PP = "2";//zoom speed 0 to 7 //5
							std::string zoomPTZ = "810104072"+PP+"FF"; //zoom out, zoom in = 810104072pFF
							udpSend(zoomPTZ, false, false);

							zoomSpeed = 1;
							zoomRate = zoomSpeed * 120 / 7;
							//cout << "zooming in to ... " << currentZoom << endl;
						}else{					

							//if(abs(PID_X_DIFF) > 1050 || abs(PID_Y_DIFF) > 1050){
							if(abs(PID_X_DIFF) > frame.rows/4 || abs(PID_Y_DIFF) > frame.rows/4){
								string PP = "4";//zoom speed 0 to 7 //2
								std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
								//udpSend(zoomPTZ, false, false);

								zoomSpeed = 2;
								zoomRate = -zoomSpeed * 120 / 7;

								//cout << "zooming OUT to ... " << currentZoom << endl;
							}else{
								//STOP ZOOM					
								std::string zoomStopPTZ = "8101040700FF"; //stop zoom
								udpSend(zoomStopPTZ, false, false);
								zoomRate=0;
								//cout << "zooming 2 stopped at ... " << currentZoom << endl;
							}
						}						
					}				
	  			}			
		}
		//END IMPLEMENT PTZ CONTROLS, 5678 = TCP port	
		//// END PTZ CONTOLS
		//else 

		if(1==1){

			//ADD ZOOM OFFSET
			if(frameID > 50){
				if(currentZoom < zoomBase - 800){
					string PP = "1";//zoom speed 0 to 7 //5
					std::string zoomPTZ = "810104072"+PP+"FF"; //zoom in = 810104072pFF
					udpSend(zoomPTZ, false, false);
				}else if(currentZoom > zoomBase + 800 ){ 
					string PP = "1";//zoom speed 0 to 7 //5
					std::string zoomPTZ = "810104073"+PP+"FF"; //zoom in = 810104072pFF
					udpSend(zoomPTZ, false, false);
				}else{
					std::string zoomStopPTZ = "8101040700FF"; //stop zoom
					udpSend(zoomStopPTZ, false, false);
					zoomRate=0;
				}
			}
	
		}
		////// END YOLO 4 PTZ CONTROLS











	}//END WHILE

	cap.release();
  	video.release();

	return 0;
}

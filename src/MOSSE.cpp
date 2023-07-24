//https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/
//cd ~/openCVmine/testROS/ocams/devel/lib/ocams

//ROS
//#include <ros/ros.h>
//#include <nodelet/nodelet.h>
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/distortion_models.h>
//#include <image_transport/image_transport.h>
//#include <boost/optional.hpp>
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/ini_parser.hpp>
//#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32.h>

#include <iomanip>
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}
//END ROS

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp> 

#include <opencv2/core/ocl.hpp>
//#include <thread>

#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <fstream>
#include <sstream>
#include <cassert>
#include <cmath>

#include "opencv2/optflow.hpp"
#include <opencv2/core/ocl.hpp> //GPU OPENCL

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

using namespace cv;
using namespace std;
using namespace optflow; ///GPU OPENCL

//GLOBAL VARIABLES
int ch=0;
char inputTERMINAL;
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
float panRate = 0; float tiltRate = 0; float zoomRate = 0;
int stopTrainBackgroundCounter = 0; //start training background if found static, and not train more for "stopTrainBackgroundCounter" frames
bool backStatic = false; //if flow map vectors low length, assume background static
double emulatedCameraDirectionX = 0;
double emulatedCameraDirectionY = 0;
vector<vector<Point>> trajectory_Poins;
vector<float> trajectory_Poins_scale;
float maxBrightMOSSE = 1;


//CONFIDENSE MEASURES FOR TRAJECTORY DECISION
int successionMasure = 0;
int section = 0;
string debugActionsString = "";
int confid = 0;

//CLUSTERING
#include "./tools/kmeansNEW.cpp"

//MOSSE TWEAKBLE
#include "./mosse2Tracker.cpp"

//GPU
//#include "GPUMosse.h"
Rect2d trackWindow;
vector<Point> prevCenters; 
vector<float> prevAreas;
Mat img1_prev;
Mat back_sub_prev;
float sourceWidth, sourceHeight;
Mat out3;
Mat fgMaskMOG2Plot;

//-------------------------------------------------------IMAGE STABILIZER rolling window --------------------------------------------------- ///
// This video stablisation smooths the global trajectory using a sliding average window

//const int SMOOTHING_RADIUS = 15; // In frames. The larger the more stable the video, but less reactive to sudden panning
const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct TrajectoryC
{
    TrajectoryC() {}
    TrajectoryC(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }
	// "+"
	friend TrajectoryC operator+(const TrajectoryC &c1,const TrajectoryC  &c2){
		return TrajectoryC(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
	}
	//"-"
	friend TrajectoryC operator-(const TrajectoryC &c1,const TrajectoryC  &c2){
		return TrajectoryC(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
	}
	//"*"
	friend TrajectoryC operator*(const TrajectoryC &c1,const TrajectoryC  &c2){
		return TrajectoryC(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
	}
	//"/"
	friend TrajectoryC operator/(const TrajectoryC &c1,const TrajectoryC  &c2){
		return TrajectoryC(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
	}
	//"="
	TrajectoryC operator =(const TrajectoryC &rx){
		x = rx.x;
		y = rx.y;
		a = rx.a;
		return TrajectoryC(x,y,a);
	}

    double x;
    double y;
    double a; // angle
};
//END IMAGE STABLIZER
//-------------------------------------------------------END IMAGE STABILIZER rolling window --------------------------------------------------- //



//PTZ CONTROLS
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
 




//#include "./tools/kmeansNEW.cpp"


//DEFINE GLOBAL TRACKER HERE !!!!
#define trackerUsed TrackerMOSSE
//#define trackerUsed TrackerBoosting
//#define trackerUsed TrackerMIL
//#define trackerUsed TrackerKCF
//#define trackerUsed TrackerTLD
//#define trackerUsed TrackerMedianFlow
//#define trackerUsed TrackerGOTURN
//#define trackerUsed TrackerCSRT


//KEYBOARD CONTROLS
//Keyboard function used to terminate the program on user input
int kbhit(void)
{
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
				cout << "  Disabled PID ..." << endl;
		}else{
				enablePIDracking = true;
				cout << "  Enabled PID ..." << endl;
		}
	}else if (ch == 122 ||ch == 90){ //z
		if(enablePIDracking){
		  	if(enablePIDZoom){
					enablePIDZoom = false;
					cout << "  Disabled PID Zoom ..." << endl;
			}else{
					enablePIDZoom = true;
					cout << "  Enabled PID Zoom ..." << endl;
			}
		}else{
			if(enableTestZoom){
					enableTestZoom = false;
					cout << "  Disabled TEST PAN TILT Zoom ..." << endl;
			}else{
					enableTestZoom = true;
					cout << "  Enabled TEST PAN TILT Zoom ..." << endl;
			}
			
		}
	
	}else if (ch == 112 ||ch == 80){ //p
			//bool enableTestPanTilt = true;	
			if(enableTestPanTilt){
					enableTestPanTilt = false;
					cout << "  Disabled TEST PAN TILT ..." << endl;
			}else{
					enableTestPanTilt = true;
					cout << "  Enabled TEST PAN TILT ..." << endl;
			}
	}else if (ch == 115 ||ch == 83){ //s - sparkfun sensor on - off
			//bool enableTestPanTilt = true;	
			if(useSparkfun){
					useSparkfun = false;
					cout << "  Disabled Sparkfun ..." << endl;
			}else{
					useSparkfun = true;
					cout << "  Enabled Sparkfun ..." << endl;
			}
	}else if (ch == 114 ||ch == 82){ //r - reset
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
		cout << "  Reseting Camera ..." << endl;
	}
	//bool enablePIDZoom = true;
	return 1;
}


// Read first frame 
Mat frame;

//SIGN
int sign(float x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}

//DOT PRODUCT ANGLE
float angleBetween(const Point &v1, const Point &v2)
{
    float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    float dot = v1.x * v2.x + v1.y * v2.y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return CV_PI;//PI;
    else
        return acos(a); // 0..PI
}

//FLOW
void drawOptFlowMap (const Mat& flow, Mat& cflowmap, int step, double scale, const Scalar& color) {


	//cout << "pan rate = " << panRate << endl;
	//panRate = -5;

 for(int y = 0; y < cflowmap.rows; y += step/2){
        for(int x = 0; x < cflowmap.cols; x += step/2)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);

		//filter result
		
		//normalized flow field		
		float flowlength = sqrt(fxy.x*fxy.x + fxy.y*fxy.y);
		float normFlowX = fxy.x / flowlength;
		float normFlowY = fxy.y / flowlength;
		//normalized pan - titl
		float panTiltlength = sqrt(panRate*panRate + tiltRate*tiltRate);
		float normPanX = panRate / panTiltlength;
		float normTiltY = tiltRate / panTiltlength;

		//if(1==0 && sign(fxy.x) == sign(panRate) && sign(fxy.y) == sign(tiltRate) || (abs(fxy.x < 0.005) && abs(fxy.y < 0.005)) 
		//	||  ( abs(fxy.x < 0.001) &&  sign(fxy.y) == sign(tiltRate) )  
		//	||  ( abs(fxy.y < 0.001) &&  sign(fxy.x) == sign(panRate) ) 
		//	|| (abs(fxy.x < 0.001) && abs(fxy.y < 0.001)) || ((abs(normFlowX - normPanX) < 0.25) && (abs(normFlowY - normTiltY) < 0.2)) 						
		// ){//if(abs(fxy.x < 0.06) || abs(fxy.y) < 0.06){
		//if( (abs(fxy.x < 0.001) && abs(fxy.y < 0.001)) || ((abs(normFlowX - normPanX) < 0.25) && (abs(normFlowY - normTiltY) < 0.2)) ){
		//}else{

			//if( (abs(fxy.x < 0.008) && abs(fxy.y < 0.008)) ){
				//DO NOTHING			
			//}else{
				line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x*scale/3), cvRound(y+fxy.y*scale/3)), color);
				circle(cflowmap, Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);
				//}
			
				//PAN TILT ZOOM FIELD
				
//				line(cflowmap, Point(x,y), Point(cvRound(x+panRate*2), cvRound(y+tiltRate*2)), CV_RGB(255, 0, 0));

				//QUERRY AROUND FLOWS
				const Point2f& fxyU = flow.at<Point2f>(y, x);
				const Point2f& fxyD = flow.at<Point2f>(y, x);
				const Point2f& fxyL = flow.at<Point2f>(y, x);
				const Point2f& fxyR = flow.at<Point2f>(y, x);
				const Point2f& fxyUL = flow.at<Point2f>(y, x);
				const Point2f& fxyUR = flow.at<Point2f>(y, x);
				const Point2f& fxyDL = flow.at<Point2f>(y, x);
				const Point2f& fxyDR = flow.at<Point2f>(y, x);
				
				//FOR EACH PIXEL with same direcion around, increase vector length
				//! dot product computed in double-precision arithmetics
    				//double ddot(const Point_& pt) const;
				float angleThres = 15;
				float impactMeasure = 1;
				float impactFactor = 2;				
				float product;
				product = angleBetween(fxy, fxyU);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyD);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyL);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyR);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyUL);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyUR);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyDL);
				if(product < angleThres) { impactMeasure += impactFactor; }
				product = angleBetween(fxy, fxyDR);
				if(product < angleThres) { impactMeasure += impactFactor; }

				//REDUCE IMPACT IF SAME DIRECTION AS PAN - TILT
				const Point2f& fxyDIR = Point(cvRound(fxy.x), cvRound(fxy.y));
				const Point2f& flowDIR =  Point(cvRound(panRate), cvRound(tiltRate));
				product = angleBetween(fxyDIR, flowDIR);
				if(product < angleThres) { impactMeasure -= impactFactor * 6; }

				//COMBINE	
				if(impactMeasure > impactFactor * 1){
					const Point2f& fxyFINAL = Point(cvRound(x+impactMeasure*0.2*(panRate*3-fxy.x*scale/3)), cvRound(y+impactMeasure*0.2*(tiltRate*3-fxy.y*scale/3)));
					//line(cflowmap, Point(x,y),fxyFINAL, CV_RGB(255, 0, 0));
				}


				//circle(cflowmap, Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);
			//}
		//}
        }
    }
}
double DistanceXYtoBBoxCenter(Rect2d bbox, double centerX, double centerY){
	double bboxCenterX = bbox.x + bbox.width / 2;
	double bboxCenterY = bbox.y + bbox.height / 2;
	double centerXdiff = abs(bboxCenterX - centerX);
	double centerYdiff = abs(bboxCenterY - centerY);
	return sqrt(centerXdiff * centerXdiff + centerYdiff * centerYdiff);
}
double DistanceBBoxtoBBox(Rect2d bbox, Rect2d bbox1){
	double bboxCenterX = bbox.x + bbox.width / 2;
	double bboxCenterY = bbox.y + bbox.height / 2;
	double bboxCenterX1 = bbox1.x + bbox1.width / 2;
	double bboxCenterY1 = bbox1.y + bbox1.height / 2;
	double centerXdiff = abs(bboxCenterX - bboxCenterX1);
	double centerYdiff = abs(bboxCenterY - bboxCenterY1);
	return sqrt(centerXdiff * centerXdiff + centerYdiff * centerYdiff);
}
bool BBoxOutOfScreenBounds(Rect2d bbox){
	double bboxCenterX = bbox.x + bbox.width / 2;
	double bboxCenterY = bbox.y + bbox.height / 2;
	return (bboxCenterX < 0 || bboxCenterY < 0 || bboxCenterX > frame.cols || bboxCenterY > frame.rows);
}
void recenterTracker(Ptr<Tracker> &tracker, double centerX, double centerY, Rect2d &bbox){
	bbox.x = centerX - bbox.width / 2;
	bbox.y = centerY - bbox.height / 2;
	tracker = trackerUsed::create();
	tracker->init(frame, bbox);
	tracker->update(frame, bbox);
}
double DistancePointToPoint(double centerX, double centerY, double centerX1,double centerY1){
	double centerMedianXdiff = abs(centerX - centerX1);
	double centerMedianYdiff = abs(centerY - centerY1);
	return sqrt( centerMedianXdiff * centerMedianXdiff + centerMedianYdiff * centerMedianYdiff);
}
//END FLOW


Rect getTrackWindowAroundPoints(float scaleFactorLK1, Mat plotPoints){
				//DRAW RECTANGLE AROUND DRONE
				Rect trackFrameEstimate(-100, -100, 10, 10);//put out of frame to initialize
				Point trackFrameEstimateCenter;
				const bool useGpu = true;
				cv::ocl::setUseOpenCL(useGpu);
				cv::dilate(plotPoints.getUMat(ACCESS_RW), plotPoints.getUMat(ACCESS_RW), getStructuringElement(MORPH_ELLIPSE, Size(13, 13)));
				float scaling = 1; //scaleFactorLK1 
				vector<vector<Point> > contours;
        			vector<Vec4i> hierarchy;
				findContours(plotPoints, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));
				//Find contours
			 	vector<vector<Point> > contours_poly(contours.size());
				vector<Rect> boundRect(contours.size());
				////approximate contours by rectangles
				for (int i = 0; i < contours.size(); i++)
				{
				  approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
				  boundRect[i] = boundingRect(Mat(contours_poly[i]));
				}
				//draw bounded recatangles        
				//Mat fgMaskMOG2Plot;
				//plotPoints.copyTo(fgMaskMOG2Plot);
				int areaThreshold = 5;
				float prevArea = 0;
				int rectID = -1;
				//SEARCH FOR BIGGEST AREA RECTANGLE - EXTEND TO MULTIPLE TARGETS LATER
				if(contours.size() > 0 &&  contours.size() < 25){					
					for (int i = 0; i< contours.size(); i++)
					{	  
						//rectangle(fgMaskMOG2Plot, boundRect[i].tl()/scaling, boundRect[i].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);							 
						if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) > prevArea){
							rectID = i;				
							prevArea = contourArea(contours[i]);
						}		
					}
				}
				if(rectID >= 0){
					//rectangle(fgMaskMOG2Plot, boundRect[rectID].tl()/scaling, boundRect[rectID].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);
					//rectangle(plotFrame, boundRect[rectID].tl() * (1/scaleFactorLK1), boundRect[rectID].br() * (1/scaleFactorLK1), CV_RGB(255, 0, 255), 2, 8, 0);
					trackFrameEstimate.x = boundRect[rectID].x * (1/scaleFactorLK1);
					trackFrameEstimate.y = boundRect[rectID].y * (1/scaleFactorLK1);
					trackFrameEstimate.width = boundRect[rectID].width * (1/scaleFactorLK1);
					trackFrameEstimate.height = boundRect[rectID].height * (1/scaleFactorLK1);
					//trackFrameEstimate = boundRect[rectID]  * (1/scaleFactorLK1);
					trackFrameEstimateCenter.x = trackFrameEstimate.x + trackFrameEstimate.width/2;
					trackFrameEstimateCenter.y = trackFrameEstimate.y + trackFrameEstimate.height/2;
					//circle(plotFrame, trackFrameEstimateCenter, 15,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/1), -1);

					//keep memory previous
					//trackFrameEstimatePrev = trackFrameEstimate;
					//trackFrameEstimateCenterPrev = trackFrameEstimateCenter;

					//add to trajectory
					//trackFrameEstimateCenters.push_back(trackFrameEstimateCenter);
					//return trackFrameEstimate;
				}
				//else{
				//	return trackFrameEstimate;
				//}
				//	imshow("Ploted Points", fgMaskMOG2Plot);
				return trackFrameEstimate;
}//END getTrackWindowAroundPoints


/// BACKGROUND SUBTRACT
Mat plotFrame;
Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
Rect2d processVideo(Mat frame, float scaling, int filterPixels, int repeats) {
        //update the background model
	for (int i = 0; i < repeats; i++)
	{
		pMOG2->apply(frame, fgMaskMOG2);
	}

	//https://answers.opencv.org/question/66257/findcontours-no-working-correctly-in-background-subtraction/
	//morphological opening (remove small objects from the foreground)
	int filerSize = filterPixels;//3;
        //erode(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	medianBlur(fgMaskMOG2,fgMaskMOG2,3);
        dilate(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));

        //morphological closing (fill small holes in the foreground)
        dilate(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));	
	
	vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        //Detect edges using Threshold
        //threshold(fgMaskMOG2, fgMaskMOG2, 100, 255, THRESH_BINARY);
        //find contours
        //findContours(fgMaskMOG2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
 	findContours(fgMaskMOG2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));
 	//findContours(fgMaskMOG2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));

	//Find contours
 	vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());

        ////approximate contours by rectangles
        for (int i = 0; i < contours.size(); i++)
        {
          approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
          boundRect[i] = boundingRect(Mat(contours_poly[i]));
        }

        //draw bounded recatangles
        Mat drawing = Mat::zeros(plotFrame.size(), CV_8UC3);        

	int areaThreshold = 55;	
	int rectID = -1;
	if( contours.size() > 0 &&  contours.size() < 4){ //7
		//cout << "contours found: " << contours.size() << endl;
		for (int i = 0; i< contours.size(); i++)
		{	  
			//rectangle(plotFrame, boundRect[i].tl()/scaling, boundRect[i].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);
			////circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);		 
			if(contourArea(contours[i]) > areaThreshold){
				rectID = i;
				//boundRect[rectID] = boundingRect(contours[i]);
				//cout << "with area: " << contourArea(contours[i]) << endl;
				break;
			}		
		}
	}	

	//PLOT RECTANGLES
	if(1==1){
		Mat fgMaskMOG2Plot;
		fgMaskMOG2.copyTo(fgMaskMOG2Plot);
		//drawContours(fgMaskMOG2Plot, contours, -1,  CV_RGB(255, 0, 255), 10,8,hierarchy );

		if( contours.size() > 0){
			for (int i = 0; i< contours.size(); i++)
			{
				rectangle(fgMaskMOG2Plot, boundRect[i].tl(), boundRect[i].br(), CV_RGB(255, 0, 255), 2, 8, 0);
			}
		}
		//imshow("FG Mask MOG 2", fgMaskMOG2Plot);
	}

	if( contours.size() > 0 &&  contours.size() < 4 && rectID >= 0){ //7
		return boundRect[rectID];
	}else{
		Rect2d boundRect1;
		boundRect1.x=0;
		boundRect1.y=0;
		boundRect1.width = 0;
		boundRect1.height =0;
		return boundRect1;
	}       
}
///END BACKGROUND SUBTRACT


int countBbox2Same = 0;

vector<Rect2d> bboxesESTIMATED_STATIC; //estimated bboxes with background subtraction
vector<int> bboxESTIMATE_STATIC_frames; //frames of confidence for each found box candidate

//READ PTZ PAN - TILT - ZOOM
int frameCounter = 0;
void task1(){
	//v0.2
	prevPan = currentPan;
	prevTilt = currentTilt;	
	//READ PAN FROM CAMERA
	std::string getPanTiltPTZ = "81090612FF"; 
	udpSend(getPanTiltPTZ, true, false);	
}
void task2(){
	//v0.2
	prevZoom = currentZoom;	
	std::string getPanTiltPTZ = "81090447FF"; 
	udpSend(getPanTiltPTZ, true, true);	
}

//OPENCL TEST
static Mat flowToDisplay(const Mat flow)
{
    Mat flow_split[2];
    Mat magnitude, angle;
    Mat hsv_split[3], hsv, rgb;
    split(flow, flow_split);
    cartToPolar(flow_split[0], flow_split[1], magnitude, angle, true);

//exp(magnitude, magnitude);
//exp(magnitude, magnitude);
//exp(magnitude, magnitude);
//exp(magnitude, magnitude);
//threshold(magnitude, magnitude, 1.8, 1.8, 3);

    normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
    hsv_split[0] = angle; // already in degrees - no normalization needed
    hsv_split[1] = Mat::ones(angle.size(), angle.type());

	//threshold low values
	//exp(magnitude, magnitude);
	//threshold(magnitude, magnitude, 0.65, 0.65, 3);

    hsv_split[2] = magnitude;
    merge(hsv_split, 3, hsv);
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    return rgb;
}


//OCL FEATURE TRACKER
void UMatToVector(const UMat & um, std::vector<Point2f> & v) 
    {
        v.resize(um.size().area());
        um.copyTo(Mat(um.size(), CV_32FC2, &v[0]));
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

//HANDLE INPUT ASYNC
//TERMINAL INPUT
void task4()
{
		char inputTERMINAL;
       		cin >> inputTERMINAL;
		if(inputTERMINAL == 't' || inputTERMINAL == 'T') //(T)oggle PID - test sin
		{
			if(enablePIDracking){
				enablePIDracking = false;
				cout << "Disabled PID ..." << endl;
			}else{
				enablePIDracking = true;
				cout << "Enabled PID ..." << endl;
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
double velocityXGrav = 0;
double velocityYGrav = 0;
double velocityZGrav = 0;
Vec3f Eulers;
void task3(string msg)
{	
	cout << "thread start"  << endl;
	if(!useSparkfun){
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
	//	std::cout << "Eulers:" << Eulers[0] << ", " << Eulers[1] << ", " << Eulers[2] << std::endl;
		Eulers[0] = Eulers[0] * CV_PI / 180;
		Eulers[1] = Eulers[1] * CV_PI / 180;
		Eulers[2] = Eulers[2] * CV_PI / 180;
		Mat EulerRotationMatrix = eulerAnglesToRotationMatrix(Eulers);
	//	std::cout << "Euler Rotation Matrix:" << EulerRotationMatrix << std::endl;

		Vec3f GravityVec;
		GravityVec[0] = 0;
		GravityVec[1] = 0;
		GravityVec[2] = 0.981;

		GravityPerAxis =Mat(GravityVec).t() * EulerRotationMatrix ;//GravityPerAxis = EulerRotationMatrix * Mat(GravityVec);
		Vec3f GravityPerAxisV = (Vec3f)GravityPerAxis;
		float magnitude = sqrt(GravityPerAxisV[0]*GravityPerAxisV[0] + GravityPerAxisV[1]*GravityPerAxisV[1] + GravityPerAxisV[2]*GravityPerAxisV[2]);
	//	std::cout << "Gravity per axis:" << GravityPerAxis << std::endl;
	//	std::cout << "Gravity per axis magnitude:" << magnitude << std::endl;

		//std::cout << "______________" << endl;

		if(vectMEASUREMENTS.size() > 0){
			POZYXsamplesCounted++;
		}		
	} 	
}
 

///END SPARKFUN handler


//////// ROS
//cv::Mat srcImg;
//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{	 
// try
//  {	
//	cv::waitKey(5);
	//GRAB IMAGE from publisher node	
//	srcImg = cv_bridge::toCvShare(msg, "bgr8")->image;
//	arucoProcess(srcImg);
///  }
 // catch (cv_bridge::Exception& e)
//  {
 //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
 // }
//}
//END ROS

//FILE OUTPUT
std::ofstream outfile;


int main(int argc, char **argv)
{

//ROS
	ros::init(argc, argv, "arrayflowPublisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::UInt32>("arrayflow", 1);
//END ROS

	// List of tracker types in OpenCV 3.4.1
	string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
	
	// Create a tracker
	string trackerType = trackerTypes[6]; //2
 
	Ptr<Tracker> tracker;
	Ptr<Tracker> tracker2;	

	cout <<  CV_MINOR_VERSION << "," <<CV_MAJOR_VERSION << endl;

	#if (CV_MINOR_VERSION < 3)
	{
		tracker = Tracker::create(trackerType);
		tracker2 = Tracker::create(trackerType);
	}
	#else
	{	
		tracker = trackerUsed::create();//Ptr<Tracker> tracker;
		tracker2 = trackerUsed::create();
	}
	#endif   

	//GRAB PRERECORDED VIDEO    
	//VideoCapture video("videos/in1.avi");   
	//VideoCapture video("videos/V5.mp4"); 

	//GRAB REAL TIME FEED
	//inputVideo.open(camId);
	VideoCapture video(0);
   	video.set(3,1920);
  	video.set(4,1080);

	// Exit if video is not opened
	if(!video.isOpened())
	{
		cout << "Could not read video file" << endl; 
		return 1; 
	} 
 
	// Read first frame	
	bool ok = video.read(frame);  

	//READ WINDOW COORDINATES FROM TEXT FILE
	//ifstream inputfile("videos/in1a.log");
	ifstream inputfile("videos/in4.log");
	vector<double> pointsCSVx;
	vector<double> pointsCSVy;
	vector<double> pointsCSVw;
	vector<double> pointsCSVh;

	vector<int> cameraPan;
	vector<int> cameraTilt;
	vector<int> cameraZoom;

	string current_line;
	// vector allows you to add data without knowing the exact size beforehand
	vector< vector<int> > all_data;
	// Start reading lines as long as there are lines in the file
	while(getline(inputfile, current_line)){
	   // Now inside each line we need to seperate the cols
	   vector<int> values;
	   stringstream temp(current_line);
	   string single_value;
	   while(getline(temp,single_value,',')){
		// convert the string element to a integer value
		values.push_back(atoi(single_value.c_str()));
	   }
	   // add the row to the complete data vector
	   all_data.push_back(values);
	   //cout << "OUT=" << values[5] << endl; 5,6,7,8

	   cameraPan.push_back(values[2]);
	   cameraTilt.push_back(values[3]);
	   cameraZoom.push_back(values[4]);

  	   pointsCSVx.push_back(values[5]);
	   pointsCSVy.push_back(values[6]);
	   pointsCSVw.push_back(values[7]);
	   pointsCSVh.push_back(values[8]);
	}	
	//END READ WINDOW COORDINATES FROM TEXT FILE


	//Define initial bounding box    

	//in1.avi
	Rect2d bbox(350, 3, 200, 180);
	Rect2d bbox2(330, 2, 190, 185);
	Rect2d bbox2P(330, 2, 190, 185);
  	Rect2d bboxCSRT(350, 3, 200, 180);	
	
	tracker->init(frame, bbox);
  	tracker2->init(frame, bbox2);	
     
	Mat prevFrame;    
	Mat flow;
	Mat cflow;
	int counter = 0;

	int start_frame = 10;
    
	double flowImageScale = 0.3;

	//PARAMS
	//bool exportVideo = true;//false;
	bool previewMosse = false;//false;
	bool previewAssisted = false;
	bool previewFlows = false;

	bool plotFromCSV = false;
	bool updateCSRT = false;
	bool fullFlowMap = false;

	bool estimateBoundBoxfromFlowMap = true;

	bool useBackgroundSubtracion = false;
	bool stateChanged = true;
	int previousZoomFactor = 0;
	int previousPanFactor = 0;
	int previousTiltFactor = 0;

	//VIDEO WRITER
	bool exportSparseOpticalFlowVideos = true;//true false
	bool exportBackSubtractVideos = false;
	bool writeVideo = true;//false;//true;
	bool writeRAW = true;

	// Default resolution of the frame is obtained.The default resolution is system dependent. 
  	int frame_width = frame.cols;// cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  	int frame_height = frame.rows;//cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
   	
std::string fileNameVIDEO = "outVIDEO_" + GetCurrentTimeForFileName() + ".avi";
std::string fileNameVIDEO_FLOW = "outVIDEO_FLOW_" + GetCurrentTimeForFileName() + ".avi";

  	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  	//VideoWriter videoOut("outcpp.avi",CV_FOURCC('M','J','P','G'),30, Size(frame_width,frame_height)); 
	VideoWriter videoOut(fileNameVIDEO,CV_FOURCC('M','J','P','G'),30, Size(frame_width,frame_height));
	VideoWriter videoOutA;//("outcppBACKSUBRACT.avi",CV_FOURCC('M','J','P','G'),30, Size(frame_width,frame_height)); 
	VideoWriter videoOutB;
	//videoOutB.open("outcppBACKSUBRACT2.avi",CV_FOURCC('M','J','P','G'),30, Size(768,432),0);
	//("outcppBACKSUBRACT2.avi",CV_FOURCC('M','J','P','G'),30, Size(768,432),0); 
	VideoWriter videoOutC;//exportSparseOpticalFlowVideos
	//END VIDEO WRITER

	//BACKGROUND SUBTRACTION
	if(useBackgroundSubtracion){
		pMOG2 = createBackgroundSubtractorKNN(780,400.0,false);//createBackgroundSubtractorMOG2(); //MOG2 approach   
 		//processVideo(frame);
	}
	//END BACKGROUND SUBTRACTION

	//PTZ init
	int panFactorDiff = (cameraPan[frameCounter-1] - previousPanFactor) ;//* zoomConvFactor;
	int tiltFactorDiff = (cameraTilt[frameCounter-1] - previousTiltFactor) ;//* zoomConvFactor;


	int toggleThread = 0;
	///////////////////////////////////////////////// LOOP //////////////////////////////////////////


	///GPU INIT
	//GPU 0.///////////====================== 360 CAMERA VIEW HANDLER GPU (APPLY RECTIFICATION in GPU INIT REGION) ===================/////////////
	//sf::ContextSettings settings;
	//settings.depthBits = 24; settings.stencilBits = 8; settings.majorVersion = 3; settings.minorVersion = 2;
	float divider = 4; //sourceWidth = 5376/divider; sourceHeight = 2688/divider;
	sourceWidth = 1920; sourceHeight = 1080;
	
	//sf::Window window(sf::VideoMode(sourceWidth, sourceHeight, 32), "OpenGL", sf::Style::Titlebar | sf::Style::Close, settings);
	//initRectify360GPU(); 
	cout << "Init finished" << endl;
	// Load sample textures OR feed through OpenCV and ROS    
	cout << "Loaded sample image" << endl;
	//GPU 0.///////////====================== 360 CAMERA VIEW HANDLER GPU (END APPLY RECTIFICATION in GPU INIT REGION) ================/////////////
	Mat backgroundMaskGPU;
	//GLuint maskTex;
	bool initGPU = false;
	//END GPU INIT

	//float panRate = 0; float tiltRate = 0; float zoomRate = 0;



	//-------------------------------------------------------IMAGE STABILIZER rolling window INIT--------------------------------------------------- ///

	// For further analysis
	ofstream out_transform("prev_to_cur_transformation.txt");
	ofstream out_trajectory("trajectory.txt");
	ofstream out_smoothed_trajectory("smoothed_trajectory.txt");
	ofstream out_new_transform("new_prev_to_cur_transformation.txt");

	//VideoCapture cap(argv[1]);
	//assert(cap.isOpened());

//SPARKFUN SENSOR
	useSparkfun = true;
	if(useSparkfun){
		initComSparkFun();		
	}
	std::thread t3(task3, "SparkFun");
//END SPARKFUN SENSOR
	

///int scaleFactorLK = 2;
bool useFlowInKalmanSubWindow = true;
bool useMosseInKalmanSubWindow = true;//use MOSSE inside Kalman window, using inner optical flow to decide starting position !!!
bool updateKalmanByMOSSE = true;
Rect2d bboxKALMAN(1, 1, 200, 200);
Ptr<Tracker> trackerKALMAN;	
trackerKALMAN = trackerUsed::create();
//trackerKALMAN->init(frame, bboxKALMAN);

//enable Roll compensation by IMU
bool enableRollCompensation = false;
bool useExactRoll = true; //use the complete expression from 3 rotations matrices
//control pan tilt test periodic motion (not PID)
//bool enableTestPanTilt = true; //must disable enablePIDracking to use !!!!
//bool enableTestZoom = false; //must disable enablePIDracking to use !!!!
//bool enablePIDracking = false;
//bool enablePIDZoom = true;
float PID_P = 0.75f;//0.75f;

//std::thread t4(task4);

int maxFeaturesGlobalImage = 268;//150;
float maxBrightMOSSEThres = 12;//9.999;
float scaleFactorLK1 = 0.65;//0.5;//0.34;//0.72;//0.5; //SCALE MATCH POINT METHOD INPUT /0.55
float minDistanceToIgnore = 0.56;//0.16; //0.26 /0.24 //0.36 //0.46
float minNormalizedDistanceToIgnore = 0.0001;
bool plotAllPredictedFlowVectors = false;// true // false
Mat frameDownscaled1;
frame.copyTo(frameDownscaled1);
//resize(frameDownscaled1, frameDownscaled1, frame.size()/scaleFactorLK);
cv::resize(frameDownscaled1, frameDownscaled1, cv::Size(),scaleFactorLK1,scaleFactorLK1);

	//Mat cur, cur_grey;
	//Mat prev, prev_grey;
	Mat frame_grey, prevFrame_grey;
	Mat prev; frameDownscaled1.copyTo(prev);

	//cap >> prev;//get the first frame.ch
	cvtColor(prev, prevFrame_grey, COLOR_BGR2GRAY);
	cvtColor(frame, frame_grey, COLOR_BGR2GRAY);
	
	// Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
	std::vector <TransformParam> prev_to_cur_transform; // previous to current
	// Accumulated frame to frame transform
	double a = 0;
	double x = 0;
	double y = 0;
	// Step 2 - Accumulate the transformations to get the image trajectory
	std::vector <TrajectoryC> trajectory; // trajectory at all frames
	//
	// Step 3 - Smooth out the trajectory using an averaging window
	std::vector <TrajectoryC> smoothed_trajectory; // trajectory at all frames
	TrajectoryC X;//posteriori state estimate
	TrajectoryC X_;//priori estimate
	TrajectoryC P;// posteriori estimate error covariance
	TrajectoryC P_;// priori estimate error covariance
	TrajectoryC K;//gain
	TrajectoryC z;//actual measurement
	double pstd = 4e-3;//can be changed
	double cstd = 0.25;//can be changed
	TrajectoryC Q(pstd,pstd,pstd);// process noise covariance
	TrajectoryC R(cstd,cstd,cstd);// measurement noise covariance 
	// Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
	vector <TransformParam> new_prev_to_cur_transform;
	//
	// Step 5 - Apply the new transformation to the video
	//cap.set(CV_CAP_PROP_POS_FRAMES, 0);
	Mat T(2,3,CV_64F);

	int vert_border = HORIZONTAL_BORDER_CROP * sourceHeight / sourceWidth; // get the aspect ratio correct
	//VideoWriter outputVideo; 
	//outputVideo.open("compare.avi" , CV_FOURCC('X','V','I','D'), 24,cvSize(cur.rows, cur.cols*2+10), true);  
	//
	int k=1;
	//int max_frames = video.get(CV_CAP_PROP_FRAME_COUNT);
	Mat last_T;
	Mat prev_grey_,cur_grey_;
	//Mat cur2;
	Mat cur2_prev; prev.copyTo(cur2_prev);
	//-------------------------------------------------------IMAGE STABILIZER rolling window INIT END--------------------------------------------------- //

	int okBs=0; //count samples tracking is active
	double prevTime = (double)getTickCount();

	//FLOW MAPPING ON FEATURES METHOD
	Rect2d trackFrameEstimatePrev(-100, -100, 10, 10);//put out of frame to initialize
	Point trackFrameEstimateCenterPrev;
	vector<Point> trackFrameEstimateCenters;
	vector<Rect2d> trackFrameEstimateBoxes;
	Mat prev_Global_MOSSE_image; //keep previous mosse image to see similarity with new
	double maxScore;

	//////////////////////////////////////////////////  KALMAN FILTER INIT //////////////////////////////////////////////////
	// Camera frame
	//    cv::Mat frame;

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
	   // kf.measurementMatrix.at<float>(0) = 1.0f;
	   // kf.measurementMatrix.at<float>(7) = 1.0f;
	   // kf.measurementMatrix.at<float>(16) = 1.0f;
	   // kf.measurementMatrix.at<float>(23) = 1.0f;

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

	    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	    //kf.processNoiseCov.at<float>(0) = 1e-2;
	    //kf.processNoiseCov.at<float>(7) = 1e-2;
	    //kf.processNoiseCov.at<float>(14) = 5.0f;
	    //kf.processNoiseCov.at<float>(21) = 5.0f;
 	    //kf.processNoiseCov.at<float>(14) = 5.0f;
	    //kf.processNoiseCov.at<float>(21) = 5.0f;
	   // kf.processNoiseCov.at<float>(28) = 1e-2;
	    //kf.processNoiseCov.at<float>(35) = 1e-2;

  	    //kf.processNoiseCov.at<float>(0) = 4.0;//6
	    //kf.processNoiseCov.at<float>(7) = 4.0;
	    //kf.processNoiseCov.at<float>(14) = 18;//20.01f; //18
	   // kf.processNoiseCov.at<float>(21) = 18;//20.01f;
	   // kf.processNoiseCov.at<float>(28) = 4.0;
	    //kf.processNoiseCov.at<float>(35) = 4.0;

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
	    //kf.measurementNoiseCov.at<float>(0) = 2700;//12150;//1e-5;
	    //kf.measurementNoiseCov.at<float>(5) = 2700;//12150;
 	    //kf.measurementNoiseCov.at<float>(10) = 22500.0;//150*150
	   // kf.measurementNoiseCov.at<float>(15) = 22500.0;//tell the system how much difference to expect in measurement of track window dimensions, lower will follow precise track window but 
	    //follow also when gets oo large oulier

	    //ACCEL
	    kf.measurementNoiseCov.at<float>(0) = 2700;//12150;//1e-5;
	    kf.measurementNoiseCov.at<float>(7) = 2700;//12150;
	    kf.measurementNoiseCov.at<float>(14) = 270*270;//
	    kf.measurementNoiseCov.at<float>(21) = 270*270;//velocity cov
 	    kf.measurementNoiseCov.at<float>(28) = 22500.0;//150*150
	    kf.measurementNoiseCov.at<float>(35) = 22500.0;

	    // <<<< Kalman Filter

	    //// Camera Index
	    int idx = 0;

	    // Camera Capture
	//    cv::VideoCapture cap;
	    // >>>>> Camera Settings
	//    if (!cap.open(idx))
	//    {
	//	cout << "Webcam not connected.\n" << "Please verify\n";
	//	return EXIT_FAILURE;
	//    }
	//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
	//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
	    // <<<<< Camera Settings
	//    cout << "\nHit 'q' to exit...\n";

	    char ch = 0;
	    double ticks = 0;
	    bool found = false;
	    int notFoundCount = 0;
	    cv::Rect predRect;
	    cv::Point center;
	    vector<Rect> predRects;
	    vector<Rect> predRects_Kalman_Window;
	////////////////////////////////////////////////// END KALMAN INIT //////////////////////////////////////////////////

	//PTZ CAMERA BRIGHTNESS
	std::string homePTZA = "8101043E03FF";
	udpSend(homePTZA, false, false);
	std::string homePTZB = "8101043303FF";
	udpSend(homePTZB, false, false);



	//LOG FILE
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

			//WRITE HEADER
			outfile << "Video Frame ID" << ";"//outfile << "time" << ";" << "Video Frame ID" << ";"
				<< "MOSSE Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
				<< "Measure Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"				
				<< "Kalman Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
				<< "Inner FLOW Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
				<< "Inner MOSSE Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";" //relative to Kalman window above !!!
				<< "Euler Sensor X" << ";" << "Y" << ";" << "Z" << ";"
				<< "Pan" << ";" << "Tilt" << ";" << "Zoom" << ";"
				<< "YawRate" << ";" << "pitchRate" << ";" << "rollRate" << ";"
				<< "accelXRate" << ";" << "accelYRate" << ";" << "accelZRate" << ";"
				<< "panRate" << ";" << "tiltRate" << ";" << "MOSSE State" << ";"
				<< "Testing State" << ";"
				<< std::endl;			
	}
	//END LOG FILE
 	

//while(ros::ok() && video.read(frame) && kbhit() == 1)
while(ros::ok() && video.read(frame))
{

	
	if(writeVideo){
		if(writeRAW){
			videoOut.write(frame);
		}
	}

	//imshow("maxBrightMOSSE",frame);
	std::string homePTZC = "8101040D02FF";
	udpSend(homePTZC, false, false);

	// Start timer
	double timer = (double)getTickCount();

	frameCounter++;
	if(frameCounter > start_frame){	

		frame.copyTo(plotFrame); //COPY FOR PLOTING ON THIS FRAME

		


		

		bool okB = tracker2->update(frame, bbox2);
	     	
		//FLOW	
		
			
		

		//frame.copyTo(prevFrame);//prevFrame = frame; //generate previous frame for vector flow field
		counter++;

	     
		// Update the tracking result
		bool okA = okB;// tracker->update(frame, bbox);
	

		//MINE
		float v1 = rand() % 100; //window permutation test 1 - numbers 1 tp 100 (normalize - scale to image res later)
			
		
		//CHECK MOSSE
		if(maxBrightMOSSE > 0.96){//if(maxBrightMOSSE > 0.46){
			//okB = 0;
			//putText(plotFrame, "maxBrightMOSSE: " + SSTR(maxBrightMOSSE) , Point(bbox2.x,bbox2.y), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(50,170,50), 2);
		}
		


		////// GPU BACKGROUND SUBTRACT !!!!
		float panDiff = 0; float tiltDiff = 0; float zoomDiff = 0;
		float imageScaleBackgroundSUBTRACT = 0.4;

		if(1==1 && !frame.empty() && !prevFrame.empty()){


			//-------------------------------------------------------IMAGE STABILIZER rolling window CREATE--------------------------------------------------- //
			bool useStabilizer = false;
			Mat cur2;
			if(useStabilizer || 1==1){
				//cap >> cur;
				if(frame.data == NULL) {
					break;
				}

				//Decouple sparse point tracking and flowmap to main images
				Mat frameDownscaled;Mat prevFrameDownscaled;
				frame.copyTo(frameDownscaled);prevFrame.copyTo(prevFrameDownscaled);
				//resize(frameDownscaled, frameDownscaled, frame.size()/scaleFactorLK);
				///resize(prevFrameDownscaled, prevFrameDownscaled, frame.size()/scaleFactorLK);
				cv::resize(frameDownscaled, frameDownscaled, cv::Size(),scaleFactorLK1,scaleFactorLK1);
				cv::resize(prevFrameDownscaled, prevFrameDownscaled, cv::Size(),scaleFactorLK1,scaleFactorLK1);

				cvtColor(frameDownscaled, frame_grey, COLOR_BGR2GRAY);

				// vector from prev to cur
				vector <Point2f> prev_corner, cur_corner;
				vector <Point2f> prev_corner2, cur_corner2, ptzEstimatedCorners;
				vector <uchar> status;
				vector <float> err;

				cvtColor(prevFrameDownscaled, prevFrame_grey, COLOR_BGR2GRAY);
				//Scalar s = sum( frame_grey - prevFrame_grey );
				//bool equal = (s[0]==0) && (s[1]==0) && (s[2]==0);
				//if(equal){
				//	cout << "SAME GRAY !!!!!!!!!!!!" << endl;
				//}
				//imshow("grey1",frame_grey);imshow("grey2",prevFrame_grey);waitKey(11110);
			
				int cornersMax = maxFeaturesGlobalImage;//200;				

				//https://github.com/opencv/opencv/blob/master/modules/imgproc/test/ocl/test_gftt.cpp
				//std::vector<Point2f> upts, pts;
				//sudo intel_gpu_top
				const bool useGpuFeaturesTrack = true;
				if(useGpuFeaturesTrack){
					cv::ocl::setUseOpenCL(useGpuFeaturesTrack);	
					UMat points, upoints;
					//OCL_ON(cv::goodFeaturesToTrack(prevFrame_grey.getUMat(ACCESS_RW), upoints, cornersMax, 0.01, 30));//qualityLevel, minDistance));
					cv::goodFeaturesToTrack(prevFrame_grey.getUMat(ACCESS_RW), upoints, cornersMax, 0.01, 30);
					//ASSERT_FALSE(upoints.empty());
					UMatToVector(upoints, prev_corner);
					//ASSERT_EQ(upts.size(), pts.size());
				}else{
					goodFeaturesToTrack(prevFrame_grey, prev_corner, cornersMax, 0.01, 30);
				}			


				//(InputArray image, OutputArray corners, int maxCorners, double qualityLevel, double minDistance, InputArray mask=noArray(), int blockSize=3, bool useHarrisDetector=false, double k=0.04 )
				calcOpticalFlowPyrLK(prevFrame_grey, frame_grey, prev_corner, cur_corner, status, err);

				// weed out bad matches
				for(size_t i=0; i < status.size(); i++) {
					if(status[i]) {
						prev_corner2.push_back(prev_corner[i]);
						cur_corner2.push_back(cur_corner[i]);
					}
				}


				

				if(useSparkfun){
					YawRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (1+3)]);
					pitchRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (2+3)]);
					rollRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (3+3)]);

					accelXRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (6+3)]);
					accelYRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (5+3)]);
					accelZRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (4+3)]);
				}

				//PTZ FLOW ESTIMATOR -----
				//PTZ flow estimation - ptzEstimatedCorners
				double timeNow = (double)getTickCount();
				//(double)getTickCount() - timer, panRate
				double timeDiff = (timeNow - prevTime) * (1/getTickFrequency());//double timeDiff = ((double)getTickCount() - timer) * 0.001;//convert millisconds to seconds
				//cout << "timeDiff secs = " << timeDiff << "timeNow  = " << timeNow <<"prevTime  = " << prevTime <<endl;
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
				float maxZoom = 16384;
				float correctPanTiltByZoomF = currentZoom/maxZoom;
				float adjustedF = 0.00442 + correctPanTiltByZoomF * (0.0885-0.00442) * 0.1; //Zoom is 0 to 16384, resize F by the zoom factor

				//corrected with non linear curve
				correctPanTiltByZoomF = 1.06 + 0.12 * exp(currentZoom * 3.18 * pow(10,-4));// 3x zoom means focal lenght max / min = 3
				//adjustedF = correctPanTiltByZoomF * (0.00442/1.18) * 1;//0.00442;
				adjustedF = correctPanTiltByZoomF * (0.00442/1.18) * 1;//0.00442;
						

				for(size_t i=0; i < prev_corner2.size(); i++) {
					

					///add a component based on zoom, radially based on distance from screen center, further points are affected more by zoom
					float zoomComponentX = 0.3 * zoomDeltaRadians * (prev_corner2[i].x - frame_grey.cols/2); //0.0000015 for zoom speed 1, 0.0000008 for zoom speed 3
					float zoomComponentY = 0.3 * zoomDeltaRadians * (prev_corner2[i].y - frame_grey.rows/2);
										

					int esimateSimplificationLevel = 1;//5;
					if(useSparkfun){
 						esimateSimplificationLevel = 5;
					}

					if(esimateSimplificationLevel == 3){	
						//ptzEstimatedCorners.push_back(Point2f(	prev_corner2[i].x + 1 * adjustedF * panDeltaRadians  * 0.15 + 1 * zoomComponentX ,
						//					prev_corner2[i].y - 1 * adjustedF * tiltDeltaRadians * 0.15 + 1 * zoomComponentY ));
						//ptzEstimatedCorners.push_back(Point2f(	prev_corner2[i].x - 1 * adjustedF * (-panDeltaRadians) * 0.15 + 1 * zoomComponentX ,
						//					prev_corner2[i].y - 1 * adjustedF * (tiltDeltaRadians) * 0.15 + 1 * zoomComponentY ));
			
						float convertXtoMeters = (prev_corner2[i].x * 0.001 * 5.37) / frame_grey.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
						float convertYtoMeters = (prev_corner2[i].y * 0.001 * 4.04) / frame_grey.rows;
						convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
						convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
						float estX = convertXtoMeters - adjustedF * (-panDeltaRadians) * 1 * 0.4;  //0.4;
						float estY = convertYtoMeters + adjustedF * (-tiltDeltaRadians) * 1 * 0.5; //0.5;

						estX = estX + ((0.001 * 5.37) / 2);
						estY = estY + ((0.001 * 4.04) / 2);

						float convertestXtoPixels = estX * frame_grey.cols / (0.001 * 5.37);
						float convertestYtoPixels = estY * frame_grey.rows / (0.001 * 4.04);						
						cout << "convertXtoMeters = " << convertXtoMeters << " ,panDeltaRadians = " << panDeltaRadians << endl;
						cout << "convertYtoMeters = " << convertYtoMeters << " ,tiltDeltaRadians = " << tiltDeltaRadians << endl;
						cout << "adjustedF = " << adjustedF << " ,divider = " << divider << " ,correctPanTiltByZoomF = " << correctPanTiltByZoomF<< endl;
						ptzEstimatedCorners.push_back(Point2f(	convertestXtoPixels * 1 + 1 * zoomComponentX ,
											convertestYtoPixels * 1 + 1 * zoomComponentY ));
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
						Vec3f grav = (Vec3f)GravityPerAxis;
						double accelXGrav = (accelXRate+grav[0]);
						double accelYGrav = (accelYRate+grav[1]);
						double accelZGrav = (accelZRate-grav[2]);

						//update - integrate speed
						//double velocityXGrav = (accelXRate+grav[0]);
						//double velocityYGrav = (accelYRate+grav[1]);
						//double velocityZGrav = (accelZRate-grav[2]);
						velocityXGrav = velocityXGrav + accelXGrav * timeDiff;
						velocityYGrav = velocityYGrav + accelYGrav * timeDiff;
						velocityZGrav = velocityZGrav + accelZGrav * timeDiff;

						double accelXDelta = accelXGrav * timeDiff * timeDiff;
						double accelYDelta = accelYGrav * timeDiff * timeDiff;
						double accelZDelta = accelZGrav * timeDiff * timeDiff;//correct for G
						//accelXDelta = velocityXGrav * timeDiff;
						//accelYDelta = velocityYGrav * timeDiff;
						//accelZDelta = velocityZGrav * timeDiff;//correct for G

				// PLOT ACCEL DATA
				//		cout << "accel X:" << accelXGrav << endl;cout << "accel Y:" << accelYGrav << endl;cout << "accel Z:" << accelZGrav << endl;
				//		cout << "accelXRate X:" << accelXRate << endl;cout << "accelXRate Y:" << accelYRate << endl;cout << "accelXRate Z:" << accelZRate << endl;
				//		cout << "grav[0] X:" << grav[0] << endl;cout << "grav[0] Y:" << grav[1] << endl;cout << "grav[0] Z:" << grav[2] << endl;
						//double accelXDelta = (accelXRate-0.03) * timeDiff * timeDiff;
						//double accelYDelta = (accelYRate-0.04) * timeDiff * timeDiff;
						//double accelZDelta = (accelZRate-0.96) * timeDiff * timeDiff;//correct for G
						///double rollDeltaRadiansA = rollDeltaA * 3.14159265359f / 180.0f;

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
							estX = F*(sin(phi)*(sin(thi)*(cos(psi) + (x*sin(psi))/f) + (y*cos(thi))/f) - cos(phi)*(sin(psi) - (x*cos(psi))/f))/(cos(psi)*(cos(thi) - (y*sin(thi) - x*cos(thi)*sin(psi))/(f*cos(psi))));
							estY = F*((sin(phi)*(sin(psi) - (x*cos(psi))/f) + cos(phi)*sin(thi)*(cos(psi) + (x*sin(psi))/f))/cos(thi) + (y*cos(phi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);

							//WITHOUT ROLL TO TEST
							//FINAL_ROT2DDIV_X =
							//-(f*sin(psi) - x*cos(psi))/(f*cos(psi)*cos(thi) - y*sin(thi) + x*cos(thi)*sin(psi))
							//FINAL_ROT2DDIV_Y =
							//(cos(psi)*tan(thi) + (y + x*sin(psi)*tan(thi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f)
							//estX =-F*(f*sin(psi) - x*cos(psi))/(f*cos(psi)*cos(thi) - y*sin(thi) + x*cos(thi)*sin(psi));
							//estY =F*(cos(psi)*tan(thi) + (y + x*sin(psi)*tan(thi))/f)/(cos(psi) + (x*sin(psi) - y*tan(thi))/f);
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

				// translation + rotation only
				Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

				// in rare cases no transform is found. We'll just use the last known good transform.
				if(T.data == NULL) {
					last_T.copyTo(T);
				}

				T.copyTo(last_T);

				// decompose T
				double dx = T.at<double>(0,2);
				double dy = T.at<double>(1,2);
				double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
				//
				//prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

				out_transform << k << " " << dx << " " << dy << " " << da << endl;
				//
				// Accumulated frame to frame transform
				x += dx;
				y += dy;
				a += da;
				//trajectory.push_back(Trajectory(x,y,a));
				//
				out_trajectory << k << " " << x << " " << y << " " << a << endl;
				//
				z = TrajectoryC(x,y,a);
				//
				if(k==1){
					// intial guesses
					X = TrajectoryC(0,0,0); //Initial estimate,  set 0
					P =TrajectoryC(1,1,1); //set error variance,set 1
				}
				else
				{
					//time update prediction 
					X_ = X; //X_(k) = X(k-1);
					P_ = P+Q; //P_(k) = P(k-1)+Q;
					// measurement update correction 
					K = P_/( P_+R ); //gain;K(k) = P_(k)/( P_(k)+R );
					X = X_+K*(z-X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
					P = (TrajectoryC(1,1,1)-K)*P_; //P(k) = (1-K(k))*P_(k);
				}
				//smoothed_trajectory.push_back(X);
				out_smoothed_trajectory << k << " " << X.x << " " << X.y << " " << X.a << endl;
				//-
				// target - current
				double diff_x = X.x - x;//
				double diff_y = X.y - y;
				double diff_a = X.a - a;

				dx = dx + diff_x;
				dy = dy + diff_y;
				da = da + diff_a;

				//new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
				//
				out_new_transform << k << " " << dx << " " << dy << " " << da << endl;
				//
				T.at<double>(0,0) = cos(da);
				T.at<double>(0,1) = -sin(da);
				T.at<double>(1,0) = sin(da);
				T.at<double>(1,1) = cos(da);

				T.at<double>(0,2) = dx;
				T.at<double>(1,2) = dy;

				//Mat cur2;
		
				frameDownscaled.copyTo(cur2);//warpAffine(prev, cur2, T, frame.size());

				cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));

				// Resize cur2 back to cur size, for better side by side comparison
				resize(cur2, cur2, frameDownscaled.size());

				// Now draw the original and stablised side by side for coolness
				//	Mat canvas = Mat::zeros(frame.rows, frame.cols*2+10, frame.type());

				Mat canvas = Mat::zeros(cur2.rows, cur2.cols*2+10, frame.type());				

				prev.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
				cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));



				




				//PLOT CORNERS
				Mat plotPoints(cur2.rows, cur2.cols, CV_8UC1, Scalar(0,0,0));
				//cur2.copyTo(plotPoints);
				for(size_t i=0; i < cur_corner2.size(); i++) {
					circle(canvas, prev_corner2[i], 6,  CV_RGB(42/112, 2/112, 220/121), -1);
					
					float distEstimatedPTZtoFLOW = 0.075 * sqrt(pow((cur_corner2[i].x - ptzEstimatedCorners[i].x),2) + pow((cur_corner2[i].y - ptzEstimatedCorners[i].y),2));

					float normalizePTZ = sqrt(pow((ptzEstimatedCorners[i].x),2) + pow((ptzEstimatedCorners[i].y),2));
					float normalizeFLOW = sqrt(pow((cur_corner2[i].x),2) + pow((cur_corner2[i].y),2));
					float anglePTZtoFLOW = sqrt(pow(((cur_corner2[i].x/normalizeFLOW) - (ptzEstimatedCorners[i].x/normalizePTZ)),2) + pow(((cur_corner2[i].y/normalizeFLOW) - (ptzEstimatedCorners[i].y/normalizePTZ)),2));


					//PLOT ROLL PITCH YAW
					if(i==0){
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
						putText(canvas, "Sensor: " + SSTR(useSparkfun) + ", PT:"+ SSTR(enableTestPanTilt)+ ", Z:"+ SSTR(enableTestZoom)+ ", PID:"+ SSTR(enablePIDracking)+ ", PIDZ:"+ SSTR(enablePIDZoom), Point(10,185), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(120,111,250), 2);
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
							circle(canvas, Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), 9,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
							circle(canvas, Point(ptzEstimatedCorners[i].x + cur2.cols+10, ptzEstimatedCorners[i].y), 6, (CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/magnitude), -1);

							if(distEstimatedPTZtoFLOW > minDistanceToIgnore  && anglePTZtoFLOW > minNormalizedDistanceToIgnore){
								circle(plotPoints, Point(cur_corner2[i].x, cur_corner2[i].y), 35,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
							}

							///plot two flow esimates
							float directionX1 = 14 * (ptzEstimatedCorners[i].x-prev_corner2[i].x);
							float directionY1 = 14 * (ptzEstimatedCorners[i].y-prev_corner2[i].y);
							float magnitude1 = sqrt(pow(directionX1,2)+pow(directionY1,2));
							float directionX2 = 14 * (cur_corner2[i].x-prev_corner2[i].x);
							float directionY2 = 14 * (cur_corner2[i].y-prev_corner2[i].y);
							float magnitude2 = sqrt(pow(directionX2,2)+pow(directionY2,2));
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX1, prev_corner2[i].y + directionY1), 
							//CV_RGB(distEstimatedPTZtoFLOW * 100, distEstimatedPTZtoFLOW * 100, 0));
							//line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX2, prev_corner2[i].y + directionY2), 
							//CV_RGB(distEstimatedPTZtoFLOW * 200, distEstimatedPTZtoFLOW * 200, 220));
							line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX1, prev_corner2[i].y + directionY1), 
							CV_RGB(1 * 0, 1 * 250, 0));
							line(canvas, Point(prev_corner2[i].x + cur2.cols+10, prev_corner2[i].y), Point(prev_corner2[i].x + cur2.cols+10 + directionX2, prev_corner2[i].y + directionY2), 
							CV_RGB(250, 1 * 0, 0));
						}
					}
					//line(canvas, Point(prev_corner2[i].x, prev_corner2[i].y),Point(cur_corner2[i].x + cur2.cols+10, cur_corner2[i].y), CV_RGB(215, 215, 215) / distEstimatedPTZtoFLOW);
				}

				//imshow("Ploted Points", plotPoints);

				//getTrackWindowAroundPoints(float scaleFactorLK1, Mat plotFrame, Mat plotPoints)


				//DRAW RECTANGLE AROUND DRONE
				Rect2d trackFrameEstimate(-100, -100, 10, 10);//put out of frame to initialize
				Point trackFrameEstimateCenter;
				const bool useGpu = true;
				cv::ocl::setUseOpenCL(useGpu);
				cv::dilate(plotPoints.getUMat(ACCESS_RW), plotPoints.getUMat(ACCESS_RW), getStructuringElement(MORPH_ELLIPSE, Size(13, 13)));
				float scaling = 1; //scaleFactorLK1 
				vector<vector<Point> > contours;
        			vector<Vec4i> hierarchy;
				findContours(plotPoints, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));
				//Find contours
			 	vector<vector<Point> > contours_poly(contours.size());
				vector<Rect> boundRect(contours.size());
				////approximate contours by rectangles
				for (int i = 0; i < contours.size(); i++)
				{
				  approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
				  boundRect[i] = boundingRect(Mat(contours_poly[i]));
				}
				//draw bounded recatangles        
				Mat fgMaskMOG2Plot;
				plotPoints.copyTo(fgMaskMOG2Plot);
				int areaThreshold = 5;
				float prevArea = 0;
				int rectID = -1;
				//SEARCH FOR BIGGEST AREA RECTANGLE - EXTEND TO MULTIPLE TARGETS LATER
				if(contours.size() > 0 &&  contours.size() < 25){					
					for (int i = 0; i< contours.size(); i++)
					{	  
						//rectangle(fgMaskMOG2Plot, boundRect[i].tl()/scaling, boundRect[i].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);							 
						if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) > prevArea){
							rectID = i;				
							prevArea = contourArea(contours[i]);
						}		
					}
				}
				if(rectID >= 0){
					rectangle(fgMaskMOG2Plot, boundRect[rectID].tl()/scaling, boundRect[rectID].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);
					rectangle(plotFrame, boundRect[rectID].tl() * (1/scaleFactorLK1), boundRect[rectID].br() * (1/scaleFactorLK1), CV_RGB(255, 0, 255), 2, 8, 0);
					trackFrameEstimate.x = boundRect[rectID].x * (1/scaleFactorLK1);
					trackFrameEstimate.y = boundRect[rectID].y * (1/scaleFactorLK1);
					trackFrameEstimate.width = boundRect[rectID].width * (1/scaleFactorLK1);
					trackFrameEstimate.height = boundRect[rectID].height * (1/scaleFactorLK1);
					//trackFrameEstimate = boundRect[rectID]  * (1/scaleFactorLK1);
					trackFrameEstimateCenter.x = trackFrameEstimate.x + trackFrameEstimate.width/2;
					trackFrameEstimateCenter.y = trackFrameEstimate.y + trackFrameEstimate.height/2;
					circle(plotFrame, trackFrameEstimateCenter, 15,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/1), -1);

					//keep memory previous
					trackFrameEstimatePrev = trackFrameEstimate;
					trackFrameEstimateCenterPrev = trackFrameEstimateCenter;

					//add to trajectory
					trackFrameEstimateCenters.push_back(trackFrameEstimateCenter);
					trackFrameEstimateBoxes.push_back(trackFrameEstimate);
				}	
			//	imshow("Ploted Points", fgMaskMOG2Plot);





			////////////// 2...CROPPED OPTICAL FLOW
				Mat frame_greyCROP, prevFrame_greyCROP;
				if(1==0 && (1==1 || trackFrameEstimate.x == -100)){//if not found window
					trackFrameEstimate = trackFrameEstimatePrev;//use previous window to see if can track extra points
					trackFrameEstimateCenter = trackFrameEstimateCenterPrev;
					//}
					////2. USE FLOW ON TRACKED WINDOW(S), if not created use previous !!! (trackFrameEstimateCenterPrev and trackFrameEstimatePrev)
					//RUN FEATURE EXTRACTION IN SMALLER WINDOW !!!! but with full resolution
				
					if(trackFrameEstimate.width > 0 && trackFrameEstimate.height > 0 
						&& trackFrameEstimate.x > 0 && trackFrameEstimate.x < frame.cols 
						&& trackFrameEstimate.y > 0 && trackFrameEstimate.y < frame.rows 
						&& trackFrameEstimate.width < frame.cols / 3 && trackFrameEstimate.height < frame.rows / 3){
					//if(trackFrameEstimate.x + trackFrameEstimate.width/2 > 0 || trackFrameEstimate.y + trackFrameEstimate.height/2 < 0){ //FIX for track window outside main window case !!!
					
						Mat frameCROP;Mat prevFrameCROP;
						cv::Rect roi(trackFrameEstimate.x, trackFrameEstimate.y, trackFrameEstimate.width, trackFrameEstimate.height);
						//frameCROP = frame(cv::Range(bbox2.x, bbox2.width), cv::Range(bbox2.y, bbox2.height));
						//prevFrameCROP = prevFrame(cv::Range(bbox2.x, bbox2.width), cv::Range(bbox2.y, bbox2.height));
						frameCROP = frame(roi);
						prevFrameCROP = prevFrame(roi);

						cvtColor(frameCROP, frame_greyCROP, COLOR_BGR2GRAY);
						cvtColor(prevFrameCROP, prevFrame_greyCROP, COLOR_BGR2GRAY);

						//imshow("T11", frame_greyCROP);imshow("T22", prevFrame_greyCROP);

						vector <Point2f> prev_cornerCROP, cur_cornerCROP;
						vector <Point2f> prev_corner2CROP, cur_corner2CROP, ptzEstimatedCornersCROP;
						vector <uchar> statusCROP;
						vector <float> errCROP;
						int cornersMaxCROP = 50;
						if(useGpuFeaturesTrack){
							cv::ocl::setUseOpenCL(useGpuFeaturesTrack);	
							UMat pointsCROP, upointsCROP;
							//OCL_ON(cv::goodFeaturesToTrack(prevFrame_grey.getUMat(ACCESS_RW), upoints, cornersMax, 0.01, 30));//qualityLevel, minDistance));
							cv::goodFeaturesToTrack(prevFrame_greyCROP.getUMat(ACCESS_RW), upointsCROP, cornersMaxCROP, 0.01, 30);
							//ASSERT_FALSE(upoints.empty());
							UMatToVector(upointsCROP, prev_cornerCROP);
							//ASSERT_EQ(upts.size(), pts.size());
						}else{
							goodFeaturesToTrack(prevFrame_greyCROP, prev_cornerCROP, cornersMaxCROP, 0.01, 30);
						}
						calcOpticalFlowPyrLK(prevFrame_greyCROP, frame_greyCROP, prev_cornerCROP, cur_cornerCROP, statusCROP, errCROP);

						// weed out bad matches
						for(size_t i=0; i < statusCROP.size(); i++) {
							if(statusCROP[i]) {							
								prev_corner2CROP.push_back(prev_cornerCROP[i]);// + Point2f(trackFrameEstimate.x,trackFrameEstimate.y) );
								cur_corner2CROP.push_back(cur_cornerCROP[i]);// + Point2f(trackFrameEstimate.x,trackFrameEstimate.y) );
							}
						}

						for(size_t i=0; i < prev_corner2CROP.size(); i++) {
							///add a component based on zoom, radially based on distance from screen center, further points are affected more by zoom
							//float zoomComponentX = 0.3 * zoomDeltaRadians * (prev_corner2CROP[i].x - frame_greyCROP.cols/2); //
							//float zoomComponentY = 0.3 * zoomDeltaRadians * (prev_corner2CROP[i].y - frame_greyCROP.rows/2);

							//float zoomComponentX = 0.26 * zoomDeltaRadians * (trackFrameEstimate.x + prev_corner2CROP[i].x - frame_grey.cols/2); //
							//float zoomComponentY = 0.26 * zoomDeltaRadians * (trackFrameEstimate.y + prev_corner2CROP[i].y - frame_grey.rows/2);	
							float zoomComponentX = 0.3 * zoomDeltaRadians * ((trackFrameEstimate.x + prev_corner2CROP[i].x)*scaleFactorLK1 - frame_grey.cols/2); //
							float zoomComponentY = 0.3 * zoomDeltaRadians * ((trackFrameEstimate.y + prev_corner2CROP[i].y)*scaleFactorLK1 - frame_grey.rows/2);					

							int esimateSimplificationLevel = 1;//4;
							if(useSparkfun){
		 						esimateSimplificationLevel = 4;
							}						
							if(esimateSimplificationLevel == 1){
								float convertXtoMeters = (prev_corner2CROP[i].x * 0.001 * 5.37) / frame_greyCROP.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
								float convertYtoMeters = (prev_corner2CROP[i].y * 0.001 * 4.04) / frame_greyCROP.rows;
								convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
								convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
								float Xi = convertXtoMeters;
								float Yi = convertYtoMeters;
								float F = adjustedF * 1;
								//float Psi = -panDeltaRadians * 0.93; //PAN
								//float Thi = -tiltDeltaRadians * 0.94;//tiltDeltaRadians*1; ///TILT

								float Psi = -panDeltaRadians * 1.2; //PAN 1.2 for scale 0.6, 1.4 for 0.8 
								float Thi = -tiltDeltaRadians * 1.2;//tiltDeltaRadians*1; ///TILT

								float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
								float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
								float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
								float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	
						
								estX = estX + ((0.001 * 5.37) / 2);
								estY = estY + ((0.001 * 4.04) / 2);							
								float convertestXtoPixels = estX * frame_greyCROP.cols / (0.001 * 5.37);
								float convertestYtoPixels = estY * frame_greyCROP.rows / (0.001 * 4.04);					
								//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
								ptzEstimatedCornersCROP.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
							}
							//SPARKFUN SENSOR
							if(esimateSimplificationLevel == 4){	
						
								//roll = pozyxOUTPUT.size() - 3
								//pitch = pozyxOUTPUT.size() - 2
								//yaw = pozyxOUTPUT.size() - 1

								//acceleration X,Y,Z = pozyxOUTPUT.size() - 6,-5,-4
	
								//double YawRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (1+3)]) * 1;
								//double pitchRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (2+3)]) * 1;

								double panDeltaA = YawRate * timeDiff;
								double panDeltaRadiansA = panDeltaA * 3.14159265359f / 180.0f;

								double titlDeltaA = pitchRate * timeDiff;
								double tiltDeltaRadiansA = titlDeltaA * 3.14159265359f / 180.0f;

								float convertXtoMeters = (prev_corner2CROP[i].x * 0.001 * 5.37) / frame_greyCROP.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
								float convertYtoMeters = (prev_corner2CROP[i].y * 0.001 * 4.04) / frame_greyCROP.rows;
								convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
								convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
								float Xi = convertXtoMeters;
								float Yi = convertYtoMeters;
								float F = adjustedF * 1;
								//float Psi = -panDeltaRadians * 0.93; //PAN
								//float Thi = -tiltDeltaRadians * 0.94;//tiltDeltaRadians*1; ///TILT

								float Psi = -panDeltaRadiansA * 1.2; //PAN 1.2 for scale 0.6, 1.4 for 0.8 
								float Thi = -tiltDeltaRadiansA * 1.2;//tiltDeltaRadians*1; ///TILT

								float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
								float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
								float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
								float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	
						
								estX = estX + ((0.001 * 5.37) / 2);
								estY = estY + ((0.001 * 4.04) / 2);							
								float convertestXtoPixels = estX * frame_greyCROP.cols / (0.001 * 5.37);
								float convertestYtoPixels = estY * frame_greyCROP.rows / (0.001 * 4.04);					
								//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
								ptzEstimatedCornersCROP.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
							}
							//END SPARKFUN
						}

						Mat canvasCROP = Mat::zeros(frame_greyCROP.rows, frame_greyCROP.cols*2+10, frame.type());						
						prevFrameCROP.copyTo(canvasCROP(Range::all(), Range(0, frame_greyCROP.cols)));
						frameCROP.copyTo(canvasCROP(Range::all(), Range(frame_greyCROP.cols+10, frame_greyCROP.cols*2+10)));
						if(1==1){						
							for(size_t i=0; i < cur_corner2CROP.size(); i++) {
								circle(canvasCROP, prev_corner2CROP[i], 5,  CV_RGB(42/112, 2/112, 120/121), -1);					
								//circle(canvasCROP, Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 4,  CV_RGB(142, 2/112, 220/121), -1);
								//line(canvasCROP, Point(prev_corner2CROP[i].x, prev_corner2CROP[i].y),Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 
								//CV_RGB(255, 255, 0));

								float distEstimatedPTZtoFLOW = 0.075 * sqrt(pow((cur_corner2CROP[i].x - ptzEstimatedCornersCROP[i].x),2) 
								+ pow((cur_corner2CROP[i].y - ptzEstimatedCornersCROP[i].y),2));

								if(distEstimatedPTZtoFLOW > minDistanceToIgnore*2.5) //0.47) //
								{ 
									float directionX = 2 * (cur_corner2CROP[i].x - ptzEstimatedCornersCROP[i].x);
									float directionY = 2 * (cur_corner2CROP[i].y - ptzEstimatedCornersCROP[i].y);
									float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;
								
									circle(canvasCROP, Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 5,
									(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
									circle(canvasCROP, Point(ptzEstimatedCornersCROP[i].x + frame_greyCROP.cols+10, ptzEstimatedCornersCROP[i].y), 4, 
									(CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/magnitude), -1);

									//circle(plotPoints, Point(cur_corner2[i].x, cur_corner2[i].y), 35,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
									circle(plotPoints, Point(trackFrameEstimate.x + cur_corner2CROP[i].x, trackFrameEstimate.y + cur_corner2CROP[i].y)*scaleFactorLK1, 35,
									(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);

									///plot two flow esimates
									float directionX1 = 2 * (ptzEstimatedCornersCROP[i].x-prev_corner2CROP[i].x);
									float directionY1 = 2 * (ptzEstimatedCornersCROP[i].y-prev_corner2CROP[i].y);
									float magnitude1 = sqrt(pow(directionX1,2)+pow(directionY1,2));
									float directionX2 = 2 * (cur_corner2CROP[i].x-prev_corner2CROP[i].x);
									float directionY2 = 2 * (cur_corner2CROP[i].y-prev_corner2CROP[i].y);
									float magnitude2 = sqrt(pow(directionX2,2)+pow(directionY2,2));
								
									line(canvasCROP, Point(prev_corner2CROP[i].x + frame_greyCROP.cols+10, prev_corner2CROP[i].y), 
									Point(prev_corner2CROP[i].x + frame_greyCROP.cols+10 + directionX1, prev_corner2CROP[i].y + directionY1), CV_RGB(0, 250, 0));
									line(canvasCROP, Point(prev_corner2CROP[i].x + frame_greyCROP.cols+10, prev_corner2CROP[i].y), 
									Point(prev_corner2CROP[i].x + frame_greyCROP.cols+10 + directionX2, prev_corner2CROP[i].y + directionY2), CV_RGB(250, 0, 0));

									//line(canvasCROP, Point(prev_corner2CROP[i].x, prev_corner2CROP[i].y),Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 
									//CV_RGB(255, 255, 0));
								}

							}
							//cout << "cur_corner2CROP.size()" << cur_corner2CROP.size() << endl;
							//namedWindow( "Tracked points before and after CROPPED", WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
							//imshow("Tracked points before and after CROPPED", canvasCROP);
							

						}
					
					}//END CROPPED
						
					//imshow("Ploted Points CROPPED", plotPoints);					
	
				}//END CROPPED 2


				

		/////3...CROPPED OPTICAL FLOW in KALMAN WINDOW
				Rect trackFrameEstimateAB;
				Mat frame_greyCROPKAL, prevFrame_greyCROPKAL;				
				if(useFlowInKalmanSubWindow && predRects.size() > 1 && (1==1 || trackFrameEstimate.x == -100)){//if not found window
					//trackFrameEstimate = predRect;//use previous window to see if can track extra points
					//trackFrameEstimateCenter = center;
					//}
					////2. USE FLOW ON TRACKED WINDOW(S), if not created use previous !!! (trackFrameEstimateCenterPrev and trackFrameEstimatePrev)
					//RUN FEATURE EXTRACTION IN SMALLER WINDOW !!!! but with full resolution
					Rect trackFrameEstimateA = predRects[predRects.size()-1];//predRect; 
					Rect trackFrameEstimateB = predRects[predRects.size()-2]; 
						//Rect trackFrameEstimateAB;
						trackFrameEstimateAB.x = min(trackFrameEstimateA.x,trackFrameEstimateB.x);
						trackFrameEstimateAB.y = min(trackFrameEstimateA.y,trackFrameEstimateB.y);
						trackFrameEstimateAB.width =max(trackFrameEstimateA.width,trackFrameEstimateB.width);
						trackFrameEstimateAB.height =max(trackFrameEstimateA.height,trackFrameEstimateB.height);
					if(trackFrameEstimateAB.x < 0){
						trackFrameEstimateAB.x = 0; 		
					}
					if(trackFrameEstimateAB.y < 0){
						trackFrameEstimateAB.y = 0; 		
					}
					if(trackFrameEstimateAB.x + trackFrameEstimateAB.width >= frame.cols ){
						trackFrameEstimateAB.width = frame.cols - trackFrameEstimateAB.x -1; 		
					}
					if(trackFrameEstimateAB.y + trackFrameEstimateAB.height >= frame.rows ){
						trackFrameEstimateAB.height = frame.rows - trackFrameEstimateAB.y -1; 		
					}

					if(trackFrameEstimateB.x < 0){
						trackFrameEstimateB.x = 0; 		
					}
					if(trackFrameEstimateB.y < 0){
						trackFrameEstimateB.y = 0; 		
					}
					if(trackFrameEstimateB.x + trackFrameEstimateB.width >= frame.cols ){
						trackFrameEstimateB.width = frame.cols - trackFrameEstimateB.x -1; 		
					}
					if(trackFrameEstimateB.y + trackFrameEstimateB.height >= frame.rows ){
						trackFrameEstimateB.height = frame.rows - trackFrameEstimateB.y -1; 		
					}

					if(trackFrameEstimateAB.width > 0 && trackFrameEstimateAB.height > 0 
						&& trackFrameEstimateAB.x > 0 && trackFrameEstimateAB.x + trackFrameEstimateAB.width  < frame.cols 
						&& trackFrameEstimateAB.y > 0 && trackFrameEstimateAB.y + trackFrameEstimateAB.height < frame.rows 
						&& trackFrameEstimateAB.width < frame.cols / 2 && trackFrameEstimateAB.height < frame.rows / 2){
					//if(trackFrameEstimate.x + trackFrameEstimate.width/2 > 0 || trackFrameEstimate.y + trackFrameEstimate.height/2 < 0){ //FIX for track window outside main window case !!!
					
						Mat frameCROP;Mat prevFrameCROP;
						//cv::Rect roi(trackFrameEstimateA.x, trackFrameEstimateA.y, trackFrameEstimateA.width, trackFrameEstimateA.height);
						
						//use bounding box of the two windows
						
						cv::Rect roi(trackFrameEstimateAB.x, trackFrameEstimateAB.y, trackFrameEstimateAB.width, trackFrameEstimateAB.height);

						//frameCROP = frame(cv::Range(bbox2.x, bbox2.width), cv::Range(bbox2.y, bbox2.height));
						//prevFrameCROP = prevFrame(cv::Range(bbox2.x, bbox2.width), cv::Range(bbox2.y, bbox2.height));
						frameCROP = frame(roi);

						//cv::Rect roiPrev(trackFrameEstimateB.x, trackFrameEstimateB.y, trackFrameEstimateB.width, trackFrameEstimateB.height);
						prevFrameCROP = prevFrame(roi);

						//KALMAN ADD
						Mat plotPointsKALMAN(frameCROP.rows, frameCROP.cols, CV_8UC1, Scalar(0,0,0)); //PLOT POINTS TO IDENIFY RECTANGLE AROUND DOMINANT POINT CONCENTRATION

						cvtColor(frameCROP, frame_greyCROPKAL, COLOR_BGR2GRAY);
						cvtColor(prevFrameCROP, prevFrame_greyCROPKAL, COLOR_BGR2GRAY);

						//imshow("T11", frame_greyCROPKAL);imshow("T22", prevFrame_greyCROP);

						vector <Point2f> prev_cornerCROP, cur_cornerCROP;
						vector <Point2f> prev_corner2CROP, cur_corner2CROP, ptzEstimatedCornersCROP;
						vector <uchar> statusCROP;
						vector <float> errCROP;
						int cornersMaxCROP = 20;
						if(useGpuFeaturesTrack){
							cv::ocl::setUseOpenCL(useGpuFeaturesTrack);	
							UMat pointsCROP, upointsCROP;
							//OCL_ON(cv::goodFeaturesToTrack(prevFrame_grey.getUMat(ACCESS_RW), upoints, cornersMax, 0.01, 30));//qualityLevel, minDistance));
							cv::goodFeaturesToTrack(prevFrame_greyCROPKAL.getUMat(ACCESS_RW), upointsCROP, cornersMaxCROP, 0.01, 30);
							//ASSERT_FALSE(upoints.empty());
							UMatToVector(upointsCROP, prev_cornerCROP);
							//ASSERT_EQ(upts.size(), pts.size());
						}else{
							goodFeaturesToTrack(prevFrame_greyCROPKAL, prev_cornerCROP, cornersMaxCROP, 0.01, 30);
						}
						calcOpticalFlowPyrLK(prevFrame_greyCROPKAL, frame_greyCROPKAL, prev_cornerCROP, cur_cornerCROP, statusCROP, errCROP);

						// weed out bad matches
						for(size_t i=0; i < statusCROP.size(); i++) {
							if(statusCROP[i]) {							
								prev_corner2CROP.push_back(prev_cornerCROP[i]);// + Point2f(trackFrameEstimate.x,trackFrameEstimate.y) );
								cur_corner2CROP.push_back(cur_cornerCROP[i]);// + Point2f(trackFrameEstimate.x,trackFrameEstimate.y) );
							}
						}

						for(size_t i=0; i < prev_corner2CROP.size(); i++) {
							///add a component based on zoom, radially based on distance from screen center, further points are affected more by zoom
							//float zoomComponentX = 0.3 * zoomDeltaRadians * (prev_corner2CROP[i].x - frame_greyCROP.cols/2); //
							//float zoomComponentY = 0.3 * zoomDeltaRadians * (prev_corner2CROP[i].y - frame_greyCROP.rows/2);

							//float zoomComponentX = 0.26 * zoomDeltaRadians * (trackFrameEstimate.x + prev_corner2CROP[i].x - frame_grey.cols/2); //
							//float zoomComponentY = 0.26 * zoomDeltaRadians * (trackFrameEstimate.y + prev_corner2CROP[i].y - frame_grey.rows/2);	
							float zoomComponentX = 0.3 * zoomDeltaRadians * ((trackFrameEstimateAB.x + prev_corner2CROP[i].x)*scaleFactorLK1 - frame_grey.cols/2); //
							float zoomComponentY = 0.3 * zoomDeltaRadians * ((trackFrameEstimateAB.y + prev_corner2CROP[i].y)*scaleFactorLK1 - frame_grey.rows/2);					

							int esimateSimplificationLevel = 1;//4;
							if(useSparkfun){
		 						esimateSimplificationLevel = 4;
							}						
							if(esimateSimplificationLevel == 1){
								float convertXtoMeters = (prev_corner2CROP[i].x * 0.001 * 5.37) / frame_greyCROPKAL.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
								float convertYtoMeters = (prev_corner2CROP[i].y * 0.001 * 4.04) / frame_greyCROPKAL.rows;
								convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
								convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
								float Xi = convertXtoMeters;
								float Yi = convertYtoMeters;
								float F = adjustedF * 1;
								//float Psi = -panDeltaRadians * 0.93; //PAN
								//float Thi = -tiltDeltaRadians * 0.94;//tiltDeltaRadians*1; ///TILT

								float Psi = -panDeltaRadians * 1.2; //PAN 1.2 for scale 0.6, 1.4 for 0.8 
								float Thi = -tiltDeltaRadians * 1.2;//tiltDeltaRadians*1; ///TILT

								float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
								float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
								float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
								float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	
						
								estX = estX + ((0.001 * 5.37) / 2);
								estY = estY + ((0.001 * 4.04) / 2);							
								float convertestXtoPixels = estX * frame_greyCROPKAL.cols / (0.001 * 5.37);
								float convertestYtoPixels = estY * frame_greyCROPKAL.rows / (0.001 * 4.04);					
								//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
								ptzEstimatedCornersCROP.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
							}
							//SPARKFUN SENSOR
							if(esimateSimplificationLevel == 4){	
						
								//roll = pozyxOUTPUT.size() - 3
								//pitch = pozyxOUTPUT.size() - 2
								//yaw = pozyxOUTPUT.size() - 1

								//acceleration X,Y,Z = pozyxOUTPUT.size() - 6,-5,-4
	
								//double YawRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (1+3)]) * 1;
								//double pitchRate = (pozyxOUTPUT[pozyxOUTPUT.size() - (2+3)]) * 1;

								double panDeltaA = YawRate * timeDiff;
								double panDeltaRadiansA = panDeltaA * 3.14159265359f / 180.0f;

								double titlDeltaA = pitchRate * timeDiff;
								double tiltDeltaRadiansA = titlDeltaA * 3.14159265359f / 180.0f;

								float convertXtoMeters = (prev_corner2CROP[i].x * 0.001 * 5.37) / frame_greyCROPKAL.cols; //9.4 - 6.72 diagonal image_sensors_format wikipedia
								float convertYtoMeters = (prev_corner2CROP[i].y * 0.001 * 4.04) / frame_greyCROPKAL.rows;
								convertXtoMeters = convertXtoMeters - ((0.001 * 5.37) / 2);
								convertYtoMeters = convertYtoMeters - ((0.001 * 4.04) / 2);
								float Xi = convertXtoMeters;
								float Yi = convertYtoMeters;
								float F = adjustedF * 1;
								//float Psi = -panDeltaRadians * 0.93; //PAN
								//float Thi = -tiltDeltaRadians * 0.94;//tiltDeltaRadians*1; ///TILT

								float Psi = -panDeltaRadiansA * 2; //PAN 1.2 for scale 0.6, 1.4 for 0.8 
								float Thi = -tiltDeltaRadiansA * 2;//tiltDeltaRadians*1; ///TILT

								float divider1 = ((Xi * tan(Psi)*cos(Thi)) - (Yi * (sin(Thi)/cos(Psi))) + (F * cos(Thi)));
								float divider2 = (Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));//divider1;//(Xi*sin(Psi) - Yi * tan(Thi) + F*cos(Psi));
								float estX = adjustedF * ((Xi - (F * tan(Psi))) / divider1);
								float estY = adjustedF * ((Yi + (F * tan(Thi)*cos(Psi)) + (Xi * sin(Psi)*tan(Thi))) / divider2);	
						
								estX = estX + ((0.001 * 5.37) / 2);
								estY = estY + ((0.001 * 4.04) / 2);							
								float convertestXtoPixels = estX * frame_greyCROPKAL.cols / (0.001 * 5.37);
								float convertestYtoPixels = estY * frame_greyCROPKAL.rows / (0.001 * 4.04);					
								//ptzEstimatedCorners.push_back(Point2f(estX+ 1 * zoomComponentX, estY+ 1 * zoomComponentY));
								ptzEstimatedCornersCROP.push_back(Point2f(convertestXtoPixels+ 1 * zoomComponentX, convertestYtoPixels+ 1 * zoomComponentY));
							}
							//END SPARKFUN
						}

						Mat canvasCROP = Mat::zeros(frame_greyCROPKAL.rows, frame_greyCROPKAL.cols*2+10, frame.type());						
						prevFrameCROP.copyTo(canvasCROP(Range::all(), Range(0, frame_greyCROPKAL.cols)));
						frameCROP.copyTo(canvasCROP(Range::all(), Range(frame_greyCROPKAL.cols+10, frame_greyCROPKAL.cols*2+10)));
						if(1==1){						
							for(size_t i=0; i < cur_corner2CROP.size(); i++) {
								circle(canvasCROP, prev_corner2CROP[i], 5,  CV_RGB(42/112, 2/112, 120/121), -1);					
								//circle(canvasCROP, Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 4,  CV_RGB(142, 2/112, 220/121), -1);
								//line(canvasCROP, Point(prev_corner2CROP[i].x, prev_corner2CROP[i].y),Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 
								//CV_RGB(255, 255, 0));

								float distEstimatedPTZtoFLOW = 0.075 * sqrt(pow((cur_corner2CROP[i].x - ptzEstimatedCornersCROP[i].x),2) 
								+ pow((cur_corner2CROP[i].y - ptzEstimatedCornersCROP[i].y),2));

								if(distEstimatedPTZtoFLOW > minDistanceToIgnore*1.5) //0.47) //bigger allows less candidates //2.5
								{ 
									float directionX = 2 * (cur_corner2CROP[i].x - ptzEstimatedCornersCROP[i].x);
									float directionY = 2 * (cur_corner2CROP[i].y - ptzEstimatedCornersCROP[i].y);
									float magnitude = sqrt(pow(directionX,2)+pow(directionY,2)) * 0.1;
								
									circle(canvasCROP, Point(cur_corner2CROP[i].x + frame_greyCROPKAL.cols+10, cur_corner2CROP[i].y), 5,
									(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
									circle(canvasCROP, Point(ptzEstimatedCornersCROP[i].x + frame_greyCROPKAL.cols+10, ptzEstimatedCornersCROP[i].y), 4, 
									(CV_RGB(255, 255, 255) -  CV_RGB(1, 212, 220)/magnitude), -1);

									//circle(plotPoints, Point(cur_corner2[i].x, cur_corner2[i].y), 35,(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);
									circle(plotPointsKALMAN, Point(cur_corner2CROP[i].x,cur_corner2CROP[i].y)*1, 30,
									(CV_RGB(255, 255, 255) - CV_RGB(242, 2/112, 220/121)/magnitude), -1);

									///plot two flow esimates
									float directionX1 = 2 * (ptzEstimatedCornersCROP[i].x-prev_corner2CROP[i].x);
									float directionY1 = 2 * (ptzEstimatedCornersCROP[i].y-prev_corner2CROP[i].y);
									float magnitude1 = sqrt(pow(directionX1,2)+pow(directionY1,2));
									float directionX2 = 2 * (cur_corner2CROP[i].x-prev_corner2CROP[i].x);
									float directionY2 = 2 * (cur_corner2CROP[i].y-prev_corner2CROP[i].y);
									float magnitude2 = sqrt(pow(directionX2,2)+pow(directionY2,2));
								
									line(canvasCROP, Point(prev_corner2CROP[i].x + frame_greyCROPKAL.cols+10, prev_corner2CROP[i].y), 
									Point(prev_corner2CROP[i].x + frame_greyCROPKAL.cols+10 + directionX1, prev_corner2CROP[i].y + directionY1), CV_RGB(0, 250, 0));
									line(canvasCROP, Point(prev_corner2CROP[i].x + frame_greyCROPKAL.cols+10, prev_corner2CROP[i].y), 
									Point(prev_corner2CROP[i].x + frame_greyCROPKAL.cols+10 + directionX2, prev_corner2CROP[i].y + directionY2), CV_RGB(250, 0, 0));

									//line(canvasCROP, Point(prev_corner2CROP[i].x, prev_corner2CROP[i].y),Point(cur_corner2CROP[i].x + frame_greyCROP.cols+10, cur_corner2CROP[i].y), 
									//CV_RGB(255, 255, 0));
								}
							}
							//cout << "cur_corner2CROP.size()" << cur_corner2CROP.size() << endl;
							//namedWindow( "Tracked points before and after CROPPED KALMAN", WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
							//imshow("Tracked points before and after CROPPED KALMAN", canvasCROP);								
						}

						//FIND BOUNDING BOX AROUND INSIDE KALMAN POINTS
						Rect trackFrameEstimateKALMAN = getTrackWindowAroundPoints(1, plotPointsKALMAN);//no scaling
						Point trackFrameEstimateCenter = Point(trackFrameEstimateKALMAN.x + trackFrameEstimateKALMAN.width, trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height);
						//getTrackWindowAroundPoints(float scaleFactorLK1, Mat plotPoints)
						//DRAW RECTANGLE AROUND DRONE
						//Rect2d trackFrameEstimate(-100, -100, 10, 10);//put out of frame to initialize
						rectangle(plotPointsKALMAN, trackFrameEstimateKALMAN.tl() * (1/1), trackFrameEstimateKALMAN.br() * (1/1), CV_RGB(255, 0, 255), 2, 8, 0);
						//plot in full window
						Rect trackFrameEstimateKALMAN_WORLD;
						trackFrameEstimateKALMAN_WORLD.x = trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x;//-trackFrameEstimateKALMAN.width/1;
						trackFrameEstimateKALMAN_WORLD.y = trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y;//- trackFrameEstimateKALMAN.height/1;
						trackFrameEstimateKALMAN_WORLD.width = trackFrameEstimateKALMAN.width *1;
						trackFrameEstimateKALMAN_WORLD.height = trackFrameEstimateKALMAN.height *1;

						trackFrameEstimateKALMAN_WORLD.tl() = Point(trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x,trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y);
						trackFrameEstimateKALMAN_WORLD.br() = Point(trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x + trackFrameEstimateKALMAN.width,	trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height);

						rectangle(plotFrame, 
						Point(trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x,trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y), 
						Point(trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x + trackFrameEstimateKALMAN.width,
						trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height), CV_RGB(0, 110, 255), 2, 8, 0);

						//rectangle(plotFrame, trackFrameEstimateKALMAN_WORLD.tl() * (1/1), trackFrameEstimateKALMAN_WORLD.br() * (1/1), CV_RGB(0, 210, 255), 2, 8, 0);

						//rectangle(plotFrame,
						//Point(trackFrameEstimateKALMAN_WORLD.x,trackFrameEstimateKALMAN_WORLD.y), 
						//Point(trackFrameEstimateKALMAN_WORLD.x + trackFrameEstimateKALMAN_WORLD.width,
						//trackFrameEstimateKALMAN_WORLD.y+trackFrameEstimateKALMAN_WORLD.height), CV_RGB(255, 110, 255), 2, 8, 0);

						//KEEP window found inside Kalman window in world coordinates to pass to Kalman predictor
						predRects_Kalman_Window.push_back(trackFrameEstimateKALMAN_WORLD);	


						/////// MOSSE
						//MOSSE IN KALMAN SUBWINDOW
						if(useMosseInKalmanSubWindow){
							
							//trackerKALMAN->init(frameCROP, bboxKALMAN);
							bool okKALMAN = trackerKALMAN->update(frameCROP, bboxKALMAN);
							Mat plotSUBKalman;
							frameCROP.copyTo(plotSUBKalman);
							if(!okKALMAN 

 								|| (bboxKALMAN.x + bboxKALMAN.width/2) > frameCROP.cols + bboxKALMAN.width/4 || (bboxKALMAN.y + bboxKALMAN.height/2) > frameCROP.rows + bboxKALMAN.height/4
								|| (bboxKALMAN.x + bboxKALMAN.width/2) < -bboxKALMAN.width/4 || (bboxKALMAN.y + bboxKALMAN.height/2) < -bboxKALMAN.height/4

							){
								//bboxKALMAN.x = 0;
								//bboxKALMAN.y = 0;
								//bboxKALMAN.width = 33;
								//bboxKALMAN.height = 33;
								bboxKALMAN.tl() = trackFrameEstimateKALMAN.tl();//bboxKALMAN.tl() = trackFrameEstimateKALMAN_WORLD.tl();
								bboxKALMAN.br() = trackFrameEstimateKALMAN.br();//bboxKALMAN.br() = trackFrameEstimateKALMAN_WORLD.br();
								bboxKALMAN.x = trackFrameEstimateKALMAN.x;
								bboxKALMAN.y = trackFrameEstimateKALMAN.y;
								bboxKALMAN.width = trackFrameEstimateKALMAN.width;
								bboxKALMAN.height = trackFrameEstimateKALMAN.height;
								trackerKALMAN = trackerUsed::create();
								trackerKALMAN->init(frameCROP, bboxKALMAN);
								okKALMAN = trackerKALMAN->update(frameCROP, bboxKALMAN);

								
							}

							//update global mosse
							float mosseLocalGlobalDist = sqrt(pow(trackFrameEstimateAB.x + bboxKALMAN.x + bboxKALMAN.width/2 - (bbox2.x + bbox2.width),2)
										     +pow(trackFrameEstimateAB.y + bboxKALMAN.y + bboxKALMAN.height/2 - (bbox2.y + bbox2.height),2));
							Rect2d bboxTEMP (trackFrameEstimateAB.x+bboxKALMAN.x,trackFrameEstimateAB.y+bboxKALMAN.y, bboxKALMAN.width,bboxKALMAN.height );
							
							//update global mosse by kalman RUN ON DEMAND if tracking lost is sensed
							if(1==0){								
								//if Kalman window far from previous, initialize MOSSE and move to tracking lost state 								
								if(mosseLocalGlobalDist > 1
								 ){ 
									bbox2.x = bboxTEMP.x;
									bbox2.y = bboxTEMP.y;
									bbox2.width = bboxTEMP.width;
									bbox2.height = bboxTEMP.height;
									tracker2 = trackerUsed::create();
									tracker2 -> init(frame, bbox2);
									okB = tracker2->update(frame, bbox2);
								}else{	
									//update only when sensed Kalman issue
									okB = tracker2->update(frame, bbox2);
								}
							}

							//update global mosse by kalman ALWAYS RUN
							if(1==1){
								//bbox2.tl() = trackFrameEstimateKALMAN.tl();//bboxKALMAN.tl() = trackFrameEstimateKALMAN_WORLD.tl();
								//bbox2.br() = trackFrameEstimateKALMAN.br();//bboxKALMAN.br() = trackFrameEstimateKALMAN_WORLD.br();								
								//if(maxScore > 0.986 || okB == 0 || (okKALMAN && (maxBrightMOSSE > maxBrightMOSSEThres) )
								//if( maxScore < 0.004 || okB == 0 || (okKALMAN && (maxBrightMOSSE > maxBrightMOSSEThres) )
								if( maxScore < 0.008 || okB == 0 || (okKALMAN && (maxBrightMOSSE > maxBrightMOSSEThres) )
									|| (bbox2.x + bbox2.width/2 > frame.cols || bbox2.y + bbox2.height/2 > frame.rows || bbox2.x + bbox2.width/2 < 0 || bbox2.y + bbox2.height/2 < 0)
								 ){ // || maxBrightMOSSE > maxBrightMOSSEThres || mosseLocalGlobalDist > 250){
									//bbox2.x = trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x;
									//bbox2.y = trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y;
									//bbox2.width = trackFrameEstimateKALMAN.width;
									//bbox2.height = trackFrameEstimateKALMAN.height;
									bbox2.x = bboxTEMP.x;
									bbox2.y = bboxTEMP.y;
									bbox2.width = bboxTEMP.width;
									bbox2.height = bboxTEMP.height;
									tracker2 = trackerUsed::create();
									tracker2 -> init(frame, bbox2);
									okB = tracker2->update(frame, bbox2);
								}else{
									//okB = tracker2->update(frame, bbox2);	
	
									//update only when sensed Kalman issue
									okB = tracker2->update(frame, bbox2);
								}
							}
							//update global mosse by kalman OLD1
							if(1==0){
								//bbox2.tl() = trackFrameEstimateKALMAN.tl();//bboxKALMAN.tl() = trackFrameEstimateKALMAN_WORLD.tl();
								//bbox2.br() = trackFrameEstimateKALMAN.br();//bboxKALMAN.br() = trackFrameEstimateKALMAN_WORLD.br();
								float mosseLocalGlobalDist = sqrt(pow(trackFrameEstimateAB.x + trackFrameEstimateKALMAN.x + trackFrameEstimateKALMAN.width/2 - (bbox2.x + bbox2.width),2)
												 +pow(trackFrameEstimateAB.y + trackFrameEstimateKALMAN.y + trackFrameEstimateKALMAN.height/2 - (bbox2.y + bbox2.height),2));
								Rect2d bboxTEMP (trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x,trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y, trackFrameEstimateKALMAN.width,trackFrameEstimateKALMAN.height );
								if(1==1 || okB == 0 || (okKALMAN && (maxBrightMOSSE > maxBrightMOSSEThres) )
									|| (bboxTEMP.x + bboxTEMP.width/2 > frame.cols || bboxTEMP.y + bboxTEMP.height/2 > frame.rows || bboxTEMP.x + bboxTEMP.width/2 < 0 || bboxTEMP.y + bboxTEMP.height/2 < 0)
								 ){ // || maxBrightMOSSE > maxBrightMOSSEThres || mosseLocalGlobalDist > 250){
									//bbox2.x = trackFrameEstimateAB.x+trackFrameEstimateKALMAN.x;
									//bbox2.y = trackFrameEstimateAB.y+trackFrameEstimateKALMAN.y;
									//bbox2.width = trackFrameEstimateKALMAN.width;
									//bbox2.height = trackFrameEstimateKALMAN.height;
									bbox2.x = bboxTEMP.x;
									bbox2.y = bboxTEMP.y;
									bbox2.width = bboxTEMP.width;
									bbox2.height = bboxTEMP.height;
									tracker2 = trackerUsed::create();
									tracker2 -> init(frame, bbox2);
									okB = tracker2->update(frame, bbox2);
								}else{
									okB = tracker2->update(frame, bbox2);
								}
							}
							//similariy metric inside MOSSE window
							//https://docs.opencv.org/2.4/modules/imgproc/doc/object_detection.html?highlight=matchtemplate
							Mat scoreImg;
							//double maxScore;
							if(bbox2.x >0 && bbox2.width > 0 && bbox2.y > 0 && bbox2.height > 0 && bbox2.x + bbox2.width <= frame.cols &&  bbox2.y + bbox2.height <= frame.rows){
								cv::Rect roi(bbox2.x, bbox2.y, bbox2.width, bbox2.height);							
								Mat current_Global_MOSSE_image = frame(roi);
								if(current_Global_MOSSE_image.rows == prev_Global_MOSSE_image.rows && current_Global_MOSSE_image.cols == prev_Global_MOSSE_image.cols){
									matchTemplate(prev_Global_MOSSE_image, current_Global_MOSSE_image, scoreImg, CV_TM_SQDIFF_NORMED);
									//CV_TM_SQDIFF);//CV_TM_CCOEFF);//CV_TM_CCORR_NORMED);//TM_CCOEFF_NORMED);								
									minMaxLoc(scoreImg, 0, &maxScore);
									//cout << "MOSSE SIMILARITY SCORE:" << maxScore << endl;
								}
								if(!prev_Global_MOSSE_image.empty()){
									//imshow("previous MOSSE small", prev_Global_MOSSE_image);
								}
								//imshow("currrent MOSSE small", current_Global_MOSSE_image);
								//prev_Global_MOSSE_image = current_Global_MOSSE_image; //load current to previous for later use
								current_Global_MOSSE_image.copyTo(prev_Global_MOSSE_image);
							}
							//END similariy metric inside MOSSE window
							
							putText(plotSUBKalman, "MOSSE STATUS: " + SSTR(okKALMAN) , Point(10,80), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(50,170,50), 2);
							rectangle(plotSUBKalman, bboxKALMAN, Scalar( 255, 255, 255 ), 2, 1 );
							rectangle(plotSUBKalman, trackFrameEstimateKALMAN, Scalar( 55, 55, 255 ), 2, 1 );
							//imshow("Kalman inside", plotSUBKalman);

							//rectangle(plotFrame, bboxKALMAN, Scalar( 255, 255, 255 ), 2, 1 );
							rectangle(plotFrame, 
							Point(trackFrameEstimateAB.x+bboxKALMAN.x,trackFrameEstimateAB.y+bboxKALMAN.y), 
							Point(trackFrameEstimateAB.x+bboxKALMAN.x + bboxKALMAN.width,
							trackFrameEstimateAB.y+bboxKALMAN.y + bboxKALMAN.height), CV_RGB(0, 10, 255), 2, 8, 0);
							//rectangle(plotPointsKALMAN, bboxKALMAN, Scalar( 255, 255, 255 ), 2, 1 );

						}
						////// END MOSSE
				
			
						//trackFrameEstimateAB.x
						//imshow("Ploted Points CROPPED KALMAN", plotPointsKALMAN);					
					}//END CROPPED						
					//imshow("Ploted Points CROPPED KALMAN", plotPointsKALMAN);	
				}////////////////// 	END CROPPED 3


////////////////////////////// KALMAN FILTER ////////////////////////////////////
		//https://github.com/Myzhar/simple-opencv-kalman-tracker/blob/master/source/opencv-kalman.cpp
		// >>>>> Main loop
		if(1==1)//while (ch != 'q' && ch != 'Q')
		{
			double precTick = ticks;
			ticks = (double) cv::getTickCount();

			double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

			// Frame acquisition
		//	cap >> frame;

		//	cv::Mat res;
		//	frame.copyTo(res);



			//USE SUB WINDOW
			bool useSubWindow = false;
			if(1==0 && predRects_Kalman_Window.size() > 35){
				useSubWindow = true;
			}


			if (found)
			{
			    // >>>> Matrix A
			    //kf.transitionMatrix.at<float>(2) = dT;
			    //kf.transitionMatrix.at<float>(9) = dT;

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

			    kf.transitionMatrix.at<float>(2)  = dT;
			    kf.transitionMatrix.at<float>(4)  = (1/2)*dT*dT;
    			    kf.transitionMatrix.at<float>(11) = dT;
			    kf.transitionMatrix.at<float>(13) = (1/2)*dT*dT;				
			    kf.transitionMatrix.at<float>(20) = dT;
			    kf.transitionMatrix.at<float>(29) = dT;
			    // <<<< Matrix A

	//		    cout << "dT:" << endl << dT << endl;

			    state = kf.predict();
	//		    cout << "State post:" << endl << state << endl;

			//    cv::Rect predRect;
			    //predRect.width = state.at<float>(4);
			   // predRect.height = state.at<float>(5);
 			    predRect.width = state.at<float>(6);
			    predRect.height = state.at<float>(7);
			    predRect.x = state.at<float>(0) - predRect.width / 2;
			    predRect.y = state.at<float>(1) - predRect.height / 2;

			    predRects.push_back(predRect);

			//    cv::Point center;
			    center.x = state.at<float>(0);
			    center.y = state.at<float>(1);

			    //////cv::circle(res, center, 2, CV_RGB(255,0,0), -1);	//PLOT KALMAN PREDICTION
			    //////cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
				
			    cv::circle(plotFrame, center, 2, CV_RGB(255,0,0), -1);	//PLOT KALMAN PREDICTION
			    cv::rectangle(plotFrame, predRect, CV_RGB(255,0,0), 2);

			    //Point prevCenter =  Point(trackFrameEstimatePrev.x + trackFrameEstimatePrev.width/2,trackFrameEstimatePrev.y + trackFrameEstimatePrev.height/2);
			    Point currentCenter =  Point(trackFrameEstimate.x + trackFrameEstimate.width/2,trackFrameEstimate.y + trackFrameEstimate.height/2);
			    Point estimateToCurrent = center - currentCenter;
			    cv::line(plotFrame, currentCenter, currentCenter + estimateToCurrent * 4, CV_RGB(212, 112, 0));
 			    cv::circle(plotFrame, currentCenter + estimateToCurrent * 4, 5, CV_RGB(255,0,0), -1);		

			
			    //PLOT PREDICTION WINDOW
			    cv::Rect predTrackRect;
			    predTrackRect.width = trackFrameEstimate.width;
			    predTrackRect.height = trackFrameEstimate.height;
			    predTrackRect.x = currentCenter.x-estimateToCurrent.x - trackFrameEstimate.width/2;
			    predTrackRect.y = currentCenter.y-estimateToCurrent.y - trackFrameEstimate.height/2;  
			    //cv::rectangle(plotFrame, predTrackRect, CV_RGB(255,233,233), 2);	
			}

			// >>>>> Noise smoothing
			//cv::Mat blur;
			//cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
			// <<<<< Noise smoothing

			// >>>>> HSV conversion
			//cv::Mat frmHsv;
			//cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
			// <<<<< HSV conversion

			// >>>>> Color Thresholding
			// Note: change parameters for different colors
			//cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
			//cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
			//	    cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
			// <<<<< Color Thresholding

			// >>>>> Improving the result
			//cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
			//cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
			// <<<<< Improving the result

			// Thresholding viewing
			//cv::imshow("Threshold", rangeRes);

			// >>>>> Contours detection
			//vector<vector<cv::Point> > contours;
			//cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
			//	         CV_CHAIN_APPROX_NONE);
			// <<<<< Contours detection

			// >>>>> Filtering
			//vector<vector<cv::Point> > balls;
			//vector<cv::Rect> ballsBox;
			//for (size_t i = 0; i < contours.size(); i++)
			//{
			//    cv::Rect bBox;
			//   bBox = cv::boundingRect(contours[i]);

			//    float ratio = (float) bBox.width / (float) bBox.height;
			//    if (ratio > 1.0f)
			//	ratio = 1.0f / ratio;
			// Searching for a bBox almost square
			//   if (ratio > 0.75 && bBox.area() >= 400)
			//   {
			//	balls.push_back(contours[i]);
			//	ballsBox.push_back(bBox);
			//   }
			//}
			// <<<<< Filtering

			//cout << "Balls found:" << ballsBox.size() << endl;

			// >>>>> Detection result
			//for (size_t i = 0; i < balls.size(); i++)
			//{
			//    cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
			//    cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

			//    cv::Point center;
			//    center.x = ballsBox[i].x + ballsBox[i].width / 2;
			//    center.y = ballsBox[i].y + ballsBox[i].height / 2;
			//    cv::circle(res, center, 2, CV_RGB(20,150,20), -1);
			//    stringstream sstr;
			//    sstr << "(" << center.x << "," << center.y << ")";
			//    cv::putText(res, sstr.str(),
			//    cv::Point(center.x + 3, center.y - 3),
			//    cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
			//}
			// <<<<< Detection result

			
			// >>>>> Kalman Update
			//if (1==0 && trackFrameEstimate.x < 0)//if (balls.size() == 0)//if (trackFrameEstimatePrev.x < 0)//if (balls.size() == 0)		//CONDITION when not found tracking window from optical flow
			//if (trackFrameEstimate.x + trackFrameEstimate.width/2 < 0 || trackFrameEstimate.y + trackFrameEstimate.height/2 < 0 
			if (	(!useSubWindow && (trackFrameEstimate.x + trackFrameEstimate.width/2 < 0 || trackFrameEstimate.y + trackFrameEstimate.height/2 < 0 
				|| trackFrameEstimate.x + trackFrameEstimate.width/2 > frame.cols || trackFrameEstimate.y + trackFrameEstimate.height/2 > frame.rows))

				|| (useSubWindow && (predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width/2 < 0 
				|| predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height/2 < 0 
				|| predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width/2 > frame.cols 
				|| predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height/2 > frame.rows))	
			)
			//if(trackFrameEstimate.width > 0 && trackFrameEstimate.height > 0 
			//			&& trackFrameEstimate.x > 0 && trackFrameEstimate.x < frame.cols 
			//			&& trackFrameEstimate.y > 0 && trackFrameEstimate.y < frame.rows 
			//			&& trackFrameEstimate.width < frame.cols / 3 && trackFrameEstimate.height < frame.rows / 3)
			{
			    notFoundCount++;
	//		    cout << "notFoundCount:" << notFoundCount << endl;
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

			    //meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
			    // meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
			    //meas.at<float>(2) = (float)ballsBox[0].width;
			    //meas.at<float>(3) = (float)ballsBox[0].height;

				//if measurement is not much distance from previous, update, otherwise use previous
				int previousID = trackFrameEstimateCenters.size()-2;
				float distancePrev = sqrt(pow(trackFrameEstimate.x - trackFrameEstimateCenters[previousID].x, 2) + pow(trackFrameEstimate.y - trackFrameEstimateCenters[previousID].y, 2));
				//float distancePrev = sqrt(pow(trackFrameEstimate.x + trackFrameEstimate.width /2 - (trackFrameEstimateCenters[previousID].x - trackFrameEstimateCenters[previousID].width/2),  2) 
				//			+ pow(trackFrameEstimate.y + trackFrameEstimate.height/2 - (trackFrameEstimateCenters[previousID].y - trackFrameEstimateCenters[previousID].height/2), 2));

				//ACCELERATION
				float speedX = -0.05	*	(trackFrameEstimate.x - trackFrameEstimateCenters[previousID].x) / dT; //-0.05 //0.35				
				float speedY = -0.05	*	(trackFrameEstimate.y - trackFrameEstimateCenters[previousID].y) / dT; //-0.05

				//UPDATE BY SUB WINDOW
				if(useSubWindow){
					speedX = 0.10 * (predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x - predRects_Kalman_Window[predRects_Kalman_Window.size()-2].x) / dT; 
					speedY = 0.10 * (predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y - predRects_Kalman_Window[predRects_Kalman_Window.size()-2].y) / dT; 


					    //PLOT DEBUG SPEED
					    cv::Rect predTrackRect;
					    predTrackRect.width = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width;
					    predTrackRect.height = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height;
					    predTrackRect.x = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x;
					    predTrackRect.y = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y;  
					    cv::rectangle(plotFrame, predTrackRect, CV_RGB(155,233,233), 2);
						predTrackRect.width = predRects_Kalman_Window[predRects_Kalman_Window.size()-2].width;
					    predTrackRect.height = predRects_Kalman_Window[predRects_Kalman_Window.size()-2].height;
					    predTrackRect.x = predRects_Kalman_Window[predRects_Kalman_Window.size()-2].x;
					    predTrackRect.y = predRects_Kalman_Window[predRects_Kalman_Window.size()-2].y;  
					    cv::rectangle(plotFrame, predTrackRect, CV_RGB(255,133,233), 3);	
				}


			 	// Measure Matrix H - MP x DP ACCEL
			  	// [ 1 0 0 0 0 0 0 0]	//measurement of x position
				// [ 0 1 0 0 0 0 0 0]	//measurement of y position
				// [ 0 0 1 0 0 0 0 0]	//measurement of velocity x
				// [ 0 0 0 1 0 0 0 0] 	//measurement of velocity y
				// [ 0 0 0 0 0 0 1 0]	//measurements of width, height
				// [ 0 0 0 0 0 0 0 1]

				//cv::Mat meas(measSize, 1, type);    // [z_x, z_y, z_vx, z_vy, z_w,  z_h]

				if(distancePrev < 1350 || useSubWindow){//if(1 ==1 || distancePrev < 390){ //320
				    meas.at<float>(0) = trackFrameEstimate.x + trackFrameEstimate.width / 2;
				    meas.at<float>(1) = trackFrameEstimate.y + trackFrameEstimate.height / 2;

					//bool updateKalmanByMOSSE = true;
					if(updateKalmanByMOSSE){
						//updateKalmanByMOSSE - 
						//Vec2f estimatedWindow;
						//estimatedWindow[0] = trackFrameEstimate.x;
						//estimatedWindow[1] = trackFrameEstimate.y;

						Point2f estimatedWindow(trackFrameEstimate.x + trackFrameEstimate.width / 2,trackFrameEstimate.y + trackFrameEstimate.height / 2);
						Point2f prevEstimatedWindow(trackFrameEstimateCenters[previousID].x,trackFrameEstimateCenters[previousID].y);
						Point2f mosseEstimatedWindow(trackFrameEstimateAB.x+bboxKALMAN.x + bboxKALMAN.width/2, trackFrameEstimateAB.y+bboxKALMAN.y + bboxKALMAN.height/2);
						Point2f KalmanEstimatedWindow(trackFrameEstimateAB.x, trackFrameEstimateAB.y);

						double diffPrevCurrentEstimation = cv::norm(estimatedWindow-prevEstimatedWindow); //sense jump of measurement comparing to pevious frame  
						double diffPrevMOSSEEstimation = cv::norm(mosseEstimatedWindow-prevEstimatedWindow); //mosse estimate comparing to previous measuement
						double diffKalmanMOSSEEstimation = cv::norm(mosseEstimatedWindow-KalmanEstimatedWindow); //mosse estimate comparing to current Kalman window

						//rectangle(plotFrame, 
						//	Point(trackFrameEstimateAB.x+bboxKALMAN.x,trackFrameEstimateAB.y+bboxKALMAN.y), 
						//	Point(trackFrameEstimateAB.x+bboxKALMAN.x + bboxKALMAN.width,
						//	trackFrameEstimateAB.y+bboxKALMAN.y + bboxKALMAN.height), CV_RGB(0, 10, 255), 2, 8, 0);

						if(diffPrevCurrentEstimation > 100 && (  diffPrevCurrentEstimation > diffPrevMOSSEEstimation || diffPrevCurrentEstimation > diffKalmanMOSSEEstimation )){
		  					meas.at<float>(0) = trackFrameEstimateAB.x+bboxKALMAN.x + bboxKALMAN.width /2;
						    	meas.at<float>(1) = trackFrameEstimateAB.y+bboxKALMAN.y + bboxKALMAN.height/2;
						}
					}


					//UPDATE BY SUB WINDOW
					if(useSubWindow){
						meas.at<float>(0) = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width/2;
						meas.at<float>(1) = predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y + predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height/2;

						Point PrevCenter = Point(predRects_Kalman_Window[predRects_Kalman_Window.size()-2].x + predRects_Kalman_Window[predRects_Kalman_Window.size()-2].width/2,
						predRects_Kalman_Window[predRects_Kalman_Window.size()-2].y + predRects_Kalman_Window[predRects_Kalman_Window.size()-2].height/2);
						Point diffVector = Point(meas.at<float>(0),meas.at<float>(1)) - PrevCenter;
						float diffVectorM = sqrt(diffVector.x*diffVector.x + diffVector.y*diffVector.y);
						cv::line(plotFrame, PrevCenter, PrevCenter + (diffVector/diffVectorM) * 224, CV_RGB(212, 112, 0));

						cv::circle(plotFrame, Point(meas.at<float>(0),meas.at<float>(1)), 25, CV_RGB(255,111,44), -1);
					}
				    meas.at<float>(2) = (float)speedX;
				    meas.at<float>(3) = (float)speedY;

				    meas.at<float>(4) = (float)trackFrameEstimate.width;
				    meas.at<float>(5) = (float)trackFrameEstimate.height;

					//UPDATE BY SUB WINDOW
					if(useSubWindow){//if(predRects_Kalman_Window.size()>1){
						//meas.at<float>(4) = (float) predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width;
					    	//meas.at<float>(5) = (float) predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height;
					}

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
				// >>>> Initialization COVARIANCE Matrix P
				//kf.errorCovPre.at<float>(0) = 1; // px
				//kf.errorCovPre.at<float>(7) = 1; // px
				//kf.errorCovPre.at<float>(14) = 1;
				//kf.errorCovPre.at<float>(21) = 1;
				//kf.errorCovPre.at<float>(28) = 1; // px
				//kf.errorCovPre.at<float>(35) = 1; // px

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

	//		    cout << "Measure matrix:" << endl << meas << endl;
			}
			// <<<<< Kalman Update

			// Final result
			//cv::imshow("Tracking", res);

			// User key
			//ch = cv::waitKey(1);
		}
		////////////////////////////// END KALMAN FILTER ////////////////////////////////

















				float MosseTrackWindowArea = bbox2.height * bbox2.width;
				float AreaDiff = MosseTrackWindowArea - prevArea * (1/scaleFactorLK1) * (1/scaleFactorLK1);
				float distFlowToMosse = sqrt(pow((trackFrameEstimateCenter.x - bbox2.x),2)+pow((trackFrameEstimateCenter.y - bbox2.y),2));
				if(	1 == 0 && 
					(okB == 0 || maxBrightMOSSE > 0.75 || (bbox2.x + bbox2.width/2) > cur2.cols || (bbox2.y + bbox2.height/2) > cur2.rows || (AreaDiff > 200)) 
					&& distFlowToMosse > 20 && prevArea < (cur2.rows * cur2.cols)/9)
				{							
							//REDEFINE TRACK WINDOW							
							bbox2.width = trackFrameEstimate.width;
							bbox2.height = trackFrameEstimate.height;

							//MAKE SQUARE
							//if(bbox2.width > bbox2.height)  { bbox2.height = bbox2.width;  }
							//if(bbox2.height > bbox2.width)  { bbox2.width  = bbox2.height; }
							if(bbox2.width > frame.cols/2) { bbox2.width  = frame.cols/2;}
							if(bbox2.height > frame.rows/2){ bbox2.height = frame.rows/2;}							

							bbox2.x = trackFrameEstimate.x;
							bbox2.y = trackFrameEstimate.y;
							
							tracker2 = trackerUsed::create();						
							tracker2->init(frame, bbox2);
							okB = tracker2->update(frame, bbox2);								
				}



				





				// If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
				if(canvas.cols > 1920) {
					//resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
				}
				//outputVideo<<canvas;
				//namedWindow( "Tracked points before and after", WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
				imshow("Tracked points before and after", canvas);

				if(exportSparseOpticalFlowVideos && !canvas.empty() && canvas.cols > 0){
					//videoOutC;//exportSparseOpticalFlowVideos
					if(!videoOutC.isOpened()){
						//fileNameVIDEO_FLOW
						//videoOutC.open("outcppSparseOpticalFlow.avi",CV_FOURCC('M','J','P','G'),30, Size(canvas.cols,canvas.rows),1);
						videoOutC.open(fileNameVIDEO_FLOW, CV_FOURCC('M','J','P','G'),30, Size(canvas.cols,canvas.rows),1);
					}
					videoOutC.write(canvas);
					//std::cout<< "canvas.channels:" << canvas.channels() << endl;							
				}

				//waitKey(10);
				//
			

				//cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
				k++;


				//TEST LOCALLY
				cur2.copyTo(cur2_prev);
				prev = frameDownscaled.clone();//cur.copyTo(prev);
				frame_grey.copyTo(prevFrame_grey);

				//CROP
				frame_greyCROP.copyTo(prevFrame_greyCROP);
			}
			//-------------------------------------------------------IMAGE STABILIZER rolling window CREATE END--------------------------------------------------- //


			
			//GPU BUFFERS
			//GPU Image manipulation
			Mat srcImgMID;
			Mat srcImgMIDPrev; 

			if(useStabilizer){
				cur2.copyTo(srcImgMID); // frame.copyTo(srcImgMID);
				cur2_prev.copyTo(srcImgMIDPrev);//Mat srcImgMIDPrev = prevFrame; //prevFrame.copyTo(srcImgMIDPrev);

				//cur2_prev = cur2;
				cur2.copyTo(cur2_prev);
				prev = frame.clone();//cur.copyTo(prev);
				frame_grey.copyTo(prevFrame_grey);
			}else{
				frame.copyTo(srcImgMID);
				prevFrame.copyTo(srcImgMIDPrev);
			}

			float resizeGPUTexture = 0.25;//0.26;
			if(1==0 || 1==0 && panRate == 0 && tiltRate == 0 && zoomRate == 0){ //GPU BACKGROUND SUBTRACT !!!

				
				cv::resize(srcImgMID, srcImgMID, cv::Size(), resizeGPUTexture, resizeGPUTexture);
				cv::resize(srcImgMIDPrev, srcImgMIDPrev, cv::Size(), resizeGPUTexture, resizeGPUTexture);
				
				//BLUR IMAGE
				medianBlur(srcImgMID,srcImgMID, 3);
				medianBlur(srcImgMIDPrev,srcImgMIDPrev, 3);
				//medianBlur(srcImgMID,srcImgMID, 9);
				//medianBlur(srcImgMIDPrev,srcImgMIDPrev, 9);
				//medianBlur(srcImgMID,srcImgMID, 9);
				//medianBlur(srcImgMIDPrev,srcImgMIDPrev, 9);
				//imshow("srcImgMID",srcImgMID);
				//imshow("srcImgMIDPrev",srcImgMIDPrev);
		
				panDiff = currentPan - prevPan;
				tiltDiff = currentTilt - prevTilt;
				zoomDiff = currentZoom - prevZoom;

				//cout << panDiff << " ," << tiltDiff  << " ," << zoomDiff << endl;
				cout << panRate << " ," << tiltRate  << " ," << zoomRate << endl;				

				//GLuint texID, texIDPrev, texIDPrev2;
				//cv::Mat prevSrcImg;
				//BindCVMat2GLTexture(srcImgMID, texID, 0);

				//v0.1		
				if(!initGPU){
					//BindCVMat2GLTexture(srcImgMIDPrev, texIDPrev, 0);
					initGPU = true;
					//rectify360imageGPU(texID, texIDPrev, texIDPrev2, true, 0, panRate, tiltRate, zoomRate, imageScaleBackgroundSUBTRACT);
				}else{
					//BindCVMat2GLTexture(backgroundMaskGPU, texIDPrev, 0);	//insert mask
					//BindCVMat2GLTexture(srcImgMIDPrev, texIDPrev2, 0);	//insert previous frame
					//rectify360imageGPU(texID, texIDPrev, texIDPrev2, true, 1, panRate, tiltRate, zoomRate, imageScaleBackgroundSUBTRACT);
				}
		
				//backgroundMaskGPU = rectify360imageGPU(texID, texIDPrev, texIDPrev2, true, 0, panDiff, tiltDiff, zoomDiff);
			
				//cout << trackWindow << endl;
				//glDeleteTextures(1, &texID);

				//}
				///// END GPU BACKGROUND SUBTRACT !!!!
			}
			//else		
			if(1==0){
			//if(1==1 && !frame.empty() && !prevFrame.empty()){ //CHECK OPENCL WORKS - Not working, must install Intel SDK, then build Opencv and maybe work ////// DENSE OPTICAL FLOW
						const bool useGpu = true;
						cv::ocl::setUseOpenCL(useGpu);
	    					//printf("OpenCL Enabled:resizeGPUTexture %u\n", useGpu && cv::ocl::haveOpenCL());
						//cout << "Has Opencl: " << cv::ocl::haveOpenCL() << endl;
						//vector<ocl::PlatformInfo> platforms;
						//try {
						//	ocl::getPlatfomsInfo(platforms);
						//}
						//catch (cv::Exception& e)
						//{
						//	const char* err_msg = e.what();
						//	std::cout << "exception caught: " << err_msg << std::endl;
						//}

						//if (!ocl::haveOpenCL()) {
						//	std::cout << "OpenCL is not available..." << std::endl;
						//}

						//Mat frameA,prevFrameA;
						frame.copyTo(srcImgMID);
						prevFrame.copyTo(srcImgMIDPrev);
						//float scaleX = 640/1920;
						//float scaleY = 480/1080;
						//cv::resize(srcImgMID, srcImgMID, cv::Size(),0.3333/2,0.4444/2);
						//cv::resize(srcImgMIDPrev, srcImgMIDPrev, cv::Size(), 0.3333/2,0.4444/2);
						cv::resize(srcImgMID, srcImgMID, cv::Size(),resizeGPUTexture * 1,resizeGPUTexture* 1);
						cv::resize(srcImgMIDPrev, srcImgMIDPrev, cv::Size(), resizeGPUTexture* 1,resizeGPUTexture* 1);

						//CHECK PREVIOUS FRAME DIFFERENT
						//imshow( "frame flow", frameA );
						//imshow( "frame Prev flow", prevFrameA );
						//imshow( "frame flow", srcImgMID );
						//imshow( "frame Prev flow", srcImgMIDPrev );
						//Scalar s = sum( frameA - prevFrameA );
						//bool equal = (s[0]==0) && (s[1]==0) && (s[2]==0);
						//if(equal){
						//	cout << "SAME !!!!!!!!!!!!" << endl;
						//}
						Mat_<Point2f> flow, ground_truth;
						cvtColor(srcImgMID,srcImgMID, COLOR_BGR2GRAY);
						cvtColor(srcImgMIDPrev,srcImgMIDPrev, COLOR_BGR2GRAY);

						flow = Mat(srcImgMID.size[0], srcImgMID.size[1], CV_32FC2);
					    	//Ptr<DenseOpticalFlow> algorithm;					    	
						//algorithm = createOptFlow_Farneback();
						calcOpticalFlowFarneback(srcImgMIDPrev, srcImgMID, flow.getUMat(ACCESS_RW), 0.5, 3, 15, 3, 5, 1.2, 0);
						//algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); //algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); // PRESET_ULTRAFAST // PRESET_FAST
						if (useGpu){
							//algorithm->calc(frameA, prevFrameA, flow);//algorithm->calc(frameA, prevFrameA, flow.getUMat(ACCESS_RW));
							//algorithm->calc(srcImgMID, srcImgMIDPrev, flow.getUMat(ACCESS_RW));
						}		
						else {
							//algorithm->calc(srcImgMID, srcImgMIDPrev, flow);
						}	
						if( 1==1 )
					    	{
							//Mat flow_image = flowToDisplay(flow);
							//namedWindow( "Computed flow", WINDOW_AUTOSIZE );
							//imshow( "Computed flow", flow_image );
		
							Mat cflow;
							int step = 9;//32/2;
							int scale = 12/1;
							cvtColor(srcImgMIDPrev, cflow, CV_GRAY2BGR);
							//prevFrame.copyTo(cflow);
							//drawOptFlowMapB(flow, cflow, step, scale, CV_RGB(0, 255, 0));
							cv::resize(cflow, cflow, cv::Size(), 2, 2);
							//imshow("optical Flow", cflow);
					    	}
			}
		}
		////////////////





		 	
		//PLOT FROM CSV FILE
		if(plotFromCSV){
			Rect2d bboxCSV(pointsCSVx[frameCounter-1], pointsCSVy[frameCounter-1], pointsCSVw[frameCounter-1], pointsCSVh[frameCounter-1]);			
			rectangle(plotFrame,bboxCSV, Scalar( 0, 255, 0 ), 2, 1 );
		}
			

		if (ok)
		{
			// Tracking success : Draw the tracked object
			//rectangle(plotFrame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
			rectangle(plotFrame, bbox2, Scalar( 255, 255, 255 ), 2, 1 );		/////// MAIN TRACKER WINDOW DRAW
			


//ZOOM - PAN TEST SIN !!!!
if(!enablePIDracking){

					

					//Pan is 0 at home. Right is positive, max 2448. Left ranges from full
					//left 63088 to 65535 before home.
					//Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully
					//depressed at 65104 to 65535 before home.  Zoom is 0 to 16384					
					if(frameCounter < 50){//if(frameCounter < 1100){
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
							if(currentZoom > maxTestZoom){//if(currentZoom > 4800){
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
				//			cout << "currentZoom=" << currentZoom << endl;							
						}

						int randomNum = rand() % 4;

				//		cout << "currentPan="  << currentPan  << endl;
						//HANDLE PAN
						if(1==0){
							if(currentPan > 300 && currentPan < 5000){
								//float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
								//int panDiff = (int)(panDiffF);
								string VV = "04";  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero, left = 103, right = 203
								//if(panDiff < 10){						
									//add pading
								//	VV = "0"+VV;
								//}
								string WW = "01";
								std::string panPTZ = "81010601"+VV+WW+"0103FF";
								//cout << "panPTZ = " <<  panPTZ << endl;
								udpSend(panPTZ, false, false);
								//panRate = -panDiffF;

								//PTZ Optics x20 camera specs
								//1.7 to 100 degrees per second Pan speed (01 to 18) (to 24)
								//1.7 to 69.9 degrees per second Tilt speed (01 to 14) (to 20)
								int speed = 4;
								panRate = speed * 100 / 18;
							}
							else if(currentPan < 50 || currentPan > 65000){
								//float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
								//int panDiff = (int)(panDiffF);
								string VV = "04";//string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
								//if(panDiff < 10){
									//add pading
								//	VV = "0"+VV;
								//}
								string WW = "01";
								std::string panPTZ = "81010601"+VV+WW+"0203FF";
								//cout << "panPTZ = " <<  panPTZ << endl;
								udpSend(panPTZ, false, false);
								//panRate = panDiffF; //pan left, background points must move to right

								int speed = 4;
								panRate = -speed * 100 / 18;
							}
						}
						//HANDLE TILT
						if(1==0){
							if(currentTilt > 200 && currentTilt < 50000){							
								string VV = "12";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
								string WW = "05"; //TILT SPEED WW
								std::string panPTZ = "81010601"+VV+WW+"0302FF";//down   std::string panPTZ = "81010601"+VV+WW+"0103FF";							
								udpSend(panPTZ, false, false);
								//PTZ Optics x20 camera specs							
								//1.7 to 69.9 degrees per second Tilt speed (01 to 14) (to 20)
								int speed = 5;
								tiltRate = speed * 70 / 14;
							}
							else if(currentTilt < 20 || currentTilt > 65000){							
								string VV = "12";//string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. 							
								string WW = "05";
								std::string panPTZ = "81010601"+VV+WW+"0301FF";//up   std::string panPTZ = "81010601"+VV+WW+"0203FF";							
								udpSend(panPTZ, false, false);
								int speed = 5;
								tiltRate = -speed * 70 / 14;
							}				
						}	
						//HANDLE PAN - TILT combination
						if(enableTestPanTilt){
							string VV = "02";  //pan 0x01 to 0x18, tilt to 0x14. left = 103, right = 203							
							string WW = "02";
							int panSpeed = 2;
							int tiltSpeed = 2;
							if(currentTilt > 180 && currentTilt < maxTestTilt){//if(currentTilt > 250 && currentTilt < 50000)						
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
				//		cout << "currentTilt="  << currentTilt  << endl;
					}
}/////END ZOOM - PAN TEST SIN !!!!


			//IMPLEMENT PTZ CONTROLS, 5678 = TCP port
			if(enablePIDracking){
				//const char* homePTZ = "81010604FF";
				//std::string homePTZ = "81010604FF";
				//udpSend(homePTZ, false);

				//float PID_P = 0.75f;//0.75f;
				float ScreenCenterX = plotFrame.cols/2; 
				float ScreenCenterY = plotFrame.rows/2; 
				float PID_X_DIFF = (bbox2.x + (bbox2.width/2) - ScreenCenterX);
				float PID_Y_DIFF = (bbox2.y + (bbox2.height/2) - ScreenCenterY);
				//float PID_Control_Pan = PID_P * PID_X_DIFF;
				float maxPanSpeed = 8;
				float proximityDivider = 22;

				//cout << "ok:" << ok << " okA:" << okA << " okB:" << okB << endl;

				bool box2Same = false;
				if(bbox2P.x == bbox2.x && bbox2P.y == bbox2.y){
					box2Same = true;
				}

				//if(box2Same || !okB || okB == 0 || abs(PID_X_DIFF) > ScreenCenterX * 0.85f || frameCounter < 110 || abs(PID_Y_DIFF) > ScreenCenterY * 0.85f ){
				if(box2Same || !okB || okB == 0  || frameCounter < 50 ){
					
					//zoom out
					string PP = "2";//zoom speed 0 to 7
					std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
					udpSend(zoomPTZ, false, false);

					//cout << "RESETTING" << endl;
					//std::string getPanTiltPTZ = "81090612FF"; //read pan - tilt
					//udpSend(getPanTiltPTZ, true, false);


					///HANDLE PAN	
					bool addRandomPan = false; //add random pan to emulate drone motion
					if(addRandomPan && frameCounter > 1100 ){
						if(currentPan > 300){							
							string VV = "03";  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero, left = 103, right = 203
							string WW = "01";
							std::string panPTZ = "81010601"+VV+WW+"0103FF";							
							udpSend(panPTZ, false, false);							
						}
						else if(currentPan < 50){							
							string VV = "02";//string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
							string WW = "01";
							std::string panPTZ = "81010601"+VV+WW+"0203FF";							
							udpSend(panPTZ, false, false);							
						}
					}else{
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
					}



					panRate = 0;
					tiltRate = 0;
					zoomRate = 0;





					//STATIC BACKGROUND - Find tracking windows
					//cout << "okB = " << okB << endl;
					if(1==1 || panDiff == 0 && tiltDiff == 0 && zoomDiff == 0){
						cout << "okB = " << okB << endl;
						
						Rect2d processVideoFrame = trackWindow;//processVideo(frameSUBRACT,imageScaleBackgroundSUBTRACT, filterPixels, repeats);
						if(processVideoFrame.x != 0 && processVideoFrame.width > 0
							//&& (
							//(processVideoFrame.width > processVideoFrame.height && processVideoFrame.width / processVideoFrame.height < 4) 
							//||
							//(processVideoFrame.height > processVideoFrame.width && processVideoFrame.height / processVideoFrame.width < 4)
							//)
						){
							///BACKGROUND SUBTRACTION BOX ESTIMATION WITH CONFIDENCE METRIC
							//vector<Rect2d> bboxesESTIMATED_STATIC; //estimated bboxes with background subtraction
							//vector<int> bboxESTIMATE_STATIC_frames; //frames of confidence
							if(1==0){	//if(backStatic){
								//estimate bboxes and if confidence high, provide this for bbox below, than current
								//if current bbox distance and area difference from one in the list small, average them
								//if not, add new member to the list
								bool foundBoxSimilar = false;
								double estimateStaticDistThreshold = 50; //30 pixel displacement
								double estimateStaticAreaDiffThreshold = 500; //area difference

								for(int i = 0; i < bboxesESTIMATED_STATIC.size(); i += 1){
									double xDiff = bboxesESTIMATED_STATIC[i].x - processVideoFrame.x;
									double yDiff = bboxesESTIMATED_STATIC[i].y - processVideoFrame.y;
									double distance = sqrt(xDiff*xDiff + yDiff*yDiff);
									double areaA = bboxesESTIMATED_STATIC[i].width * bboxesESTIMATED_STATIC[i].height;
									double areaB = processVideoFrame.width * processVideoFrame.height;
									double areaDiff = abs(areaA - areaB);							
									if(distance < estimateStaticDistThreshold && areaDiff < estimateStaticAreaDiffThreshold){
										foundBoxSimilar = true;								
										bboxesESTIMATED_STATIC[i] = processVideoFrame; //take average here !!!! TO DO
										bboxESTIMATE_STATIC_frames[i] = bboxESTIMATE_STATIC_frames[i] + 1;//increase confidense
										//cout << "found similar in:" << i << " with confidense:" 
										//<< to_string(bboxESTIMATE_STATIC_frames[i]) << " frames" << endl;
									}
								}
								//if not found similar, add to the list
								if(!foundBoxSimilar){
									bboxesESTIMATED_STATIC.push_back(processVideoFrame); 
									bboxESTIMATE_STATIC_frames.push_back(1); //start with single frame confidence
								}

								//choose the highest confidense item from the list 
								//and if confidence higher than threshold replace processVideoFrame
								double estimated_static_confidence_threshold = 6; //close window appear in at least 3 frames
								double previousMaxConfidence = 0;
								for(int i = 0; i < bboxesESTIMATED_STATIC.size(); i += 1){						
									if(bboxESTIMATE_STATIC_frames[i] > previousMaxConfidence
									   && bboxESTIMATE_STATIC_frames[i] >= estimated_static_confidence_threshold)
									{ //if found better, replace
										processVideoFrame = bboxesESTIMATED_STATIC[i];
										previousMaxConfidence = bboxESTIMATE_STATIC_frames[i];
										//cout << "replaced window with confidense:" << previousMaxConfidence << endl;
									}
								}	
								///rectangle(plotFrame, processVideoFrame, Scalar( 255, 110, 0 ), 2, 1 );	
							}
					
							//cout << "Found rect " << processVideoFrame.x << " , " << processVideoFrame.y << ", width = " 
							//<< processVideoFrame.width << ", height = " << processVideoFrame.height << endl;					
								     	
							bbox2.width = processVideoFrame.width * 1.1 / imageScaleBackgroundSUBTRACT;
							bbox2.height = processVideoFrame.height * 1.1 / imageScaleBackgroundSUBTRACT;

							//MAKE SQUARE
							if(bbox2.width > bbox2.height)  { bbox2.height = bbox2.width;  }
							if(bbox2.height > bbox2.width)  { bbox2.width  = bbox2.height; }
							if(bbox2.width > frame.cols/2) { bbox2.width  = frame.cols/4;}
							if(bbox2.height > frame.rows/2){ bbox2.height = frame.rows/4;}
							bbox2.width = bbox2.width * 1.1;

							bbox2.x = processVideoFrame.x / imageScaleBackgroundSUBTRACT;// - bbox2.width/2;
							bbox2.y = processVideoFrame.y / imageScaleBackgroundSUBTRACT;// - bbox2.height/2;

							//bbox2.width = bbox.width;
							//bbox2.height = bbox.height;	
							//bbox2.x = bbox.x;
							//bbox2.y = bbox.y;
					
							//tracker = trackerUsed::create();						
							//tracker->init(frame, bbox);
							//tracker->update(frame, bbox);

							

							if(okB == 0){
								tracker2 = trackerUsed::create();						
								tracker2->init(frame, bbox2);
								okB = tracker2->update(frame, bbox2);	
							}	
										

							//fullFlowMap = false;
							//cout << " ... and Background subtraction" << endl;				
						}
					}
//cout << "PAN ENABLED 111111111111111111aaa" << endl;
//cout << "PAN ENABLED 111111111111111111aaa" << endl;
//cout << "PAN ENABLED 111111111111111111aaa" << endl;

				}else{

					//RESET TRACK WINDOWS CANDIDATES
					bboxesESTIMATED_STATIC.clear();
					bboxESTIMATE_STATIC_frames.clear();


//SINUSOIDAL MOTION TEST
if(1==0){
					if(1==1){
						float panDiffF = 1;
						int panDiff = (int)(panDiffF);
						string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero, left = 103, right = 203
						if(panDiff < 10){						
							//add pading
							VV = "0"+VV;
						}
						string WW = "01";
						std::string panPTZ = "81010601"+VV+WW+"0203FF"; //PAN RIGHT
						//cout << "panPTZ = " <<  panPTZ << endl;
						udpSend(panPTZ, false, false);
						panRate = -panDiffF;
					}

}

						

//PAN MOTION ENABLE
if(1==0){
cout << "PAN ENABLED 111111111111111111" << endl;
cout << "PAN ENABLED 111111111111111111" << endl;
cout << "PAN ENABLED 111111111111111111" << endl;
					///HANDLE PAN
					if(PID_X_DIFF > ScreenCenterX/proximityDivider){
cout << "PAN ENABLED 222" << endl;
cout << "PAN ENABLED 222" << endl;
cout << "PAN ENABLED 222" << endl;
						float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
						int panDiff = (int)(panDiffF);
						string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero, left = 103, right = 203
						if(panDiff < 10){						
							//add pading
							VV = "0"+VV;
						}
						string WW = "01";
						std::string panPTZ = "81010601"+VV+WW+"0203FF";
						//cout << "panPTZ = " <<  panPTZ << endl;
						udpSend(panPTZ, false, false);
						
						//panRate = -panDiffF;
						int panSpeed = panDiff; // VV
						//int tiltSpeed = 4; // WW												
						//if tilt looks up, go down right
						//tiltRate = tiltSpeed * 70 / 14;
						panRate = -panSpeed * 100 / 18;
					}
					if(PID_X_DIFF <= -ScreenCenterX/proximityDivider){
cout << "PAN ENABLED 333" << endl;
cout << "PAN ENABLED 333" << endl;
cout << "PAN ENABLED 333" << endl;
						float panDiffF = PID_P * maxPanSpeed * (abs(PID_X_DIFF) / ScreenCenterX);
						int panDiff = (int)(panDiffF);
						string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
						if(panDiff < 10){
							//add pading
							VV = "0"+VV;
						}
						string WW = "01";
						std::string panPTZ = "81010601"+VV+WW+"0103FF";
						//cout << "panPTZ = " <<  panPTZ << endl;
						udpSend(panPTZ, false, false);

						//panRate = panDiffF; //pan left, background points must move to right
						int panSpeed = panDiff; // VV
						//int tiltSpeed = 4; // WW												
						//if tilt looks up, go down right
						//tiltRate = tiltSpeed * 70 / 14;
						panRate = panSpeed * 100 / 18;
					}
}

///TILT MOTION ENABLE
if(1==0){
					//HANDLE TILT
					if(PID_Y_DIFF > ScreenCenterY/proximityDivider){
						float panDiffF = PID_P * maxPanSpeed * (abs(PID_Y_DIFF) / ScreenCenterY);
						int panDiff = (int)(panDiffF);
						string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero, left = 103, right = 203
						if(panDiff < 10){
							//add pading
							VV = "0"+VV;
						}
						string WW = "01";
						std::string panPTZ = "81010601"+WW+VV+"0302FF";//std::string panPTZ = "81010601"+VV+WW+"0302FF";//down
						//cout << "panPTZ = " <<  panPTZ << endl;
						udpSend(panPTZ, false, false);

						//tiltRate = -panDiffF;
						//int panSpeed = panDiff; // VV
						int tiltSpeed = panDiff; // WW												
						//if tilt looks up, go down right
						tiltRate = tiltSpeed * 70 / 14;
						//panRate = panSpeed * 100 / 18;
					}
					if(PID_Y_DIFF <= -ScreenCenterY/proximityDivider){
						float panDiffF =PID_P * maxPanSpeed * (abs(PID_Y_DIFF) / ScreenCenterY);					
						int panDiff = (int)(panDiffF);
						string VV = to_string(panDiff);  //pan 0x01 to 0x18, tilt to 0x14. If plotFrame.cols/2, pan 18, if zero pan zero
						if(panDiff < 10){
							//add pading
							VV = "0"+VV;
						}
						string WW = "01";
						std::string panPTZ = "81010601"+WW+VV+"0301FF";//up
						//cout << "panPTZ = " <<  panPTZ << endl;
						udpSend(panPTZ, false, false);

						//tiltRate = panDiffF;
						//int panSpeed = panDiff; // VV
						int tiltSpeed = panDiff; // WW												
						//if tilt looks up, go down right
						tiltRate = -tiltSpeed * 70 / 14;
						//panRate = panSpeed * 100 / 18;
					}
}

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
							float diffThreshold = 5;
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - right
								std::string panPTZ = "81010601"+VV+WW+"0201FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
							if(PID_X_DIFF > diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - right
								std::string panPTZ = "81010601"+VV+WW+"0202FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = -panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF <= diffThreshold){ //go up - left
								std::string panPTZ = "81010601"+VV+WW+"0101FF"; //
								tiltRate = -tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}
							if(PID_X_DIFF <= diffThreshold && PID_Y_DIFF > diffThreshold){ //go down - left
								std::string panPTZ = "81010601"+VV+WW+"0102FF"; //
								tiltRate = tiltSpeed * 70 / 14;
								panRate = panSpeed * 100 / 18;
								udpSend(panPTZ, false, false);
							}

							
							//if(currentTilt > 180 && currentTilt < 50000){  //if(currentTilt > 250 && currentTilt < 50000){							
							//	std::string panPTZ = "81010601"+VV+WW+"0202FF"; //if tilt looks up, go down right
							//	tiltRate = tiltSpeed * 70 / 14;
							//	panRate = -panSpeed * 100 / 18;
							//	udpSend(panPTZ, false, false);
							//}
							//else if(currentTilt < 1 || currentTilt > 65300){
							//	std::string panPTZ = "81010601"+VV+WW+"0101FF";
							//	tiltRate = -tiltSpeed * 70 / 14;
							//	panRate = panSpeed * 100 / 18;	
							//	udpSend(panPTZ, false, false);			
							//}											
											
						}	
//END HANDLE PAN - TILT combination

//ZOOM MOTION ENABLE
if(enablePIDZoom && 1==1){
					int zoomSpeed = 3;
					//if(abs(PID_X_DIFF) < ScreenCenterX/proximityDivider && abs(PID_Y_DIFF) < ScreenCenterY/proximityDivider){ //when centered, zoom in
					if(abs(PID_X_DIFF) < 1250 && abs(PID_Y_DIFF) < 1250 && currentZoom < 2400){ //when centered, zoom in
						//stop motion
						//string WW = "00";
						//std::string panPTZ = "81010601"+WW+WW+"0303FF";
						//udpSend(panPTZ, false, false);
						//ZOOM IN				
						string PP = "1";//zoom speed 0 to 7 //5
						std::string zoomPTZ = "810104072"+PP+"FF"; //zoom out, zoom in = 810104072pFF
						udpSend(zoomPTZ, false, false);

						zoomSpeed = 1;
						zoomRate = zoomSpeed * 120 / 7;
					}else{					

						if(1==0){
							string PP = "3";//zoom speed 0 to 7 //2
							std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out, zoom in = 810104072pFF
							udpSend(zoomPTZ, false, false);

							zoomSpeed = 3;
							zoomRate = -zoomSpeed * 120 / 7;
						}else{
							//STOP ZOOM					
							std::string zoomStopPTZ = "8101040700FF"; //stop zoom
							udpSend(zoomStopPTZ, false, false);
							zoomRate=0;
						}
					}

							//int zoomSpeed = 3;
							//if(currentZoom > 5800){//if(currentZoom > 4800){
								//string PP = "3";//zoom speed 0 to 7
								//std::string zoomPTZ = "810104073"+PP+"FF"; //zoom out = 810104073pFF
								//udpSend(zoomPTZ, false, false);
								//zoomRate = -zoomSpeed * 120 / 7;
							//}else if(currentZoom < 5){
								//string PP = "3";
								//std::string zoomPTZ = "810104072"+PP+"FF"; //zoom in  = 810104072pFF
								//udpSend(zoomPTZ, false, false);
								//zoomRate = zoomSpeed * 120 / 7;
							//}

					//ZOOM IN				
					//string PP = "1";//zoom speed 0 to 7
					//std::string zoomPTZ = "810104072"+PP+"FF"; //zoom out, zoom in = 810104072pFF
					//udpSend(zoomPTZ, false, false);

					//if(currentZoom < 425){
					//	zoomRate=0;
					//}

}
				
	  			}
			
			}
			//END IMPLEMENT PTZ CONTROLS, 5678 = TCP port	  
		
			bbox2P.x = bbox2.x;
			bbox2P.y = bbox2.y;
			bbox2P.width = bbox2.width;
			bbox2P.height = bbox2.height;
			
		}
		else
		{
		    // Tracking failure detected.
		    putText(plotFrame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
		}
		 


		///////////PLOT TRAJECTORIES
		if(1==0){
			for(int i = 0; i < trajectory_Poins.size(); i += 1){ 
				int sizeM = trajectory_Poins[i].size();
				if( trajectory_Poins[i].size() > 10){
					for(int j = 0; j< trajectory_Poins[i].size(); j += 1){ 
						//line(cflow, Point(cvRound((cflow.cols/2)+0), cvRound((cflow.rows/2)+0)), Point(cvRound((cflow.cols/2)+sumDX), cvRound((cflow.rows/2)+sumDY)), CV_RGB(255, 255, 0));
						if(j > 0){
							line(plotFrame, trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5)),trajectory_Poins[i][j-1]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 
							CV_RGB(2*sizeM, 2*sizeM, 0));//CV_RGB(255*sizeM, 255*sizeM, 0));
						}
					}
				}
			}
		}
		////////////////PLOT LAST TRAJECTORIES POINT AND DEFINE TRACK WINDOW BY OPTICAL FLOW ON BACKGROUND SUBRACTION USED TO FILL LATEST TRAJECTORY POINT
		if(1==0){
			for(int i = 0; i < trajectory_Poins.size(); i += 1){ 
				int sizeM = trajectory_Poins[i].size();
				if( trajectory_Poins[i].size() > 0 && i ==  trajectory_Poins.size() - 1){
					
					for(int j = 0; j < trajectory_Poins[i].size(); j += 1){ 
						//line(cflow, Point(cvRound((cflow.cols/2)+0), cvRound((cflow.rows/2)+0)), Point(cvRound((cflow.cols/2)+sumDX), cvRound((cflow.rows/2)+sumDY)), CV_RGB(255, 255, 0));
						if(j > 0 && j == trajectory_Poins[i].size()-1){
							//line(plotFrame, trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5)),trajectory_Poins[i][j-1]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 
							//CV_RGB(2*sizeM, 2*sizeM, 0));//CV_RGB(255*sizeM, 255*sizeM, 0));
							//circle(plotFrame, trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 22,  CV_RGB(2*sizeM, 2*sizeM, 0), -1);
							circle(plotFrame, trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 52,  CV_RGB(42*sizeM/112, 2*sizeM/112, 220/121), -1);
						}
						//compare last 3 samples to current
						
						if(1==1 && trajectory_Poins[i].size() > 4 && j == trajectory_Poins[i].size()-1 ){
							float prevX5 = trajectory_Poins[i][j-4].x;
							float prevY5 = trajectory_Poins[i][j-4].y;
							float prevX4 = trajectory_Poins[i][j-3].x;
							float prevY4 = trajectory_Poins[i][j-3].y;
							float prevX3 = trajectory_Poins[i][j-2].x;
							float prevY3 = trajectory_Poins[i][j-2].y;
							float prevX2 = trajectory_Poins[i][j-1].x;
							float prevY2 = trajectory_Poins[i][j-1].y;
							float currentX = trajectory_Poins[i][j].x;
							float currentY = trajectory_Poins[i][j].y;
							float DIFF2 = sqrt((currentX-prevX2)*(currentX-prevX2) + (currentY-prevY2)*(currentY-prevY2));
							float DIFF3 = sqrt((currentX-prevX3)*(currentX-prevX3) + (currentY-prevY3)*(currentY-prevY3));
							float DIFF4 = sqrt((currentX-prevX4)*(currentX-prevX4) + (currentY-prevY4)*(currentY-prevY4));
							float DIFF5 = sqrt((currentX-prevX5)*(currentX-prevX5) + (currentY-prevY5)*(currentY-prevY5));
							float DIFFthreshold = 95;///65;
							int confidenceLevel = 0;
							if(DIFF2 < DIFFthreshold && DIFF2 != 0){
								confidenceLevel++;
							}
							if(DIFF3 < DIFFthreshold && DIFF3 != 0){
								confidenceLevel++;
							}
							if(DIFF4 < DIFFthreshold && DIFF4 != 0){
								confidenceLevel++;
							}
							if(DIFF5 < DIFFthreshold && DIFF5 != 0){
								confidenceLevel++;
							}
							if(confidenceLevel > 1){
							//if(DIFF2 < DIFFthreshold && DIFF3 < DIFFthreshold && DIFF4 < DIFFthreshold && DIFF2 != 0  && DIFF3 != 0 && DIFF4 != 0){
								Point cicleCenter = trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5));
								float scaledScale = trajectory_Poins_scale[j]*1;

								//fix center half way current and previous frame
								cicleCenter = (trajectory_Poins[i][j]/2+trajectory_Poins[i][j-1]/2)*(1/(imageScaleBackgroundSUBTRACT*0.5));

								circle(plotFrame, cicleCenter, 22, CV_RGB(2*sizeM, 2*sizeM, 0), -1); //trajectory_Poins_scale

								rectangle(plotFrame, Point(cicleCenter.x - scaledScale/2,cicleCenter.y - scaledScale/2), 
								Point(cicleCenter.x + scaledScale/2,cicleCenter.y + scaledScale/2), CV_RGB(255, 0, 255), 12, 8, 0);

								circle(plotFrame, trajectory_Poins[i][j-1]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 15, CV_RGB(1*sizeM/11, 1*sizeM/99, 0), -1);
								circle(plotFrame, trajectory_Poins[i][j-2]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 8,  CV_RGB(0.5*sizeM/39, 0.5*sizeM/99, 0), -1);
								circle(plotFrame, trajectory_Poins[i][j-3]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 4,  CV_RGB(0.2*sizeM/99, 0.2*sizeM/99, 0), -1);

								//////SENSE when mosse works, by checking previous tracking window is 1. moving and 2. not moving too fast

								////// RESTART MOSSE
								//if(okB == 0 && abs(bbox2.x - (cicleCenter.x - scaledScale/2)) > 110 && abs(bbox2.y - (cicleCenter.y - scaledScale/2)) > 110 ){
								//if(abs(bbox2.x - (cicleCenter.x - scaledScale/2)) > 80 && abs(bbox2.y - (cicleCenter.y - scaledScale/2)) > 80 ){
								//if(okB == 0 || abs((bbox2.x+bbox2.width/2) - (cicleCenter.x )) > 20 || abs((bbox2.y+bbox2.height/2) - (cicleCenter.y )) > 20 ){
								float termX = (bbox2.x+bbox2.width/2) - (cicleCenter.x);
								float termY = (bbox2.y+bbox2.height/2) - (cicleCenter.y);
								float distBoxCenter = sqrt(termX*termX + termY*termY);
								//if(okB == 0 || (abs((bbox2.x+bbox2.width/2) - (cicleCenter.x )) > 190 && abs((bbox2.y+bbox2.height/2) - (cicleCenter.y )) > 190) ){
								if(okB == 1 && distBoxCenter > 90){
									//if confident center away from actual window, reset
									okBs++;
									//debugActionsString ="GC:3 - MOSSE STATE: " + SSTR(okB) + " C: "+SSTR(okBs) + " Sect:1";
									section = 1;
								}
								if(okB == 0 && confid < 2){
									confid++;			
								}
								if((okB == 0 && confid >=2) || okBs > 12 ){								
								//if(okB == 0 || distBoxCenter > 160 ){
									section = 2; 
									debugActionsString ="G:3 ST:" + SSTR(okB) + " C2:"+SSTR(okBs) + " C1:"+SSTR(confid);
									okBs = 0; confid=0;
									bbox2.width = 160;//4*scaledScale/2;
									bbox2.height = 160;// 4*scaledScale/2;
									//MAKE SQUARE
									if(bbox2.width > bbox2.height)  { bbox2.height = bbox2.width;  }
									if(bbox2.height > bbox2.width)  { bbox2.width  = bbox2.height; }
									if(bbox2.width > frame.cols/4) { bbox2.width  = frame.cols/6;}
									if(bbox2.height > frame.rows/4){ bbox2.height = frame.rows/6;}
									bbox2.width = bbox2.width * 1.1;
									bbox2.x = cicleCenter.x - 80;//- 4*scaledScale/2;
									bbox2.y = cicleCenter.y - 80;//- 4*scaledScale/2;
									
									//if(okB == 0){
										tracker2 = trackerUsed::create();						
										tracker2->init(frame, bbox2);
										okB = tracker2->update(frame, bbox2);
									//}			
								}						
								/////// END RESTART MOSSE

							}else{
								//debugActionsString = "GC:0 STATE: " + SSTR(okB) + " C: "+SSTR(okBs) + " FromSect: " + SSTR(section);
							}	
							putText(plotFrame, debugActionsString, Point(bbox2.x,bbox2.y-11), 
							FONT_HERSHEY_SIMPLEX, 0.6, Scalar(25,10,250), 2);					
						}
						//compare last 10 samples, each to its previous
						if(1==0 && trajectory_Poins[i].size() > 6 && j == trajectory_Poins[i].size()-1 ){
							float prevX7 = trajectory_Poins[i][j-6].x;
							float prevY7 = trajectory_Poins[i][j-6].y;
							float prevX6 = trajectory_Poins[i][j-5].x;
							float prevY6 = trajectory_Poins[i][j-5].y;
							float prevX5 = trajectory_Poins[i][j-4].x;
							float prevY5 = trajectory_Poins[i][j-4].y;
							float prevX4 = trajectory_Poins[i][j-3].x;
							float prevY4 = trajectory_Poins[i][j-3].y;
							float prevX3 = trajectory_Poins[i][j-2].x;
							float prevY3 = trajectory_Poins[i][j-2].y;
							float prevX2 = trajectory_Poins[i][j-1].x;
							float prevY2 = trajectory_Poins[i][j-1].y;
							float currentX = trajectory_Poins[i][j].x;
							float currentY = trajectory_Poins[i][j].y;
							float DIFF2 = sqrt((currentX-prevX2)*(currentX-prevX2) + (currentY-prevY2)*(currentY-prevY2));
							float DIFF3 = sqrt((prevX2-prevX3)*(prevX2-prevX3) + (prevX2-prevY3)*(prevX2-prevY3));
							float DIFF4 = sqrt((prevX3-prevX4)*(prevX3-prevX4) + (prevX3-prevY4)*(prevX3-prevY4));
							float DIFF5 = sqrt((prevX4-prevX5)*(prevX4-prevX5) + (prevX4-prevY5)*(prevX4-prevY5));
							float DIFF7 = sqrt((prevX6-prevX7)*(prevX6-prevX7) + (prevX6-prevY7)*(prevX6-prevY7));

							float DIFFthreshold = 125;
							//if(DIFF2 < DIFFthreshold && DIFF3 < DIFFthreshold && DIFF4 < DIFFthreshold && DIFF5 < DIFFthreshold && DIFF6 < DIFFthreshold){
							if(DIFF2 < DIFFthreshold && DIFF3 < DIFFthreshold && DIFF4 < DIFFthreshold && DIFF5 < DIFFthreshold){
								successionMasure++;
							}else{
								successionMasure = 0;							
							}

							if(successionMasure > 0){
							//if(DIFF2 < DIFFthreshold && DIFF3 < DIFFthreshold && DIFF4 < DIFFthreshold && DIFF2 != 0  && DIFF3 != 0 && DIFF4 != 0){
								Point cicleCenter = trajectory_Poins[i][j]*(1/(imageScaleBackgroundSUBTRACT*0.5));
								float scaledScale = trajectory_Poins_scale[j]*1;

								//fix center half way current and previous frame
								cicleCenter = (trajectory_Poins[i][j]/2+trajectory_Poins[i][j-1]/2)*(1/(imageScaleBackgroundSUBTRACT*0.5));

								circle(plotFrame, cicleCenter, 22, CV_RGB(2*sizeM, 2*sizeM, 0), -1); //trajectory_Poins_scale

								rectangle(plotFrame, Point(cicleCenter.x - scaledScale/2,cicleCenter.y - scaledScale/2), 
								Point(cicleCenter.x + scaledScale/2,cicleCenter.y + scaledScale/2), CV_RGB(255, 0, 255), 12, 8, 0);

								circle(plotFrame, trajectory_Poins[i][j-1]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 15, CV_RGB(1*sizeM/11, 1*sizeM/99, 0), -1);
								circle(plotFrame, trajectory_Poins[i][j-2]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 8,  CV_RGB(0.5*sizeM/39, 0.5*sizeM/99, 0), -1);
								circle(plotFrame, trajectory_Poins[i][j-3]*(1/(imageScaleBackgroundSUBTRACT*0.5)), 4,  CV_RGB(0.2*sizeM/99, 0.2*sizeM/99, 0), -1);


								///// RESTART MOSSE
								if(1==1 && abs(bbox2.x - (cicleCenter.x - scaledScale/2)) > 80 || abs(bbox2.y - (cicleCenter.y - scaledScale/2)) > 80 ){
									bbox2.width = 2*scaledScale/2;
									bbox2.height = 2*scaledScale/2;
									//MAKE SQUARE
									if(bbox2.width > bbox2.height)  { bbox2.height = bbox2.width;  }
									if(bbox2.height > bbox2.width)  { bbox2.width  = bbox2.height; }
									if(bbox2.width > frame.cols/2) { bbox2.width  = frame.cols/4;}
									if(bbox2.height > frame.rows/2){ bbox2.height = frame.rows/4;}
									bbox2.width = bbox2.width * 1.1;
									bbox2.x = cicleCenter.x + 2*scaledScale/2;
									bbox2.y = cicleCenter.y + 2*scaledScale/2;							
									//if(okB == 0){
										tracker2 = trackerUsed::create();						
										tracker2->init(frame, bbox2);
										okB = tracker2->update(frame, bbox2);
									//}			
								}						
								////////// END RESTART MOSSE

							}						
						}
					}
				}
			}
			//trajectory_Poins.clear();
		}

		//END PLOT TRAJECTORIES



		frame.copyTo(prevFrame);


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
		// Display tracker type on frame
	        putText(plotFrame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
		 
		// Display FPS on frame
		// Calculate Frames per second (FPS)
		float fps = getTickFrequency() / ((double)getTickCount() - timer);
		putText(plotFrame, "FPS : " + SSTR(int(fps)) + " Frame:" + SSTR(int(counter+start_frame)) , Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
		putText(plotFrame, "MOSSE STATE: " + SSTR(okB) , Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
	 
		

		// Write the frame into the file 'outcpp.avi'
		if(writeVideo){

			if(!writeRAW){
				videoOut.write(plotFrame);
			}
			
			//if(exportSparseOpticalFlowVideos && !out3.empty() && out3.cols == frame_width && !fgMaskMOG2Plot.empty()){
			//	cout << "frame_width" << frame_width << ", " << frame_height << ", " << out3.cols << "," << out3.rows << endl;
			//					
			//	Mat im_coloured = Mat::zeros(out3.rows,out3.cols,CV_8UC3);				
			//	cvtColor(out3,im_coloured, CV_BGRA2BGR, 1);
//
			//	std::cout<< "out3.channels:" << im_coloured.channels() << endl;
			//	std::cout<< "fgMaskMOG2Plot.channels:" << fgMaskMOG2Plot.channels() << endl;			
//
			//	videoOutA.write(im_coloured);
			//	videoOutB.write(fgMaskMOG2Plot);
			//}

			
			if(exportBackSubtractVideos && !out3.empty() && out3.cols == frame_width && !fgMaskMOG2Plot.empty()){
				cout << "frame_width" << frame_width << ", " << frame_height << ", " << out3.cols << "," << out3.rows << endl;
				//Mat channels;
				//cvtColor(out3,channels, COLOR_GRAY2BGR, 1);
				//imshow("out3",channels);
				
				Mat im_coloured = Mat::zeros(out3.rows,out3.cols,CV_8UC3);

				//vector<Mat> planes;

				//for(int i=0;i<3;i++){
				//    planes.push_back(out3);
				//}
				//cv::merge(planes,im_coloured);
				//imshow("out3",im_coloured);

				//cv::Mat out4 = Mat::zeros(out3.rows,out3.cols,CV_8UC4);
				//cv::Mat in[] = {out3, out3, out3, out3};
				//cv::merge(in, 4, out4);
				//imshow("out3",out4);


			 	//vector<Mat> channels;
				//Mat fin_img;
			    	//Mat g = Mat::zeros(Size(out3.rows, out3.cols), CV_8UC1);

			    	//channels.push_back(g);
			    	//channels.push_back(g);
			    	//channels.push_back(out3);

			    	//merge(channels, fin_img);
			    	//imshow("img", fin_img);

				///cvCvtColor(out3, im_coloured, CV_BGRA2BGR);
				cvtColor(out3,im_coloured, CV_BGRA2BGR, 1);

				std::cout<< "out3.channels:" << im_coloured.channels() << endl;

				std::cout<< "fgMaskMOG2Plot.channels:" << fgMaskMOG2Plot.channels() << endl;

				
				if(!videoOutA.isOpened()){
					videoOutA.open("outcppBACKSUBRACT.avi",CV_FOURCC('M','J','P','G'),30, Size(im_coloured.cols,im_coloured.rows),1);
				}
				videoOutA.write(im_coloured);

				//Mat fgMaskMOG2PlotR;//=Mat::zeros(out3.rows,out3.cols,CV_8UC1);

				//resize(fgMaskMOG2Plot,fgMaskMOG2Plot, cv::Size(), out3.rows, out3.cols); 
				//fgMaskMOG2Plot.copyTo(im_coloured);

				if(!videoOutB.isOpened()){
					videoOutB.open("outcppBACKSUBRACT2.avi",CV_FOURCC('M','J','P','G'),30, Size(fgMaskMOG2Plot.cols,fgMaskMOG2Plot.rows),1);
				}
				videoOutB.write(fgMaskMOG2Plot);//resize(passGPU,passGPU, cv::Size(), scaling, scaling); 
			}
		}

		// Exit if ESC pressed.

//Do this.
		

		int k = waitKey(1);
		if(k == 27)
		{
		    break;
		}

		if(k == 84) //(T)oggle PID - test sin
		{

			if(enablePIDracking){
				enablePIDracking = false;
				cout << "Disabled PID ..." << endl;
			}else{
				enablePIDracking = true;
				cout << "Enabled PID ..." << endl;
			}
			//bool enablePIDracking = false;
			//bool enablePIDZoom = true;
		    //break;
		}
		
		if(kbhit() == 0){
			//return 0;
			outfile.close();
			videoOut.release();
			videoOutA.release();
			videoOutB.release();
			videoOutC.release();
			out_transform.close();
			out_trajectory.close();
			out_smoothed_trajectory.close();
			out_new_transform.close();
			break;		
		}
		

		//ROS
		//SSTR(int(counter+start_frame))
		int frameID = int(counter+start_frame);
		std_msgs::UInt32 array;//std_msgs::UInt32MultiArray array;
		array.data = frameID;
		//Clear array
		//array.data.clear();
		//array.data.push_back(frameID); //insert frame id
		//array.data.push_back(bbox2.x);
		//array.data.push_back(bbox2.y);
		//array.data.push_back(bbox2.width);
		//array.data.push_back(bbox2.height);
		//add mosse (white), measurement (pink), Kalman (red), Inner Mosse (blue) - center x,y, width, height

		//for loop, pushing data in the size of the array
		//for (int i = 0; i < 99; i++)
		//{
			//assign array a random number between 0 and 255.
			//array.data.push_back(rand() % 255);
		//	array.data.push_back(i);
		//}		
		//cout << "------" << endl;
		//Publish array
		pub.publish(array);
		//Let the world know
		//ROS_INFO("I published something!");
		//Do this.
		//ros::spinOnce();
		//Added a delay so not to spam
		//sleep(2);
		//END ROS

		//LOG FILE
		//rectangle(plotFrame, bbox2, Scalar( 255, 255, 255 ), 2, 1 );		
		if(logData && trackFrameEstimateBoxes.size() > 0 && predRects.size() > 0 && predRects_Kalman_Window.size() > 0){
			//WRITE DATA
			//outfile << "Video Frame ID" << ";"//outfile << "time" << ";" << "Video Frame ID" << ";"
			//	<< "MOSSE Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
			//	<< "Measure Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"				
			//	<< "Kalman Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
			//	<< "Inner FLOW Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";"
			//	<< "Inner MOSSE Center X" << ";" << "Y" << ";" << "Width" << ";" << "Height" << ";" //relative to Kalman window above !!!
			//	<< "Euler Sensor X" << ";" << "Y" << ";" << "Z" << ";"
			//	<< "Pan" << ";" << "Tilt" << ";" << "Zoom" << ";"
			//	<< std::endl;		
			outfile << frameID << ";"
				<< bbox2.x << ";" << bbox2.y << ";" << bbox2.width << ";" << bbox2.height << ";"
				<< trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].x << ";" << trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].y << ";" 
				<< trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].width << ";" << trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].height << ";"
				<< predRects[predRects.size()-1].x << ";" << predRects[predRects.size()-1].y << ";" 
				<< predRects[predRects.size()-1].width << ";" << predRects[predRects.size()-1].height << ";"
				<< predRects_Kalman_Window[predRects_Kalman_Window.size()-1].x << ";" << predRects_Kalman_Window[predRects_Kalman_Window.size()-1].y << ";" 
				<< predRects_Kalman_Window[predRects_Kalman_Window.size()-1].width << ";" << predRects_Kalman_Window[predRects_Kalman_Window.size()-1].height << ";"
				<< bboxKALMAN.x << ";" << bboxKALMAN.y << ";" << bboxKALMAN.width << ";" << bboxKALMAN.height << ";"	//bboxKALMAN
				<< Eulers[0] << ";" << Eulers[1] << ";" << Eulers[2] << ";"
				//<< currentPan << ";" << currentTilt << ";" << currentZoom << ";"
				<< currentPan << ";" << currentTilt << ";" << currentZoom << ";"
				<< YawRate << ";" << pitchRate << ";" << rollRate << ";"
				<< accelXRate << ";" << accelYRate << ";" << accelZRate << ";"
				<< panRate << ";" << tiltRate << ";" << SSTR(okB) << ";"
				<< "Sensor: " + SSTR(useSparkfun) + ", PT:"+ SSTR(enableTestPanTilt)+ ", Z:"+ SSTR(enableTestZoom)+ ", PID:"+ SSTR(enablePIDracking)+ ", PIDZ:"+ SSTR(enablePIDZoom) << ";"
				<< std::endl;

			//DEBUG DATA
			rectangle(plotFrame, bbox2.tl(), bbox2.br()*1.004, CV_RGB(11, 11, 11), 2, 1, 0);
			rectangle(plotFrame, trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].tl(), trackFrameEstimateBoxes[trackFrameEstimateBoxes.size()-1].br()*1.004, CV_RGB(11, 11, 11), 2, 1, 0);
			rectangle(plotFrame, predRects[predRects.size()-1].tl(), predRects[predRects.size()-1].br()*1.004, CV_RGB(11, 11, 11), 2, 1, 0);
			rectangle(plotFrame, predRects_Kalman_Window[predRects_Kalman_Window.size()-1].tl(), predRects_Kalman_Window[predRects_Kalman_Window.size()-1].br()*1.004, CV_RGB(11, 11, 11), 2, 1, 0);
		}
		//END LOG FILE

		// Display frame.
		//cout << "FPS : " << SSTR(int(fps)) << " Frame:" <<  SSTR(int(counter+start_frame)) <<endl;
		imshow("Tracking", plotFrame);

	}//frame check 

		//ros::spinOnce();
		//ros::spin();
		//Added a delay so not to spam
		//sleep(1);


    }
}

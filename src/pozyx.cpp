//#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include <string> 
#include <math.h>
#include <fstream>
#include <sstream>

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

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
}

//SPAKFUN
bool useSparkfun = false; //use Sparkfun 9DOF 14001 Razor sensor

//ARDUINO COM READ (global variables)
int fd, n, i;
char buf[256] = "temp text";
std::vector<double> pozyxOUTPUT;
int POZYXsamplesCounted = 0;
//sample counter

int usePOZYX = 1;

//TIMER
int totalIterationsTimer=0;
struct timeval beginT, endT;
double REAL_seconds_since_start=0;
double REAL_PREV_seconds_since_start=0;
int REAL_PREV_POZYX_samples = 0;
//TIMER2
struct timeval beginT2, endT2;
double REAL_seconds_since_start2=0;

int totalIterations = 0;
//sample counter

float PI = 3.141592653589793238462643383279502884;

//GET DATA
void getPozyxData(){
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
				int counterStart = REAL_PREV_POZYX_samples;						
				int samplePOZYXDiffCurrentPrevArucoIterationROUNDED  = pozyxOUTPUT.size() - counterStart;				
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
			
					POZYX_OUTPUT_X = POZYX_OUTPUT_X / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_Y = POZYX_OUTPUT_Y / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_Z = POZYX_OUTPUT_Z / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_ROLL = POZYX_OUTPUT_ROLL / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_PITCH = POZYX_OUTPUT_PITCH / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
					POZYX_OUTPUT_YAW = POZYX_OUTPUT_YAW / samplePOZYXDiffCurrentPrevArucoIterationROUNDED;
				}else{					
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
				
				cout << "POZYX Full Mean Yaw: " << POZYX_OUTPUT_YAW << " Roll:" << POZYX_OUTPUT_ROLL << " Pitch:" << POZYX_OUTPUT_PITCH << endl;				
				cout << "POZYX Full Mean X:" << POZYX_OUTPUT_X << " Y:" << POZYX_OUTPUT_Y << " Z:" << POZYX_OUTPUT_Z << endl;				

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

			float pitch = POZYX_OUTPUT_PITCH* PI / 180.0; //on local POZYX TAG X axis
			float roll = POZYX_OUTPUT_ROLL* PI / 180.0; //on local POZYX TAG Y axis
			float yaw = POZYX_OUTPUT_YAW* PI / 180.0; //on local POZYX TAG Z axis

			float DistPOZYXAxisToGround = 0.04f;
			float distanceMEAN_tX_POZYX = ((POZYX_OUTPUT_X)/1000.0);
			float distanceMEAN_tY_POZYX = ((POZYX_OUTPUT_Y)/1000.0);
			float distanceMEAN_tZ_POZYX = ((DistPOZYXAxisToGround-POZYX_OUTPUT_Z)/1000.0);

	}//END if use POZYX ==1
}

// POZYX THREAD
void initComPOZYX(){
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
		cfsetispeed(&toptions, B115200);
		cfsetospeed(&toptions, B115200);
		//cfsetispeed(&toptions, B500000);
		//cfsetospeed(&toptions, B500000);
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
void task1(string msg)
{	
	if(usePOZYX==0){
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
		std::vector<int> vectMEASUREMENTS;
		std::stringstream ss(buf);
		int icount;
		while (ss >> icount)
		{
			vectMEASUREMENTS.push_back(icount);
			//cout << "icount = " << icount << endl;
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
		cout << vectMEASUREMENTS.size() << endl;
		for (i=0; i< vectMEASUREMENTS.size(); i++){
			if(i == rot1 || i==rot2 || i==rot3){				
				float angle = ((int)(vectMEASUREMENTS.at(i))/16) % 360; 
				int id = 3;				
				if(i == rot1){					
					id = 1;
				}
				if(i == rot2){
					angle = -angle;
					float angleNext = ((int)(vectMEASUREMENTS.at(i+1))/16) % 360; 					
					id = 2;
				}				
				pozyxOUTPUT.push_back(angle);
				cout << "angle " << to_string(id) << " = " << angle << endl;					
			}
			//print POZYX corrds
			if(i == pos1 || i==pos2 || i==pos3){				
				pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			}			
		}

		cout << endl;
		for (i=0; i< vectMEASUREMENTS.size(); i++){
			if(i == rot1 || i==rot2 || i==rot3){				
				//float angle = vectMEASUREMENTS.at(i); 
				//std::cout << angle <<std::endl;
			}
		}					
		//END ARDUINO COM READ

		if(vectMEASUREMENTS.size() > 0){
			POZYXsamplesCounted++;
		}		
	} 	
}

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
void task2(string msg)
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

		cout << "vectMEASUREMENTS size = " << vectMEASUREMENTS.size() << endl;
		for (i=0; i< vectMEASUREMENTS.size(); i++){
			//cout << "vectMEASUREMENTS.at  " << i << " =" << vectMEASUREMENTS.at(i) << endl;			
			//if(i == time||i == pos1 || i==pos2 || i==pos3){				
				//pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			//}
			//if(i == rot1 || i==rot3 || i==rot3){				
				pozyxOUTPUT.push_back(vectMEASUREMENTS.at(i));
			//}			
		}

		cout << endl;
		for (i=0; i< pozyxOUTPUT.size(); i++){
			//if(i == rot1 || i==rot2 || i==rot3){				
				//float angle = vectMEASUREMENTS.at(i); 
			if(i == pozyxOUTPUT.size() - 7){
				cout << endl;std::cout << "Time:" << endl;
			}
			if(i == pozyxOUTPUT.size() - 6){
				cout << endl;std::cout << "Acceleration X,Y,Z:" << endl;
			}
			if(i == pozyxOUTPUT.size() - 3){
				cout << endl;std::cout << "Roll, pitch, Yaw:" << endl;
			}
			if(i > pozyxOUTPUT.size() - 8){		
				std::cout << pozyxOUTPUT[i] <<std::endl;
			}
				
			//}
		}					
		//END ARDUINO COM READ

		std::cout << "______________" << endl;

		if(vectMEASUREMENTS.size() > 0){
			POZYXsamplesCounted++;
		}		
	} 	
}

//MAIN
int main(int argc, char **argv)
{
	usePOZYX = 0;
	if(usePOZYX==1){
		initComPOZYX();
	}

	useSparkfun = true;//true; false
	if(useSparkfun){
		initComSparkFun();
	}
	
	//if(usePOZYX==1){
		thread t1(task1, "Hello");
	//}
	//if(useSparkfun){
		thread t2(task2, "Hello");
	//}	

	while(2>1){
		if(usePOZYX==1){
			//cout << "buf1 = " << buf << endl;
			thread t1(task1, "Hello"); t1.detach();sleep(1);
		}
		if(useSparkfun){
			//cout << "buf1 = " << buf << endl;
			//thread t2(task2, "Hello2"); t2.detach();sleep(1);
		}
	}
}

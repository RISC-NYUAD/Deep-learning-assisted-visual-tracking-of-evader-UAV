//#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace aruco;

double PI = 3.141592653589793;

typedef struct MarkerOrientation{ // struct to return x,y angles
	double x_rot;
	double y_rot;
}XY_rot;

typedef struct SeparatedMarkers{ // struct to return list of separated markers per rhombi and their rhombi identifier
	vector< vector< Marker > > markers_list;
	vector<int> flags;
}Markers_List;

XY_rot find_xy_orientation(int); // function that computes x,y angles from marker id

Markers_List separate_markers(vector<Marker>); // function that separates markers based on rhombi of origin


int main()
{
	VideoCapture cam;
	cam.open(0,CAP_V4L);
	Mat frame;
	
	vector< Mat > Final_Results;  // this is where we hold the 4x4 Homogeneous matrix of each rhombi
	Mat init = Mat::eye(4,4, CV_32F);
	for(int i = 0 ; i < 3 ; i++)
		Final_Results.push_back(init); // initialized as identity matrices
	
	MarkerDetector Mdetector;
	std::map<uint32_t, MarkerPoseTracker> Mtracker;
	Mdetector.setDictionary("ARUCO_MIP_36h12", 0.f);
	
	aruco::CameraParameters CamParam;
	float markerLength = 0.07 ;
	double focal_len = 476.7030836014194 ;	double cxy = 400.5 ;
	Mat camMatrix  = (Mat1d(3,3) <<  focal_len , 0.0 , cxy , 0.0 , focal_len , cxy , 0.0 , 0.0 , 1.0) ;
	Mat distCoeffs = (Mat1d(1,4) << 0.0 , 0.0 , 0.0 , 0.0) ; 
	Size camSize = Size(800, 800);

	CamParam.setParams(camMatrix, distCoeffs, camSize);

	
	while(cam.read(frame)){ // loop that reads the camera
		Mat input_image;
		input_image = frame;
		//input_image = imread("Aruco_2.png");
			
		vector<Marker> Markers = Mdetector.detect(input_image); // find all markers in frame
		if(Markers.size()>0){ // do nothing if no markers were found
			
			Markers_List marker_list = separate_markers(Markers); // separate them
			vector< vector<Marker> > rhombi_markers = marker_list.markers_list ; // vector of marker vectors, each element is a vector of markers of one rhombi
			vector<int> flags = marker_list.flags ; // vector of rhombi identifiers. flags[i] is the rhombi identity of rhombi_markers[i].
		
			CamParam.resize(input_image.size());
			for (int i = 0; i < flags.size() ; i++){ // for every different rhombi seen (shouldn't be more than two)
				
				Mat H = Final_Results[flags[i]]; // get the latest homogeneous matrix for this rhombi
				int count = 0; // counter of valid poses
				vector<Marker> rhombi_ = rhombi_markers[i] ; // get vector of detected markers of this rhombi 
				
				for (auto& marker : rhombi_ ){ // for every one of them
			    	Mtracker[marker.id].estimatePose(marker, CamParam, markerLength, 50 );  // call its tracker and estimate the pose
			    	if (CamParam.isValid() && markerLength != -1){
		        		if (marker.isPoseValid()){
		            		CvDrawingUtils::draw3dAxis(input_image, marker, CamParam); // this is just for drawing
		        		}
		        	}
	 
					if(marker.isPoseValid()){ // if you successfully posed the marker
						count++; 
						XY_rot xy_angles = find_xy_orientation(marker.id); // get the marker's x,y angles
						double x_angle, y_angle;
						x_angle = xy_angles.x_rot ;
						y_angle = xy_angles.y_rot ;
						Mat R,T;
						Rodrigues(marker.Rvec, R) ;
						T = marker.Tvec ;
						//Now we can compute the homogeneous transform and add it.
						//e.g. H = H + new_H ;
					}
				}
				//Now we can use the saved homogeneous transforms to do consensus
				//e.g.:
				if(count>0){ // this is necessary cause there may be detected markers but with no valid poses
					//H = scale(H,count);
					//Final_Results.at(flags[i]) = H;  // such that, Final_Results has 3 vectors, which contain the latest Homogeneous matrices of Rhombis 1, 2 and 3
				}
			}
			// Now we have in Final_Results, 3 homogeneous matrices (ordered as 1, 2, 3), so for each camera, one of those will forever remain identity
				
		
			for ( int i = 0 ; i < Markers.size() ; i++ ) {
				Markers[i].draw(input_image, Scalar(0, 0, 255), 2);			
			}
		}		        


  		imshow("frame",input_image);
	//	waitKey(0);
		char a = waitKey(10);
		if(a==27){
			break;
		}
	}
	
	return 0;


}


Markers_List separate_markers(vector<Marker> Markers){
	
	vector< vector<Marker> > rhombi_markers, temp;
	vector<Marker> rhombi_1, rhombi_2, rhombi_3;
	vector<int> counters(3, 0) ; // initialize a 3-element zero vector
	vector<int> flags;
	for(auto& marker : Markers){
		int num = marker.id;
		int index = floor(num/53.)+1;
		switch(index){
			case(1):
				counters.at(0)++;
				rhombi_1.push_back(marker);
				break;
			case(2):
				counters.at(1)++;
				rhombi_2.push_back(marker);
				break;
			case(3):
				counters.at(2)++;
				rhombi_3.push_back(marker);
				break;
		}
	}
	temp.push_back(rhombi_1);
	temp.push_back(rhombi_2);
	temp.push_back(rhombi_3); // temp is a holder, because it is certain that at least some of them are empty, we will only send forward non-empty ones
	for(int j = 0; j<3 ; j++){
		if(counters[j]>0){
			rhombi_markers.push_back(temp[j]);
			flags.push_back(j);
		}
	}
	// so now, rhombi_markers has vectors of markers of specific rhombicubes
	// and flags has integers specifying the rhombi of each rhombi_markers vector
	Markers_List list;
	list.markers_list = rhombi_markers;
	list.flags = flags;
	return list;
}



XY_rot find_xy_orientation(int num){

	double x_ang, y_ang;
	int index = floor(num/53.)+1; // index = 1 if marker belongs in rhombi-1, 2 if marker belongs in rhombi-2, 3 if marker belongs in rhombi-3
	switch(index){
		case(1):
			switch(num){
				case(33): //central bottom line (reference for all)
					x_ang = 0;
					y_ang = 0;
					break;
				case(24):
					x_ang = 0;
					y_ang = PI/4.;
					break;
				case(36):
					x_ang = 0;
					y_ang = PI/2.;
					break;
				case(3):
					x_ang = 0;
					y_ang = 3*PI/4.;
					break;
				case(6):
					x_ang = 0;
					y_ang = PI;
					break;
				case(9):
					x_ang = 0;
					y_ang = 5*PI/4.;
					break;
				case(0):
					x_ang = 0;
					y_ang = 6*PI/4.;
					break;
				case(30):
					x_ang = 0;
					y_ang = 7*PI/4.;
					break;
				case(27): //central middle line
					x_ang = -PI/4.;
					y_ang = 0;
					break;						
				case(48):
					x_ang = -PI/4.;
					y_ang = PI/4.;
					break;
				case(12):
					x_ang = -PI/4.;
					y_ang = PI/2.;
					break;
				case(39):
					x_ang = -PI/4.;
					y_ang = 3*PI/4.;
					break;
				case(18):
					x_ang = -PI/4.;
					y_ang = PI;
					break;
				case(42):
					x_ang = -PI/4.;
					y_ang = 5*PI/4.;
					break;
				case(21):
					x_ang = -PI/4.;
					y_ang = 6*PI/4.;
					break;
				case(45):
					x_ang = -PI/4.;
					y_ang = 7*PI/4.;
					break; 	
				case(15): //top one
					x_ang = -PI/2.;
					y_ang = 0;
					break;				
			}
			break;
		case(2):
			switch(num){
				case(76):
					x_ang = 0;
					y_ang = 0;
					break;
				case(79):
					x_ang = 0;
					y_ang = PI/4.;
					break;
				case(91):
					x_ang = 0;
					y_ang = PI/2.;
					break;
				case(55):
					x_ang = 0;
					y_ang = 3*PI/4.;
					break;
				case(85):
					x_ang = 0;
					y_ang = PI;
					break;
				case(67):
					x_ang = 0;
					y_ang = 5*PI/4.;
					break;
				case(64):
					x_ang = 0;
					y_ang = 6*PI/4.;
					break;
				case(73):
					x_ang = 0;
					y_ang = 7*PI/4.;
					break;
				case(82):
					x_ang = -PI/4.;
					y_ang = 0;
					break;
				case(97):
					x_ang = -PI/4.;
					y_ang = PI/4.;
					break;
				case(88):
					x_ang = -PI/4.;
					y_ang = PI/2.;
					break;
				case(100):
					x_ang = -PI/4.;
					y_ang = 3*PI/4.;
					break;
				case(58):
					x_ang = -PI/4.;
					y_ang = PI;
					break;
				case(94):
					x_ang = -PI/4.;
					y_ang = 5*PI/4.;
					break;
				case(61):
					x_ang = -PI/4.;
					y_ang = 6*PI/4.;
					break;
				case(103):
					x_ang = -PI/4.;
					y_ang = 7*PI/4.;
					break; 
				case(70):
					x_ang = -PI/2.;
					y_ang = 0;
					break;
			}
			break;
		case(3):
			switch(num){
				case(140):
					x_ang = 0;
					y_ang = 0;
					break;
				case(116):
					x_ang = 0;
					y_ang = PI/4.;
					break;
				case(119):
					x_ang = 0;
					y_ang = PI/2.;
					break;
				case(134):
					x_ang = 0;
					y_ang = 3*PI/4.;
					break;
				case(146):
					x_ang = 0;
					y_ang = PI;
					break;
				case(143):
					x_ang = 0;
					y_ang = 5*PI/4.;
					break;
				case(131):
					x_ang = 0;
					y_ang = 6*PI/4.;
					break;
				case(110):
					x_ang = 0;
					y_ang = 7*PI/4.;
					break;
				case(113):
					x_ang = -PI/4.;
					y_ang = 0;
					break;
				case(152):
					x_ang = -PI/4.;
					y_ang = PI/4.;
					break;
				case(122):
					x_ang = -PI/4.;
					y_ang = PI/2.;
					break;
				case(158):
					x_ang = -PI/4.;
					y_ang = 3*PI/4.;
					break;
				case(137):
					x_ang = -PI/4.;
					y_ang = PI;
					break;
				case(155):
					x_ang = -PI/4.;
					y_ang = 5*PI/4.;
					break;
				case(128):
					x_ang = -PI/4.;
					y_ang = 6*PI/4.;
					break;
				case(149):
					x_ang = -PI/4.;
					y_ang = 7*PI/4.;
					break;
				case(125):
					x_ang = -PI/2.;
					y_ang = 0;
					break;
			}
			break;
	}

	XY_rot xy ;
	xy.x_rot = x_ang;
	xy.y_rot = y_ang;
	return xy;		
}































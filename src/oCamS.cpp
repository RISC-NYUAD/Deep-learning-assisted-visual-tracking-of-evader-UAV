// see camera formats
// v4l2-ctl -d /dev/video0 --list-formats-ext /

//roscore

// cd ~/openCVmine/testROS/ocams/
//make all

// cd ~/openCVmine/testROS/ocams/devel/lib/ocams
//./ocams -cc=1 -ci=1 _exposure:=60

// cd ~/openCVmine/testROS/ocams/devel/lib/ocams
//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 -d=10 -d"detector_params.yml" -c="outCalib999.txt" -p=1 -af=0 -HorAngle=0 -VerAngle=0 -RealDist=1.01 -shape=0  -usePOZYX=0 -useLIDAR=0


#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include "withrobot_camera.hpp"

#include <opencv2/calib3d.hpp> //add Rodrigues transformation
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>

//OCAM
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


namespace Withrobot {





class StereoCamera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

	

public:

	/**
	 * @brief      { stereo camera driver }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
    StereoCamera(int resolution, double frame_rate): camera(NULL) {

        enum_dev_list();

	devPath_ = "/dev/video0";  ///oCAM-1MGN-U

        camera = new Withrobot::Camera(devPath_.c_str());

        if (resolution == 0) { width_ = 2160; height_ = 2160;}
        if (resolution == 1) { width_ = 2160; height_ = 2160;}
        if (resolution == 2) { width_ = 640; height_  = 480;}

width_ = 3840; height_ = 2160;

        //camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, (unsigned int)frame_rate); //oCAM-1MGN-U

	//camera->set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 30); //oCAM-1MGN-U
	//camera->set_format(1920, 1080, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 30); //oCAM-1MGN-U
	camera->set_format(3840, 2160, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 45); //oCAM-1MGN-U
	//camera->set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 60); //oCAM-1MGN-U  //if i put 45fps, still works with the 60fps of ROS !!!!! 
	//camera->set_format(640, 480, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 80); //oCAM-1MGN-U
	//camera->set_format(320, 240, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 160); //oCAM-1MGN-U  //115-130fps in practice (showing image in consumer), tiny bit faster when not showing image

	//OCAM FPS
	//Frame Rate : 45fps@1280x960, 60fps@1280x720, 80fps@640x480, 160fps@320x240
	// BASIC !!! - //if i put 45fps, still works with the 60fps !!!!! ALSO if above 60fps, must also declare this in ROS framerate variable below !!!!! or defaults to 60fps when 
	// going out of the camera image server

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);

        camFormat.print();

        /* Withrobot camera start */
        camera->start();
	}

	~StereoCamera() {
        camera->stop();
        delete camera;

	}

    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) {
            if (dev_list[i].product == "oCamS-1CGN-U")
            {
                devPath_ = "/dev/video0"; //dev_list[i].dev_node; //oCAM-1MGN-U
                return;
            }

        }

    }

    void uvc_control(int exposure, int gain, int blue, int red, bool ae)
    {
        /* Exposure Setting */
        camera->set_control("Exposure (Absolute)", exposure);

        /* Gain Setting */
        camera->set_control("Gain", gain);

        /* White Balance Setting */
        camera->set_control("White Balance Blue Component", blue);
        camera->set_control("White Balance Red Component", red);

        /* Auto Exposure Setting */
        camera->set_control("Exposure, Auto", !ae);

    }

	/**
	 * @brief      Gets the images.
	 *
	 * @param      left_image   The left image
	 * @param      right_image  The right image
	 *
	 * @return     The images.
	 */
	bool getImages(cv::Mat& left_image, cv::Mat& right_image) {

		cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1); //cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC2);  //oCAM-1MGN-U

		//oCAM-1MGN-U
		int size = camera->get_frame(srcImg.data, camFormat.image_size, 1);

		/* If the error occured, restart the camera. */
		if (size == -1) {
			printf("error number: %d\n", errno);
			perror("Cannot get image from camera");
			camera->stop();
			usleep(35);
			camera->start();
 			return false;
			//continue;
		}else{
			//right_image = srcImg;
			left_image = srcImg;
			return true;
		}	

		if(1==0){
			cv::Mat dstImg[2];
			if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1) {
				cv::split(srcImg, dstImg);
				right_image = dstImg[0];
				left_image = dstImg[1];
				return true;
			} else {
				return false;
			}
		}
	}

private:
    int width_;
    int height_;
    std::string devPath_;

};

/**
 * @brief       the camera ros warpper class
 */
class oCamStereoROS {
public:


	//TIMER
//time_t endingTime;
clock_t startTime = clock();
clock_t endingTime;
int totalIterationsTimer=0;
int currentFrameID=0;
struct timeval beginT, endT;

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
    oCamStereoROS(ros::NodeHandle nh, ros::NodeHandle priv_nh, VideoCapture inputVideo, cv::Mat srcImg1, int colorCamera) {

        /* default parameters */
        resolution_ = 1;
        frame_rate_ = 50;///50.0;
        exposure_ = 100;
        gain_ = 50;
        wb_blue_ = 200;
        wb_red_ = 160;
        autoexposure_= false;
        left_frame_id_ = "left_camera";
        right_frame_id_ = "right_camera";
        show_image_ = true;

//printf("----------------- Current format informations 001 -----------------\n");

        /* get parameters */
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("frame_rate", frame_rate_);
        priv_nh.getParam("exposure", exposure_);
        priv_nh.getParam("gain", gain_);
        priv_nh.getParam("wb_blue", wb_blue_);
        priv_nh.getParam("wb_red", wb_red_);
        priv_nh.getParam("left_frame_id", left_frame_id_);
        priv_nh.getParam("right_frame_id", right_frame_id_);
        priv_nh.getParam("show_image", show_image_);
        priv_nh.getParam("auto_exposure", autoexposure_);

//printf("----------------- Current format informations 002 -----------------\n");

        /* initialize the camera */
        ocams = new StereoCamera(resolution_, frame_rate_);

//printf("----------------- Current format informations 003 -----------------\n");

        ocams->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_); //Use both 70 and 270 to cover for all brightness in all sides
        ROS_INFO("Initialized the camera");

        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
        image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

        ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
        ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

        sensor_msgs::CameraInfo left_info, right_info;

        ROS_INFO("Loading from ROS calibration files");

        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);

        info_manager.setCameraName("left");
        info_manager.loadCameraInfo( "package://ocams/config/left.yaml");
        left_info = info_manager.getCameraInfo();

        info_manager.setCameraName("right");
        info_manager.loadCameraInfo( "package://ocams/config/right.yaml");
        right_info = info_manager.getCameraInfo();

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;

        std::cout << left_info << std::endl;
        std::cout << right_info << std::endl;

        ROS_INFO("Got camera calibration files");

        // loop to publish images;
        cv::Mat left_image, right_image;
        ros::Rate r(frame_rate_);

	int timer=0;
	//ros::Rate loop_rate(5);


	//TIMER
	gettimeofday(&beginT, NULL);

	

        while (ros::ok()) {     // dave
            ros::Time now = ros::Time::now();


		
		//int colorCamera = 1;
		//COLOR CAMERA
		if(colorCamera == 1){
				//inputVideo.open(1);
				//ROS_INFO("INSIDE 1");
				if(inputVideo.grab()){
					//inputVideo.grab();
					//Mat image;
					
 					//ROS_INFO("INSIDE 11");
				//	//char key = cv::waitKey(10); // BASIC - MUST BE USED TO ENABLE IMSHOW PREVIEW without getting stuck, needs the delay

					inputVideo.retrieve(srcImg1);
					currentFrameID++;

					//


 	//if(!inputVideo.read(srcImg1)) {
         //   std::cout << "No frame" << std::endl;
        //    cv::waitKey();
        //}
					
					

					Mat imageCopy;
					srcImg1.copyTo(imageCopy);
					if (imageCopy.empty()){
						//return 0;
					}else{				 		
						//imshow("out", imageCopy);
						left_image = imageCopy;
						//right_image = imageCopy;
						//resize(left_image, left_image, Size(1920, 1080), 0, 0, INTER_CUBIC);

						

					    //TIMER
					    bool FPS_measure=0;
					    if(FPS_measure == 1) {
						totalIterationsTimer++;
						//if(totalIterationsTimer % 20 == 0) {
						if(totalIterationsTimer == 30) {

							cout << "Width " << srcImg1.cols << endl;
							//TIMER
							double fps1 = inputVideo.get(CV_CAP_PROP_FPS);
							cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps1 << endl;

						 	endingTime = clock(); //time(&endingTime);
							double seconds = double(endingTime - startTime) / CLOCKS_PER_SEC; //difftime( endingTime, start);
							startTime = endingTime;
							cout.precision(5);
							cout << "Time taken : " << seconds << " seconds" << " with iterations:" << totalIterationsTimer << endl;     
						    	// Calculate frames per second
						    	double fps  = totalIterationsTimer / seconds;
						    	cout << "Estimated frames per second : " << fps << endl;
						


							//TIMER ACCURATE
							gettimeofday(&endT, NULL);						
							double elapsed = (endT.tv_sec - beginT.tv_sec) + 
		      					((endT.tv_usec - beginT.tv_usec)/1000000.0);
							beginT=endT;
							cout << "Precision Time taken : " << elapsed << " seconds" << " with iterations:" << totalIterationsTimer << endl;  
							double fps2  = totalIterationsTimer / elapsed;
						    	cout << "Estimated Precision frames per second : " << fps2 << endl;


							totalIterationsTimer = 0;
						}
					     }//END FPS CALCS
					}


					//arucoProcess(srcImg,ExposureAbsolute);
					//return true;
					//printf("----------------- Current format informations AAAAAAAAAAAAAAAAAAAAAAAAAAA -----------------\n");
				}
		}

		if(colorCamera == 0){
			    if (!ocams->getImages(left_image, right_image)) {
				usleep(10);
			//ocams->stop();
			//usleep(15);
			//ocams->start();
				cout << "Sleep 10s " << endl;
				continue;
			    } else {
				ROS_INFO_ONCE("Success, found camera");
			    }
		}

			timer++;
			
			//if (show_image_) {
				//publishImage(left_image, left_image_pub, "left_frame", now);
				//publishCamInfo(left_cam_info_pub, left_info, now);
				//cv::imshow("left", left_image);
				//cv::imshow("right", right_image);
				//cv::waitKey(10);
			//}

			if (left_image_pub.getNumSubscribers() > 0) {
				//publishImage(left_image, left_image_pub, "left_frame", now, colorCamera);
				//printf("----------------- Left Image subscriber at TIME = %d", timer );
				publishImage(left_image, left_image_pub, std::to_string(currentFrameID), now, colorCamera);
			}
			//if (right_image_pub.getNumSubscribers() > 0) {
				//publishImage(right_image, right_image_pub, "right_frame", now, colorCamera);
				//printf("----------------- Right Image subscriber at TIME = %d", timer );
			//}
			if (left_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(left_cam_info_pub, left_info, now);
				//printf("----------------- Left Image Info at TIME = %d", timer );
			}
			if (right_cam_info_pub.getNumSubscribers() > 0) {
				publishCamInfo(right_cam_info_pub, right_info, now);
				//printf("----------------- Right Image Info at TIME = %d", timer );
			}

            r.sleep();
		
	    //ros::spinOnce();
	    //loop_rate.sleep();

	    // since the frame rate was set inside the camera, no need to do a ros sleep
        }
    }

    ~oCamStereoROS() 
    {
        delete ocams;
    }

	/**
	 * @brief      { publish camera info }
	 *
	 * @param[in]  pub_cam_info  The pub camera information
	 * @param[in]  cam_info_msg  The camera information message
	 * @param[in]  now           The now
	 */
	void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now) {
		cam_info_msg.header.stamp = now;
		pub_cam_info.publish(cam_info_msg);
	}

	/**
	 * @brief      { publish image }
	 *
	 * @param[in]  img           The image
	 * @param      img_pub       The image pub
	 * @param[in]  img_frame_id  The image frame identifier
	 * @param[in]  t             { parameter_description }
	 */
	cv_bridge::CvImage cv_image;
	void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t, int colorCamera) {
		
		cv_image.image = img;

		if(colorCamera==0){
			cv_image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
		}else{
			cv_image.encoding = sensor_msgs::image_encodings::BGR8;
		}


		cv_image.header.frame_id = img_frame_id;
		cv_image.header.stamp = t;
		img_pub.publish(cv_image.toImageMsg());
	}

private:
	int resolution_;
	double frame_rate_;
	int exposure_, gain_, wb_blue_, wb_red_;
	bool autoexposure_;
	bool show_image_;
	bool config_changed_;

	std::string left_frame_id_, right_frame_id_;

	StereoCamera* ocams;

};

}



	VideoCapture inputVideo;
	cv::Mat srcImg1;
	int waitTime;
	int colorCamera=0;
	const char* keys  = 
			"{cc |       | Chose color or grey camera (oCAM) }"
			"{ci | 0     | Camera id if input doesnt come from video (-v) }";

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "ocams");

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh("~");

	//printf("----------------- Current format informations 0 -----------------\n");

	cv::String video;


	//video = "./TEST.mp4"; //"http://devimages.apple.com/iphone/samples/bipbop/bipbopall.m3u8?dummy=param.mjpg"
	//video = "/home/nasos/openCVmine/TEST.mp4";
	//video = "/media/nasos/8765-4321/DCIM/100MEDIA/DJI_0037.MP4"; //use lsblk to find !!!
 
	//video = "http://wmccpinetop.axiscam.net/mjpg/video.mjpg";
	//video = "http://10.225.0.196:8080/";
	//video = "http://10.225.0.231:8080/";
	//video = "http://10.225.1.182:8080/";
	//video = "http://10.225.0.224:8080/";
	//  video = "http://10.225.0.106:8080/";
	//video = "http://10.225.0.196:11006";

//	video = "http://10.42.0.134:8080/";

	//video = "/home/nasos/openCVmine/testROS/4D78A8AC7D3411B50324D9916EA2A628.MOV";
	//video = "/home/nasos/openCVmine/testROS/416D681FF38E0537FBB73EA3641574B0.MOV";
	//video = "/home/nasos/openCVmine/testROS/A70A50D6F0DCDD196A615C988951C303.MOV";

	//video = "/home/nasos/openCVmine/testROS/20190205_163552.mp4";
	//video = "/home/nasos/openCVmine/testROS/20190205_163525.mp4";
	//video = "/home/nasos/openCVmine/testROS/20190205_164306.mp4";
	//video = "/home/nasos/openCVmine/testROS/20190205_165500.mp4";

	//video = "/home/nasos/openCVmine/testROS/20190205_170005.mp4";
	//video = "/home/nasos/openCVmine/testROS/2.mp4";
	//video = "/home/nasos/openCVmine/testROS/1.avi";

	//video = "/home/nasos/openCVmine/testROS/a1.MOV";
	//video = "/home/nasos/openCVmine/testROS/Feb13_StaticMavic.mov";
	//video = "/home/nasos/openCVmine/testROS/Feb14_52cmRight137cmHeight1945cmDepth.mov";
	//video = "/home/nasos/openCVmine/testROS/Feb14.mov";
//video = "/home/nasos/openCVmine/testROS/Feb13_StaticMavic1.mov";
//video = "/home/nasos/openCVmine/testROS/Feb26_MavicHover_1m.mov";
//video = "/home/nasos/openCVmine/testROS/HoverMavic1m_v2.mov";
//video = "/home/nasos/openCVmine/testROS/MovingMavic_Feb26.mov";
//video = "/home/nasos/openCVmine/testROS/BothMoving.mov";
//video = "/home/nasos/openCVmine/testROS/AllMoving2.mov";
//video = "http://10.42.0.96:8888/";

//how to find youtube live stream
//https://ytdl-org.github.io/youtube-dl/download.html
//youtube-dl --list-formats https://www.youtube.com/watch?v=TPm9jiFBvXw
//youtube-dl -f 95 -g https://www.youtube.com/watch?v=TPm9jiFBvXw
//video = "https://manifest.googlevideo.com/api/manifest/hls_playlist/id/TPm9jiFBvXw.0/itag/95/source/yt_live_broadcast/requiressl/yes/ratebypass/yes/live/1/cmbypass/yes/goi/160/sgoap/gir%3Dyes%3Bitag%3D140/sgovp/gir%3Dyes%3Bitag%3D136/hls_chunk_host/r1---sn-4wg7ln7l.googlevideo.com/ei/Fdy-XN2OAsTYVMKEhagM/gcr/ZZ/playlist_type/DVR/initcwndbps/21310/mm/32/mn/sn-4wg7ln7l/ms/lv/mv/m/pl/24/dover/11/keepalive/yes/mt/1556011945/disable_polymer/true/ip/91.230.41.204/ipbits/0/expire/1556033653/sparams/ip,ipbits,expire,id,itag,source,requiressl,ratebypass,live,cmbypass,goi,sgoap,sgovp,hls_chunk_host,ei,gcr,playlist_type,initcwndbps,mm,mn,ms,mv,pl/signature/856705E0C5D35CA07CEABDC2EF8526404CF090F0.5EE7F9C11347EAEA06666E3A3D9A8EEB660C467E/key/dg_yt0/playlist/index.m3u8";



	int colorCamera = 0;

	CommandLineParser parser(argc, argv, keys);
	colorCamera = parser.get<int>("cc");

	//VideoCapture inputVideo;
	//inputVideo.set(CV_CAP_PROP_BUFFERSIZE, 3); // internal buffer will now store only 3 frames



	//CHANGE HERE
	int camId = 0;

	//COLOR CAMERA INIT
	if(colorCamera == 1){
		//int waitTime;
		if(!video.empty()) {
			inputVideo.open(video);

			//inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 3840);//CHANGE HERE
			//inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 2160);
			//inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//CHANGE HERE
			//inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

			//inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 3840);//CHANGE HERE
			//inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 2160);
			//waitTime = 0;
		} else {			
			inputVideo.open(camId); //inputVideo.open(camId);			
			//waitTime = 10;
			inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);//CHANGE HERE
			inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
			
		}
	}	

	//ROS_INFO("INSIDE 0");
	//colorCamera=0;

        Withrobot::oCamStereoROS ocams_ros(nh, priv_nh, inputVideo, srcImg1, colorCamera);

//printf("----------------- Current format informations 1 -----------------\n");

        ros::spin();



	



        return 0;
    }

    catch(std::runtime_error& e) {
        ros::shutdown();
        return 1;
    }

}

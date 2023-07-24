/////////////========================== 360 CAMERA VIEW HANDLER GPU (INIT REGION) =================================================/////////////

//GPU speed test
//sudo intel_gpu_top -o gpustats.txt -s 1000

//COMMAND TO BUILD:
//g++ -g -std=c++11 -I/usr/local/include/opencv -I/usr/local/include/opencv2 -I. main.cpp -lSOIL -lglut -lglfw -lGLU -lGL -lGLEW -lsfml-graphics -lsfml-window -lsfml-system -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

//////////////// g++ -g -std=c++11 -I/usr/local/include/opencv -I/usr/local/include/opencv2 -I. main.cpp -lSOIL -lGL -lGLEW -lsfml-window -lopencv_core -lsfml-system -lopencv_imgproc -lopencv_highgui

// Link statically with GLEW
#define GLEW_STATIC

// Headers
#include <GL/glew.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "SOIL.h"
#include <SFML/Window.hpp>
//#include <chrono>
#include <iostream>
#include <termios.h>
#define STDIN_FILENO 0
//

//OPENCV
#include <opencv2/calib3d.hpp> //add Rodrigues transformation
//#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"

using namespace cv; //this must go AFTER libs declaration above
using namespace cv::ppf_match_3d;
using namespace std;

float c_pi = 3.14159265358979323846264;

// Shader sources
const GLchar* sceneVertexSource = R"glsl(
    #version 150 core
    in vec3 position;
    in vec3 color;
    in vec2 texcoord;
    out vec3 Color;
    out vec2 Texcoord;
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 proj;
    uniform vec3 overrideColor;
    void main()
    {
        Color = overrideColor * color;
        Texcoord = texcoord;
        //gl_Position = vec4(position, 1.0);
	gl_Position = proj * view * model * vec4(position, 1.0);
    }
)glsl";

const GLchar* sceneFragmentSource = R"glsl(
    #version 150 core    
    out vec4 outColor;   

    void main()
    {        
	outColor = vec4(1,0,0,1); //texture(texKitten, vec2(Texcoord.x, Texcoord.y));
    }
)glsl";

const GLchar* screenVertexSource = R"glsl(
    #version 150 core
    in vec2 position;
    in vec2 texcoord;
    out vec2 Texcoord;
    void main()
    {
        Texcoord = 1-texcoord;
        gl_Position = vec4(position, 0.0, 1.0);
    }
)glsl";

const GLchar* screenFragmentSource = R"glsl(
    #version 150 core
    in vec2 Texcoord;    

    //v0.3
    //layout (location = 1) out vec4 outColor;
    //layout (location = 0) out vec4 outColorBW;
    out vec4 outColor;   
    uniform vec3 rotY;
    uniform vec4 scaleTranslate;
    uniform sampler2D texFramebuffer;

    //v0.2
    uniform float time;   
    uniform float choosePass;
    uniform sampler2D texFramebufferPrev; //v0.1
    uniform sampler2D texFramebufferPrev2; //v0.2

float c_pi = 3.14159265358979323846264;
float myAtan2(float a, float b)
{
    float atan2val=0;
    if (b > 0) {
        atan2val = atan(a/b);
    }
    else if ((b < 0) && (a >= 0)) {
        atan2val = atan(a/b) + c_pi;
    }
    else if ((b < 0) && (a < 0)) {
        atan2val = atan(a/b) - c_pi;
    }
    else if ((b == 0) && (a > 0)) {
        atan2val = c_pi / 2;
    }
    else if ((b == 0) && (a < 0)) {
        atan2val = 0 - (c_pi / 2 );
    }
    else if ((b == 0) && (a == 0)) {
        atan2val = 1000;  //represents undefined
    }
    return atan2val;//atan2val;
}

    void main()
    {
	float threshold = 0.3;//0.1;
	vec4 outColorCurrent =  texture(texFramebuffer, vec2(1-Texcoord.x, 1-Texcoord.y));

	//PTZ, sample mask based on PAN shifing
	//vec2 offsetMask = vec2(1-Texcoord.x, 1-Texcoord.y);

	//Zoom is 0 to 16384
	//Pan is 0 at home. Right is positive, max 2448. Left ranges from full left 63088 to 65535 before home.
        //Tilt is 0 at home. Up is positive, max 1296. Down ranges from fully depressed at 65104 to 65535 before home.

	vec2 offsetMask = vec2(1-Texcoord.x - (0.0013*rotY.x*0), 1-Texcoord.y); 		//vec2 offsetMask = vec2(1-Texcoord.x + 40.5*rotY.x/3124, 1-Texcoord.y); 			
	vec4 outColorMask =  texture(texFramebufferPrev, offsetMask); //mask

	if(choosePass==0){		
		
		vec4 outColorPrev =  texture(texFramebufferPrev2, vec2(1-Texcoord.x, 1-Texcoord.y)); //previous frame

		//ZOOM FIELD - distance to zoom center of pixel
		vec2 invTexcoord = 1 - Texcoord;
		float distZoomX = rotY.z * (invTexcoord.x - 0.5) / 5000;
		float distZoomY = rotY.z * (invTexcoord.y - 0.5) / 5000;
		
		//MASK will start build confidence in pixels appear with high frequency and hold it in alpha channel, if confidence high, keep mask than current pixel !!! 1 = least conidence

		vec4 outfinal = outColorMask;
		float backLearnSpeed = 0.003 * 8;
		
		if(outColorMask.a > 1-threshold || outColorMask.a == 0){

			if(length(outColorCurrent.rgb - outColorMask.rgb) > threshold){ 
				
				if(length(outColorCurrent.rgb - outColorPrev.rgb) < threshold){ 
					outfinal = outColorCurrent;					
					outfinal.a = outfinal.a - backLearnSpeed;
				}else{
					if(outColorMask.a > (1-threshold) ){
						outfinal = vec4(0,0,0,1);
					}else{
						outfinal = outColorMask;	
					}
				}
			}else{
				outfinal = outColorMask;  outfinal.a = outfinal.a - backLearnSpeed;		
			}

			if(outColorMask.r == 0 && outColorMask.g == 0){
				if(length(outColorCurrent.rgb - outColorPrev.rgb) < threshold){						
					outfinal = outColorCurrent;						
					outfinal.a = outfinal.a - backLearnSpeed;
				}		
			}

		}else {
			outfinal = outColorMask;
			float diff = length(outColorCurrent.rgb - outColorMask.rgb);
			if(diff > 0){				
				outfinal.a = outfinal.a + 2.2*(diff - 0) * backLearnSpeed ;
				outfinal.rgb = mix(outfinal.rgb, outColorCurrent.rgb, 2.2*(diff - 0) * backLearnSpeed);
			}	
			if(outColorMask.r == 0 && outColorMask.g == 0){
				if(length(outColorCurrent.rgb - outColorPrev.rgb) < threshold){						
					outfinal = outColorCurrent;						
					outfinal.a = outfinal.a - backLearnSpeed;
				}		
			}			
		}

		//if black and previous and current same pixel, restore		
		outColor = outfinal;
	}else{		
		float diff = length(outColorCurrent.rgb - outColorMask.rgb);
		if(diff > threshold){
			outColor = vec4(1,1,1,1); 
		}else{
			outColor = vec4(0,0,0,1); 
		}
	}

    }//END MAIN

)glsl";

const GLchar* screenFragmentSourcePARAM = R"glsl(
    #version 150 core
    in vec2 Texcoord;
    out vec4 outColor;
    //in float rotY;
    uniform vec3 rotY;//v0.2
    uniform float time;   
    uniform sampler2D texFramebufferPrev; //v0.1
    uniform vec4 scaleTranslate;
    uniform sampler2D texFramebuffer;

    //v0.2
    uniform float time;   
    uniform sampler2D texFramebufferPrev; //v0.1

float c_pi = 3.14159265358979323846264;
float myAtan2(float a, float b)
{
    float atan2val=0;
    if (b > 0) {
        atan2val = atan(a/b);
    }
    else if ((b < 0) && (a >= 0)) {
        atan2val = atan(a/b) + c_pi;
    }
    else if ((b < 0) && (a < 0)) {
        atan2val = atan(a/b) - c_pi;
    }
    else if ((b == 0) && (a > 0)) {
        atan2val = c_pi / 2;
    }
    else if ((b == 0) && (a < 0)) {
        atan2val = 0 - (c_pi / 2 );
    }
    else if ((b == 0) && (a == 0)) {
        atan2val = 1000;  //represents undefined
    }
    return atan2val;//atan2val;
}

    void main()
    {
	vec2 u_translate = vec2(scaleTranslate.z,scaleTranslate.w);
		vec2 u_scale = vec2(scaleTranslate.x,scaleTranslate.y);
		vec2 u_rotate = vec2(rotY.x,rotY.y);

		//ROTATE
		vec2 TexcoordA = Texcoord;
		float sin_factor = sin(rotY.z);
		float cos_factor = cos(rotY.z);		
	     	vec2 coord = vec2((TexcoordA.x - 0.5) * (16 / 9), TexcoordA.y - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
		coord += 0.5;	   
		//END ROTATE	

		//float c_pi = 3.14159265358979323846264;
		float c_halfPi = c_pi * 0.5;
		float c_twoPi = c_pi * 2.0;
		float cosphi0 = cos(u_rotate.y);
		float sinphi0 = sin(u_rotate.y);
		float x = (TexcoordA.x - u_translate.x) / u_scale.x;
		float y = (u_translate.y - TexcoordA.y) / u_scale.y;
		// inverse stereographic projection
		float rho = sqrt(x * x + y * y);
		float c = 2.0 * atan(rho);
		float sinc = sin(c);
		float cosc = cos(c);
		float lambda = myAtan2(x * sinc, rho * cosc);
		float phi = asin(y * sinc / rho);
		// inverse rotation
		float cosphi = cos(phi);
		float x1 = cos(lambda) * cosphi;
		float y1 = sin(lambda) * cosphi;
		float z1 = y * sinc / rho;
		lambda = myAtan2(y1, x1 * cosphi0 + z1 * sinphi0) + u_rotate.x;	
		phi = asin(z1 * cosphi0 - x1 * sinphi0);

		vec2 uvsProjected = vec2((lambda + c_pi) / c_twoPi, (phi + c_halfPi) / c_pi);

		if (uvsProjected.x < 0) {
				uvsProjected.x = 1 + mod(uvsProjected.x,1) ;//1 + uvsProjected.x;
		}
		if (uvsProjected.x > 1) {
				//uvsProjected.x = uvsProjected.x - 1;
				uvsProjected.x = mod(uvsProjected.x,1);
		}
	

	//outColor =  vec4(Color, 1.0) * texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
	outColor =  12*texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
    }
)glsl";

// Cube vertices
GLfloat cubeVertices[]  = {
    -1.0f,  1.0f,  0.0f, 1.0f,//low left, y moves x left, x moves y down
     1.0f,  1.0f,  0.0f, 1.0f,//x no effect
     1.0f, -1.0f,  1.0f, 1.0f,
     1.0f, -1.0f,  1.0f, 1.0f   
};

// Quad vertices
GLfloat quadVertices[] = {
    -1.0f,  1.0f,  0.0f, 1.0f,
     1.0f,  1.0f,  1.0f, 1.0f,
     1.0f, -1.0f,  1.0f, 0.0f,
     1.0f, -1.0f,  1.0f, 0.0f,
    -1.0f, -1.0f,  0.0f, 0.0f,
    -1.0f,  1.0f,  0.0f, 1.0f
};

// Create a texture from an image file
GLuint loadTexture(const GLchar* path)
{
    GLuint texture;
    glGenTextures(1, &texture);

    int width, height;
    unsigned char* image;

    glBindTexture(GL_TEXTURE_2D, texture);
    image = SOIL_load_image(path, &width, &height, 0, SOIL_LOAD_RGB);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    SOIL_free_image_data(image);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texture;
}

void createShaderProgram(const GLchar* vertSrc, const GLchar* fragSrc, GLuint& vertexShader, GLuint& fragmentShader, GLuint& shaderProgram)
{
    // Create and compile the vertex shader
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertSrc, NULL);
    glCompileShader(vertexShader);

    // Create and compile the fragment shader
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragSrc, NULL);
    glCompileShader(fragmentShader);

    // Link the vertex and fragment shader into a shader program
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glBindFragDataLocation(shaderProgram, 0, "outColor");     
    glLinkProgram(shaderProgram);
}

void specifySceneVertexAttributes(GLuint shaderProgram)
{
    GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
    glEnableVertexAttribArray(posAttrib);
    glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), 0);

    GLint colAttrib = glGetAttribLocation(shaderProgram, "color");
    glEnableVertexAttribArray(colAttrib);
    glVertexAttribPointer(colAttrib, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));

    GLint texAttrib = glGetAttribLocation(shaderProgram, "texcoord");
    glEnableVertexAttribArray(texAttrib);
    glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(6 * sizeof(GLfloat)));
}

void specifyScreenVertexAttributes(GLuint shaderProgram)
{
    GLint posAttrib = glGetAttribLocation(shaderProgram, "position");
    glEnableVertexAttribArray(posAttrib);
    glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);

    GLint texAttrib = glGetAttribLocation(shaderProgram, "texcoord");
    glEnableVertexAttribArray(texAttrib);
    glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (void*)(2 * sizeof(GLfloat)));
}

//v0.1
float rotXY, rotYY, rotZY, scaleX, scaleY, transX, transY;

//v0.2
float timeSHA_INCR = 0;

//v0.2
unsigned char* gl_texture_bytes;
cv::Mat get_ocv_img_from_gl_img(GLuint ogl_texture_id)
{
    glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
    GLenum gl_texture_width, gl_texture_height;

    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&gl_texture_width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&gl_texture_height);

    //unsigned char* gl_texture_bytes = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*3);
    if(gl_texture_bytes == NULL){
	gl_texture_bytes = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*4);
    }
    glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_RGBA, GL_UNSIGNED_BYTE, gl_texture_bytes);

    return cv::Mat(gl_texture_height, gl_texture_width, CV_8UC4, gl_texture_bytes);
}
//v0.3
unsigned char* gl_texture_bytesA;
cv::Mat get_ocv_img_from_gl_imgA(GLuint ogl_texture_id)
{
    glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
    GLenum gl_texture_width, gl_texture_height;

    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&gl_texture_width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&gl_texture_height);

    //unsigned char* gl_texture_bytesA = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*3);
    if(gl_texture_bytesA == NULL){
	gl_texture_bytesA = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*4);
    }
    glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_RGBA, GL_UNSIGNED_BYTE, gl_texture_bytesA);

    return cv::Mat(gl_texture_height, gl_texture_width, CV_8UC4, gl_texture_bytesA);
}

Mat out;
//auto t_start;
float sourceWidth, sourceHeight;
GLuint vaoCube, vaoQuad, vboCube, vboQuad;
GLuint screenVertexShader, screenFragmentShader, screenShaderProgram;
GLuint sceneVertexShader, sceneFragmentShader, sceneShaderProgram;
GLint uniModel;
GLuint texColorBuffer, frameBuffer, rboDepthStencil, texColorBufferA;//, texColorBufferB;
GLint uniColor, rotY, scaleTranslate;
bool toggleForward;
GLint timeSHA, choosePass;//v0.2
GLuint texKitten;


//FLOW
//DOT PRODUCT ANGLE
float angleBetweenA(const Point &v1, const Point &v2)
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
void drawOptFlowMapA (const Mat& flow, Mat& cflowmap, int step, double scale, const Scalar& color) {
	for(int y = 0; y < cflowmap.rows; y += step/2){
		for(int x = 0; x < cflowmap.cols; x += step/2)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);		
			line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x*scale/3), cvRound(y+fxy.y*scale/3)), color);
			circle(cflowmap, Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);				
		}
	}
}
void drawOptFlowMapA1 (const Mat& flow, Mat& cflowmap, int step, double scale, const Scalar& color) {
	for(int y = 0; y < cflowmap.rows; y += step){
		for(int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);		
			line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x*scale/8), cvRound(y+fxy.y*scale/8)), color);
			circle(cflowmap, Point(cvRound(x+fxy.x*scale/8), cvRound(y+fxy.y*scale/8)), 1, color, -1);				
		}
	}
}
void drawOptFlowMapB (const Mat& flow, Mat& cflowmap, int step, double scale, const Scalar& color) { //DISCARD RANDOM DIRECTIONS

	//INIT CLUSTERING
	int total_points, total_values, K, max_iterations, has_name;
	//total_points = cflow.rows * cflow.cols;
	total_values = 2;
	K=2;
	max_iterations=100;
	has_name = 0;
	//cin >> total_points >> total_values >> K >> max_iterations >> has_name;
	vector<PointM> points;
	vector<PointM> pointsXY;
	string point_name;		
	int count_points = 0;
	//END INIT CLUSTERING

	if(1==1){
		for(int y = 0; y < cflowmap.rows; y += step/1){
			for(int x = 0; x < cflowmap.cols; x += step/1)
			{
					vector<double> values;// CLUSTER
					const Point2f& fxy = flow.at<Point2f>(y, x);
					//QUERRY AROUND FLOWS
					const Point2f& fxyU = flow.at<Point2f>(y+1, x);
					const Point2f& fxyD = flow.at<Point2f>(y-1, x);
					const Point2f& fxyL = flow.at<Point2f>(y, x-1);
					const Point2f& fxyR = flow.at<Point2f>(y, x+1);
					const Point2f& fxyUL = flow.at<Point2f>(y+1, x-1);
					const Point2f& fxyUR = flow.at<Point2f>(y+1, x+1);
					const Point2f& fxyDL = flow.at<Point2f>(y-1, x-1);
					const Point2f& fxyDR = flow.at<Point2f>(y-1, x+1);
				
					//FOR EACH PIXEL with same direcion around, increase vector length
					//! dot product computed in double-precision arithmetics
	    				//double ddot(const Point_& pt) const;
					float angleThres = 15;//25;//25;//15;
					float impactMeasure = 0;
					float impactFactor = 2;				
					float product;

					float flowlength = sqrt(fxy.x*fxy.x + fxy.y*fxy.y);
					float flowlengthU = sqrt(fxyU.x*fxyU.x + fxyU.y*fxyU.y);
					float flowlengthD = sqrt(fxyD.x*fxyD.x + fxyD.y*fxyD.y);
					float flowlengthL = sqrt(fxyL.x*fxyL.x + fxyL.y*fxyL.y);
					float flowlengthR = sqrt(fxyR.x*fxyR.x + fxyR.y*fxyR.y);
					float flowlengthUL = sqrt(fxyUL.x*fxyUL.x + fxyUL.y*fxyUL.y);
					float flowlengthUR = sqrt(fxyUR.x*fxyUR.x + fxyUR.y*fxyUR.y);
					float flowlengthDL = sqrt(fxyDL.x*fxyDL.x + fxyDL.y*fxyDL.y);
					float flowlengthDR = sqrt(fxyDR.x*fxyDR.x + fxyDR.y*fxyDR.y);				

					float speedThreshold = 0.01;//0.15;//0.02;//0.0012;0.05
					float speedDiff = 8;
					product = angleBetweenA(fxy, fxyU);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthU)  < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyD);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthD)  < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyL);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthL)  < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyR);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthR)  < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyUL);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthUL) < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyUR);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthUR) < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyDL);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthDL) < speedThreshold) { speedDiff -= 1; }
					product = angleBetweenA(fxy, fxyDR);
					if(product < angleThres) { impactMeasure += impactFactor; } if(abs(flowlength - flowlengthDR) < speedThreshold) { speedDiff -= 1; }

					//REDUCE IMPACT IF SAME DIRECTION AS PAN - TILT
					//const Point2f& fxyDIR = Point(cvRound(fxy.x), cvRound(fxy.y));
					//const Point2f& flowDIR =  Point(cvRound(panRate), cvRound(tiltRate));
					//product = angleBetweenA(fxyDIR, flowDIR);
					//if(product < angleThres) { impactMeasure -= impactFactor * 6; }

					//COMBINE	
					if(impactMeasure > impactFactor * 7 && speedDiff < 1 && flowlength < 8){ //18){
						const Point2f& fxyFINAL = Point(cvRound(x+impactMeasure*0.2*(fxy.x*scale/3)), cvRound(y+impactMeasure*0.2*(fxy.y*scale/3)));
						//line(cflowmap, Point(x,y),fxyFINAL, CV_RGB(255, 0, 0));

						line(cflowmap, Point(x,y), fxyFINAL, color);
						circle(cflowmap, Point(x,y), 1, color, -1);
						//circle(cflowmap, fxyFINAL, 1, color, -1);

						//CLUSTER
						values.push_back(fxyFINAL.x);//values.push_back(x); //values.push_back(fxy.x / sqrt(fxy.x*fxy.x + fxy.y*fxy.y)); //values.push_back(fxyFINAL.x);
						values.push_back(fxyFINAL.y);//values.push_back(y); //values.push_back(fxy.y / sqrt(fxy.x*fxy.x + fxy.y*fxy.y)); //values.push_back(fxyFINAL.y);				
						PointM p(count_points, values);
						points.push_back(p);

						vector<double> valuesXY;
						valuesXY.push_back(x);
						valuesXY.push_back(y);				
						PointM pXY(count_points, valuesXY);
						pointsXY.push_back(pXY);				
						count_points++;
						//END CLUSTER
					}else{
						//CLUSTER
						//values.push_back(0);
						//values.push_back(0);				
						//PointM p(count_points, values); 
						//points.push_back(p);				
						//count_points++;
						//END CLUSTER			
					}				
			}
		}

		//CLUSTER and return points of interest

		//KMEANS
		
		//PLOT VECTORS
		
		
	}//END if 1==0
	else{
		points.clear();
		for(int y = 0; y < cflowmap.rows; y += step){
				for(int x = 0; x < cflowmap.cols; x += step)
				{
					vector<double> values;
					const Point2f& fxy = flow.at<Point2f>(y, x);				
					values.push_back(fxy.x);
					values.push_back(fxy.y);				
					PointM p(count_points, values);
					points.push_back(p);				
					count_points++;
				}
		}
	}
	

	total_points = count_points;

	KMeans kmeans(K, total_points, total_values, max_iterations);	//INITIALIZE KMEANS 

	//cout<<"number of points = " << count_points << " ," << K << " ," << total_points <<" ," << total_values <<" ," << max_iterations << endl;
	if(count_points-1 < K){
		cout<<"Error: Number of clusters greater than number of points." << count_points << endl;
		///////return 1;
	}

	int maxPointCount = 15;//in1
	maxPointCount = 12;//8;//10;//30; //in2 

	cout << "point count to cluster" << points.size() << endl;

	vector<double> pointOfInterest = kmeans.run(points, maxPointCount, flow, cflowmap); //provides points IDs for smallest cluster
	kmeans.meanMedian(points, pointsXY, pointOfInterest, flow, cflowmap,step, scale); ///
	//END CLUSTERING
}

Mat fgMaskMOG2Plot;

//// FINAL RESULT
void printGradient(cv::Mat &_input,const cv::Point &_center, const double radius)
{
   cv::circle(_input, _center, radius, CV_RGB(0, 0, 0),-1);//cv::Scalar(0, 0, 0), -1);

   //for(int i=1; i<2; i=i++)//for(double i=1; i<radius; i=i++)
   //{
       //const int color = 255-int(i/radius * 255); //or some another color calculation
       //cv::circle(_input,_center,i,cv::Scalar(color, color, color),2);
       //int colorA = 255 - int(i/radius * 255); //or some another color calculation
       //cv::circle(_input,_center,i,cv::Scalar(color, color, color),2);
   //}  
}//
Rect2d trackWindow;
vector<Point> prevCenters; 
vector<float> prevAreas;
Mat img1_prev;
Mat back_sub_prev;
Rect2d processVideo(Mat out2, float scaling, int filterPixels1) {

	//float scaling=1; 
	int filterPixels = 6; 

	//imshow("FG Mask MOG 2 FULL", out2);

///	cvtColor(out2,out2, CV_BGR2GRAY);//out2.convertTo(out2, CV_8UC1);
//	cv::resize(out2, out2, cv::Size(), scaling, scaling);  

	int filerSize = filterPixels;//3;
        //erode(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	//erode(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	//erode(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize+1, filerSize+1)));
//	//medianBlur(out2,out2, 1);
 //     //dilate(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));


	//GET GPU USAGE INFO

	//sudo intel_gpu_top -o gpustats.txt -s 1000

	const bool useGpu = true;
	cv::ocl::setUseOpenCL(useGpu);
	UMat passGPU;
	out2.copyTo(passGPU);
	cvtColor(passGPU,passGPU, CV_BGR2GRAY);
	resize(passGPU,passGPU, cv::Size(), scaling, scaling); 
	medianBlur(passGPU,passGPU, 3);
	dilate(passGPU,passGPU, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	dilate(passGPU,passGPU, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	//dilate(passGPU,passGPU, getStructuringElement(MORPH_ELLIPSE, Size(filerSize+3, filerSize+3)));
	//dilate(passGPU,passGPU, getStructuringElement(MORPH_ELLIPSE, Size(filerSize+3, filerSize+3)));	
	passGPU.copyTo(out2);
	//flow.getUMat(ACCESS_RW)
	//cv::ocl::dilate(out2.getUMat(ACCESS_RW), out2.getUMat(ACCESS_RW), getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));
	//ocl::dilate(const oclMat& src, oclMat& dst, const Mat& kernel, Point anchor=Point(-1, -1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue())

        //morphological closing (fill small holes in the foreground)
 //     dilate(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize, filerSize)));	
//	dilate(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize+1, filerSize+1)));
//	dilate(out2, out2, getStructuringElement(MORPH_ELLIPSE, Size(filerSize+5, filerSize+5)));	
	
	vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        //Detect edges using Threshold
	//threshold(out2, out2, 100, 255, THRESH_BINARY);
        //threshold(fgMaskMOG2, fgMaskMOG2, 100, 255, THRESH_BINARY);

        //find contours
        //findContours(fgMaskMOG2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); 	
 	//findContours(fgMaskMOG2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));
	findContours(out2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));

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
	out2.copyTo(fgMaskMOG2Plot);
	int areaThreshold = 10;//130;//55; 260	
	float prevArea=0;
	int rectID = -1;
	if( contours.size() > 0 &&  contours.size() < 18){ //7
		//cout << "contours found: " << contours.size() << endl;
		for (int i = 0; i< contours.size(); i++)
		{	  
			//rectangle(fgMaskMOG2Plot, boundRect[i].tl()/scaling, boundRect[i].br()/scaling, CV_RGB(255, 0, 255), 2, 8, 0);
			//circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);		 
			if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) > prevArea){

				rectID = i;
				
				prevArea = contourArea(contours[i]);				

				//boundRect[rectID] = boundingRect(contours[i]);
				//cout << "with area: " << contourArea(contours[i]) << endl;

				/////break;
			}		
		}
	}	

	//PLOT RECTANGLES
	int image_width = fgMaskMOG2Plot.cols;
	float areaMAX = (fgMaskMOG2Plot.rows * fgMaskMOG2Plot.cols) / 4;//9;
	Mat img1(fgMaskMOG2Plot.rows, fgMaskMOG2Plot.cols, CV_8UC3, Scalar(1, 0, 0));
	vector<int> weightCloseness;
	if(1==1){		
		//drawContours(fgMaskMOG2Plot, contours, -1,  CV_RGB(255, 0, 255), 10,8,hierarchy );
		if( contours.size() > 0){

			//use memory info ///// FIND CLOSEST CENTER
			int closestID = 0;
			float minDist = 10000;
			for (int i = 0; i< contours.size(); i++)
			{
				if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) < areaMAX){
					//check closest center and area
					if((float)boundRect[i].size().height / (float)boundRect[i].size().width > 1.5f
								|| 	 (float)boundRect[i].size().width / (float)boundRect[i].size().height > 3.0f						
					){
						//DONT PLOT
					}else{
						for (int j = 0; j< prevCenters.size(); j++)
						{
							float currentLen = sqrt( (prevCenters[j].x - boundRect[i].x) * (prevCenters[j].x - boundRect[i].x) + 
							(prevCenters[j].y - boundRect[i].y)*(prevCenters[j].y - boundRect[i].y) ) ;
							if(currentLen < minDist){
								minDist = currentLen;
								closestID = i;								
							}
						}
					}
				}
				//GIVE  WEIGHT TO THOSE CENTERS THAT APPEAR CLOSE TO ANOTHER
				//weightCloseness.push_back();
			}
			cout << "closestID = " << closestID << endl;
			//rectangle(img1, boundRect[closestID].tl()/1, boundRect[closestID].br()/1, CV_RGB(255, 0, 0), 2, 8, 0);


			////// FIND CLOSE CENTERS and paint weighted with that information						
			for (int i = 0; i < contours.size(); i++)
			{		
				if(prevCenters.size() > 0){
					closestID = 0;
					Point center_of_rect = (boundRect[i].br() + boundRect[i].tl()) * 0.5;
					float currentLenINIT = sqrt( (prevCenters[0].x - center_of_rect.x) * (prevCenters[0].x - center_of_rect.x) + 
					(prevCenters[0].y - center_of_rect.y)*(prevCenters[0].y - center_of_rect.y) ) ;
					minDist = currentLenINIT;		
					for (int j = 0; j< prevCenters.size(); j++)
					{
						//Point center_of_rect = (boundRect[i].br() + boundRect[i].tl()) * 0.5;
						float currentLen = sqrt( (prevCenters[j].x - center_of_rect.x) * (prevCenters[j].x - center_of_rect.x) + 
						(prevCenters[j].y - center_of_rect.y)*(prevCenters[j].y - center_of_rect.y) ) ;
						if(currentLen < minDist){
							minDist = currentLen;
							closestID = j;								
						}
					}					
					//GIVE  WEIGHT TO THOSE CENTERS THAT APPEAR CLOSE TO ANOTHER
					float weight = 255 - ((minDist*255)/image_width);
					//weight = 255 - ((minDist*minDist*255)/(image_width*image_width));
					if(minDist > 19 || minDist < 1){ //if(minDist > 12 || minDist < 1){ //if lower than 60 pixel do not draw //6
						weight = 0;				
					}

					////// FIND CENTERS THAT MOVE SIMILAR DIRETION and paint weighted with that information
					//if in assumed motion check each center if has similar direcion to others and weight by that count of similariy to find background resulting rectangles
					//if(1==1 && contours.size() > 10)  
					//{

						//line(fgMaskMOG2Plot, Point(prevCenters[closestID].x,prevCenters[closestID].y),Point(center_of_rect.x,center_of_rect.y), CV_RGB(255, 255, 0));
					//}

					weightCloseness.push_back(weight);
					//cout << "center " << i << " weight = " << weightCloseness[i] << endl;
				}else
				{
					weightCloseness.push_back(0);
				}
			}

			

			//CLEAR previous Centers afer use, to update with new below
			prevCenters.clear();
			
			for (int i = 0; i< contours.size(); i++)
			{
				if(contourArea(contours[i]) > areaThreshold && contourArea(contours[i]) < areaMAX)
				{

					//if(contours.size() < 6){
					//	rectangle(fgMaskMOG2Plot, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 2, 8, 0);//bottom right, W-H
					//	rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 2, 8, 0);//bottom right, W-H
					//}else{
						//eliminate some boxes						
						if((float)boundRect[i].size().height / (float)boundRect[i].size().width > 1.5 //1.1f
							|| 	 (float)boundRect[i].size().width / (float)boundRect[i].size().height > 4.0f						
						){
							//DONT PLOT
						}else
						{
							Point center_of_rect = (boundRect[i].br() + boundRect[i].tl())*0.5;
							//cout << "H=" << boundRect[i].size().height << "... W=" << boundRect[i].size().width << endl;
							rectangle(fgMaskMOG2Plot, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 2, 8, 0);	
							//rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 12, 8, 0);//put -1 to fill full	
							
							if(boundRect[i].size().width/2 < fgMaskMOG2Plot.rows / 6){
								//circle(img1, center_of_rect, boundRect[i].size().width/2,CV_RGB(255, 0, 255), CV_FILLED);
								//circle(img1, center_of_rect, fgMaskMOG2Plot.rows / 12,CV_RGB(255, 0, 255), CV_FILLED);
								//circle(img1, center_of_rect, boundRect[i].size().width/2,CV_RGB(1, 1, 1) * weightCloseness[i], CV_FILLED);
								if(weightCloseness[i] > 200){
									
									//PAINT GRADIENT
									//printGradient(img1, center_of_rect,  boundRect[i].size().width/2);
									double radius = boundRect[i].size().width/2;
									//cv::circle(img1, center_of_rect, radius, cv::Scalar(0, 0, 0)* weightCloseness[i],-1);//cv::Scalar(0, 0, 0), -1);
								   	//for(int m=1; m<2; m=m++)//for(double i=1; i<radius; i=i++)
								   	//{
								       		//const int color = 255-int(i/radius * 255); //or some another color calculation
								       		//cv::circle(_input,_center,i,cv::Scalar(color, color, color),2);
								       		//int colorA = 255 - int(i/radius * 255); //or some another color calculation
								       		//cv::circle(_input,_center,i,cv::Scalar(color, color, color),2);
								   	//} 
									for (double i1 = 1; i1 < radius; i1=i1+5){
										int color = 255-int(i1/radius * 255); //or some another color calculation
								       		cv::circle(img1, center_of_rect,i1,CV_RGB(color, color, color)* weightCloseness[i],2);
									}

									circle(img1, center_of_rect, boundRect[i].size().width/2,CV_RGB(1, 1, 1) * weightCloseness[i], CV_FILLED);
									rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 12, 8, 0);
									//rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 6, 4, 0);
									/////circle(img1, center_of_rect, fgMaskMOG2Plot.rows / 12,CV_RGB(1, 1, 1) * weightCloseness[i], CV_FILLED);
									//line(img1, Point(center_of_rect.x,center_of_rect.y-30), Point(center_of_rect.x,center_of_rect.y+30),CV_RGB(255, 0, 255), 3);
								}
								//rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 12, 8, 0);
							}else{
								/////circle(img1, center_of_rect, fgMaskMOG2Plot.rows / 12,CV_RGB(255, 0, 255), CV_FILLED);
							//	circle(img1, center_of_rect, fgMaskMOG2Plot.rows / 24,CV_RGB(1, 1, 1) * weightCloseness[i], CV_FILLED);	
								//rectangle(img1, boundRect[i].tl()/1, boundRect[i].br()/1, CV_RGB(255, 0, 255), 12, 8, 0);						
							}
							
							//circle(img1, center_of_rect, 45,CV_RGB(255, 255,255) * 0.5, CV_FILLED);					// DRAW CIRCLE TO FIND FLOWMAP FROM !!!!
							//circle(img1, center_of_rect, 45,CV_RGB(255,255,255) * boundRect[i].size().width/fgMaskMOG2Plot.cols, CV_FILLED);
							//circle(img1, center_of_rect, 45,CV_RGB(255,255,255) *1, CV_FILLED);
							
							prevCenters.push_back(center_of_rect);
							prevAreas.push_back(contourArea(contours[i]));		
						}											
					//}
				}
			}
			
		}
		//imshow("FG Mask MOG 22", img1);		
	}

	//imshow("FG Mask MOG 2", fgMaskMOG2Plot);

	//cvtColor(fgMaskMOG2Plot,img1, COLOR_GRAY2BGR);
	//fgMaskMOG2Plot.copyTo(img1);

//////FLOW ON TRACKING WINDOWS
if(1==0 && !img1.empty() && !img1_prev.empty()){
			//if(1==1 && !frame.empty() && !prevFrame.empty()){ //CHECK OPENCL WORKS - Not working, must install Intel SDK, then build Opencv and maybe work
			const bool useGpu = true;
			cv::ocl::setUseOpenCL(useGpu);	
			Mat srcImgMID;
			Mat srcImgMIDPrev;     
			float resizeGPUTexture = 0.5;					
			img1.copyTo(srcImgMID);
			img1_prev.copyTo(srcImgMIDPrev);
					
			cv::resize(srcImgMID, srcImgMID, cv::Size(),resizeGPUTexture *1,resizeGPUTexture* 1);
			cv::resize(srcImgMIDPrev, srcImgMIDPrev, cv::Size(), resizeGPUTexture* 1,resizeGPUTexture* 1);

			//CHECK PREVIOUS FRAME DIFFERENT						
			//Scalar s = sum( frameA - prevFrameA );
			//bool equal = (s[0]==0) && (s[1]==0) && (s[2]==0);
			//if(equal){
			//	cout << "SAME !!!!!!!!!!!!" << endl;
			//}
			Mat_<Point2f> flow, ground_truth;
			cvtColor(srcImgMID,srcImgMID, COLOR_BGR2GRAY);
			cvtColor(srcImgMIDPrev,srcImgMIDPrev, COLOR_BGR2GRAY);

			flow = Mat(srcImgMID.size[0], srcImgMID.size[1], CV_32FC2);
					   
			calcOpticalFlowFarneback(srcImgMIDPrev, srcImgMID, flow.getUMat(ACCESS_RW), 0.5, 3, 15, 3, 5, 1.2, 0);
			//algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); //algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); // PRESET_ULTRAFAST // PRESET_FAST
				
			if( 1==1 )
			{		
				Mat cflow(srcImgMIDPrev.rows, srcImgMIDPrev.cols, CV_8UC3, Scalar(1, 0, 0));//Mat cflow;
				int step = 8;//24/2;
				int scale = 12/1;
				cvtColor(srcImgMIDPrev, cflow, CV_GRAY2BGR);
				//prevFrame.copyTo(cflow);
				drawOptFlowMapB(flow, cflow, step, scale, CV_RGB(0, 255, 0));
				cv::resize(cflow, cflow, cv::Size(), 2, 2);
				//imshow("optical Flow BACK", cflow);
			}
}
img1.copyTo(img1_prev);

//////FLOW ON BACKGROUND SUBTRACTION
if(1==0 && !fgMaskMOG2Plot.empty() && !back_sub_prev.empty()){
			//if(1==1 && !frame.empty() && !prevFrame.empty()){ //CHECK OPENCL WORKS - Not working, must install Intel SDK, then build Opencv and maybe work
			const bool useGpu = true;
			cv::ocl::setUseOpenCL(useGpu);	
			Mat srcImgMID;
			Mat srcImgMIDPrev;     
			float resizeGPUTexture = 0.5;					
			fgMaskMOG2Plot.copyTo(srcImgMID);
			back_sub_prev.copyTo(srcImgMIDPrev);
					
			cv::resize(srcImgMID, srcImgMID, cv::Size(),resizeGPUTexture *1,resizeGPUTexture* 1);
			cv::resize(srcImgMIDPrev, srcImgMIDPrev, cv::Size(), resizeGPUTexture* 1,resizeGPUTexture* 1);

			//CHECK PREVIOUS FRAME DIFFERENT						
			//Scalar s = sum( frameA - prevFrameA );
			//bool equal = (s[0]==0) && (s[1]==0) && (s[2]==0);
			//if(equal){
			//	cout << "SAME !!!!!!!!!!!!" << endl;
			//}
			Mat_<Point2f> flow, ground_truth;
			//cvtColor(srcImgMID,srcImgMID, COLOR_BGR2GRAY);
			//cvtColor(srcImgMIDPrev,srcImgMIDPrev, COLOR_BGR2GRAY);

			flow = Mat(srcImgMID.size[0], srcImgMID.size[1], CV_32FC2);
					   
			calcOpticalFlowFarneback(srcImgMIDPrev, srcImgMID, flow.getUMat(ACCESS_RW), 0.5, 3, 15, 3, 5, 1.2, 0);
			//algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); //algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM); // PRESET_ULTRAFAST // PRESET_FAST
				
			if( 1==1 )
			{		
				Mat cflow(srcImgMIDPrev.rows, srcImgMIDPrev.cols, CV_8UC3, Scalar(1, 0, 0));//Mat cflow;
				int step = 8;//24/2;
				int scale = 12/1;
				cvtColor(srcImgMIDPrev, cflow, CV_GRAY2BGR);
				//prevFrame.copyTo(cflow);
				drawOptFlowMapA1(flow, cflow, step, scale, CV_RGB(0, 255, 0));
				cv::resize(cflow, cflow, cv::Size(), 2, 2);
				//imshow("optical Flow on BACK SUBTRACTOR", cflow);
			}
}
fgMaskMOG2Plot.copyTo(back_sub_prev);
//////END FLOW




	if( contours.size() > 0 &&  contours.size() < 6 && rectID >= 0){ //7
		return boundRect[rectID];
	}
/*
	else if( contours.size() > 0 &&  contours.size() >= 6){ 
		//try discriminate trajectories from previous frames, versus backround rectangles, draw red for debug for now

						Rect2d processVideoFrame = trackWindow;//processVideo(frameSUBRACT,imageScaleBackgroundSUBTRACT, filterPixels, repeats);
						if(processVideoFrame.x != 0 && processVideoFrame.width > 0
							//&& (
							//(processVideoFrame.width > processVideoFrame.height && processVideoFrame.width / processVideoFrame.height < 4) 
							//||
							//(processVideoFrame.height > processVideoFrame.width && processVideoFrame.height / processVideoFrame.width < 4)
							//)
						){
							//BACKGROUND SUBTRACTION BOX ESTIMATION WITH CONFIDENCE METRIC
							//vector<Rect2d> bboxesESTIMATED_STATIC; //estimated bboxes with background subtraction
							//vector<int> bboxESTIMATE_STATIC_frames; //frames of confidence
							if(1==1){	//if(backStatic){
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
								//rectangle(plotFrame, processVideoFrame, Scalar( 255, 110, 0 ), 2, 1 );	
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
							//if(okB == 0){
								tracker2 = trackerUsed::create();						
								tracker2->init(frame, bbox2);
								okB = tracker2->update(frame, bbox2);	
							//}				

							//fullFlowMap = false;
							//cout << " ... and Background subtraction" << endl;				
						}

		//END TRY DISCRIMINATE TRAJECTORIES -------------------------------------------------------///////////////////////
	}
*/
	else{
		Rect2d boundRect1;
		boundRect1.x = 0;
		boundRect1.y = 0;
		boundRect1.width = 0;
		boundRect1.height = 0;
		return boundRect1;
	}     
}
//// END FINAL RESULT




void initRectify360GPU(){

    cout << "Init started" << endl;

    /////////////========================== 360 CAMERA VIEW HANDLER GPU (APPLY RECTIFICATION in GPU INIT REGION) ======================/////////////    
    //t_start = std::chrono::high_resolution_clock::now();   

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    glewInit();

    // Create VAOs
    //vaoCube, vaoQuad;
    glGenVertexArrays(1, &vaoCube);
    glGenVertexArrays(1, &vaoQuad);

    // Load vertex data
    //GLuint vboCube, vboQuad;
    glGenBuffers(1, &vboCube);
    glGenBuffers(1, &vboQuad);

    glBindBuffer(GL_ARRAY_BUFFER, vboCube);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboQuad);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    // Create shader programs
    //GLuint sceneVertexShader, sceneFragmentShader, sceneShaderProgram;
    createShaderProgram(sceneVertexSource, sceneFragmentSource, sceneVertexShader, sceneFragmentShader, sceneShaderProgram);

    //GLuint screenVertexShader, screenFragmentShader, screenShaderProgram;
    createShaderProgram(screenVertexSource, screenFragmentSource, screenVertexShader, screenFragmentShader, screenShaderProgram);

    // Specify the layout of the vertex data
    glBindVertexArray(vaoCube);
    glBindBuffer(GL_ARRAY_BUFFER, vboCube);
    specifySceneVertexAttributes(sceneShaderProgram);

    glBindVertexArray(vaoQuad);
    glBindBuffer(GL_ARRAY_BUFFER, vboQuad);
    specifyScreenVertexAttributes(screenShaderProgram);  

    glUseProgram(sceneShaderProgram);   
    glUniform1i(glGetUniformLocation(sceneShaderProgram, "texFramebuffer"), 0);
    glUseProgram(sceneShaderProgram);
    glUniform1i(glGetUniformLocation(sceneShaderProgram, "texFramebufferPrev"), 1);   

    glUseProgram(screenShaderProgram);
    glUniform1i(glGetUniformLocation(screenShaderProgram, "texFramebuffer"), 0);

    //v0.2
    glUseProgram(screenShaderProgram);
    glUniform1i(glGetUniformLocation(screenShaderProgram, "texFramebufferPrev"), 1);
    glUseProgram(screenShaderProgram);
    glUniform1i(glGetUniformLocation(screenShaderProgram, "texFramebufferPrev2"), 2);

    uniModel = glGetUniformLocation(sceneShaderProgram, "model");

    //Create framebuffer
    //GLuint frameBuffer;
    glGenFramebuffers(1, &frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

    //Create texture to hold color buffer
    //GLuint texColorBuffer;//GLuint frameBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sourceWidth, sourceHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0);

    // Create Renderbuffer Object to hold depth and stencil buffers
    //GLuint rboDepthStencil;
    glGenRenderbuffers(1, &rboDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepthStencil);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, sourceWidth, sourceHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rboDepthStencil);

    // Set up projection   
    glm::mat4 view = glm::lookAt(
        glm::vec3(1.0f, 0.0f, 1.0f),// glm::vec3(2.5f, 2.5f, 2.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    );    

    glUseProgram(sceneShaderProgram);

    GLint uniView = glGetUniformLocation(sceneShaderProgram, "view");
    glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));

    glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.01f, 110.0f);
    //glm::mat4 proj = glm::ortho( 0.f, 800.f, 0.f, 600.f, -10.f, 10.f );

    GLint uniProj = glGetUniformLocation(sceneShaderProgram, "proj");
    glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));

    uniColor = glGetUniformLocation(sceneShaderProgram, "overrideColor");

    //v0.1
    rotY = glGetUniformLocation(screenShaderProgram, "rotY");
    scaleTranslate = glGetUniformLocation(screenShaderProgram, "scaleTranslate");


    //v0.2
    timeSHA = glGetUniformLocation(screenShaderProgram, "time");
    choosePass = glGetUniformLocation(screenShaderProgram, "choosePass");

    //CREATE FINAL RENDER TARGET FOR OPENCV //v0.2    
    glGenTextures(1, &texColorBufferA);
    glBindTexture(GL_TEXTURE_2D, texColorBufferA);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sourceWidth, sourceHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBufferA, 0);

    //https://www.gamedev.net/forums/topic/526339-flipping-an-image-when-drawing/
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glRotatef(180.0f,0.0f,0.0f,1.0f);
    glScalef(-1.0f,1.0f,1.0f);
   
    rotYY  = 0.0f; rotXY  = 0.0f;  rotZY  =  0.0f; scaleX =  1.0f;  scaleY =  1.0f; transX =  0.0f; transY =  0.0f;
    
    toggleForward = false;
    /////////////========================== 360 CAMERA VIEW HANDLER GPU (END APPLY RECTIFICATION in GPU INIT REGION) ===================/////////////

}//end init
Mat out3;
cv::Mat rectify360imageGPU(GLuint texKitten, GLuint texKittenPrev, GLuint texKittenPrev2, bool running, int image360part, float panDiff, float tiltDiff, float zoomDiff, float scale){

	/////////////====================== 360 CAMERA VIEW HANDLER GPU (APPLY RECTIFICATION in GPU LOOP REGION) ======================/////////////

	//GET KEYS
	// Black magic to prevent Linux from buffering keystrokes.
	if(1==0){
	    	struct termios t;
	    	tcgetattr(STDIN_FILENO, &t);
	    	t.c_lflag &= ~ICANON;
	    	tcsetattr(STDIN_FILENO, TCSANOW, &t);

		// Once the buffering is turned off, the rest is simple.
	    	//cout << "Enter a character: ";
	    	//char c,d,e;
		char e,c;
	    	cin >> c;
	    	//cin >> d;
	    	cin >> e;
	    	//cout << "\nYour character was ";
		// Using 3 char type, Cause up down right left consist with 3 character
	    	//if ((c==27)&&(d=91)) {
	    	if(toggleForward){
			if (e==65) { rotYY=rotYY+0.05f;/*cout << "Vert angle:" << rotYY << "Hor angle:" << rotXY;*/}//cout << "UP";
			if (e==66) { rotYY=rotYY-0.05f;/*cout << "Vert angle:" << rotYY << "Hor angle:" << rotXY;*/}//cout << "DOWN";
			if (e==67) { rotXY=rotXY+0.05f;/*cout << "Vert angle:" << rotYY << "Hor angle:" << rotXY;*/}//cout << "RIGHT";
			if (e==68) { rotXY=rotXY-0.05f;/*cout << "Vert angle:" << rotYY << "Hor angle:" << rotXY;*/}// cout << "LEFT";
	    	}else{
			if (e==65) { rotZY=rotZY+0.05f;/*cout << "Zoom:" << rotZY ;*/} //cout << "UP";
			if (e==66) { rotZY=rotZY-0.05f;/*cout << "Zoom:" << rotZY ;*/}//cout << "DOWN";
	 		if (e==67) { scaleX=scaleX+0.05f;/*cout << "Hor Scale:" << scaleX;*/}//cout << "RIGHT";
			if (e==68) { scaleX=scaleX-0.05f;/*cout << "Hor Scale:" << scaleX;*/}// cout << "LEFT";
	    	}

		if(c=='E' || c=='e')
		{
			if(toggleForward)
			{
				toggleForward=false;
			}
			else{
				toggleForward=true;
			}
		}
	}//end 1==0

    	//}
	//END GET KEYS	

	
	// SHADER VARIABLES	
	//do rotation based on part
	if(image360part == 0){
		rotXY = 0.0f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 0.0f; scaleY = 0.0f; transX = 0.0f; transY = 0.0f;
	}else
	if(image360part == 1){
		rotXY = -0.8f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 1.12f; scaleY = 2.0f; transX = 0.0f; transY = 0.0f;
	}
	else if(image360part == 2){
		rotXY = 60.0f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 1.12f; scaleY = 2.0f; transX = 0.0f; transY = 0.0f;
	}
	else if(image360part == 3){
		rotXY = 120.0f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 1.12f; scaleY = 2.0f; transX = 0.0f; transY = 0.0f;
	}
	else if(image360part == 4){
		rotXY = 180.0f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 1.12f; scaleY = 2.0f; transX = 0.0f; transY = 0.0f;
	}else {
		rotXY = 220.0f; rotYY = 0.0f; rotZY = 0.0f; scaleX = 1.12f; scaleY = 2.0f; transX = 0.0f; transY = 0.0f;
	}

	//v0.2
	glUniform3f(rotY, panDiff, tiltDiff, zoomDiff);
	glUniform4f(scaleTranslate,scaleX,scaleY,transX,transY);

	//v0.2
	timeSHA_INCR = timeSHA_INCR + 0.001f;
	glUniform1f(timeSHA, timeSHA_INCR);//v0.2

        /////glDrawArrays(GL_TRIANGLES, 0, 36);
        glUniform3f(uniColor, 1.0f, 1.0f, 1.0f);
        glDisable(GL_STENCIL_TEST);

	//v0.3 - choose pass
	glUniform1f(choosePass, 0); //PASS 0 - Create background subtract mask

        // Bind default framebuffer and draw contents of our framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, texColorBufferA);//glBindFramebuffer(GL_FRAMEBUFFER, 0); //v0.2
        glBindVertexArray(vaoQuad);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(screenShaderProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texKitten);
	//v0.2
if(image360part == 0){
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texKittenPrev);
}else{
	glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texColorBufferA);
}
 	glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texKittenPrev2);
        glDrawArrays(GL_TRIANGLES, 0, 6);
	
	// CONVERT TO OPENCV MAT
	//out = get_ocv_img_from_gl_img(texColorBufferA);	
	frameBuffer = texColorBufferA;

	glUniform1f(choosePass, 1); //PASS 1 - Draw final background subtract result

	glBindFramebuffer(GL_FRAMEBUFFER, texColorBufferA);//glBindFramebuffer(GL_FRAMEBUFFER, 0); //v0.2
        glBindVertexArray(vaoQuad);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(screenShaderProgram);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texKitten);
	//v0.2
if(image360part == 0){
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texKittenPrev);
}else{
	glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, frameBuffer);
}
 	glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texKittenPrev2);
        glDrawArrays(GL_TRIANGLES, 0, 6);

	Mat out2 =  get_ocv_img_from_gl_imgA(texColorBufferA);
	
	//imshow("out2",out2);

	out3 = out2;

	//VideoWriter videoOut1("outcppBACKSUBRACT.avi",CV_FOURCC('M','J','P','G'),30, Size(out2.cols,out2.rows)); 
	//videoOut1.write(out2);

	///////////// ---------------------- TEST BORDERS FINDER ---------------------------------------

 	trackWindow = processVideo(out2, scale, 6);

	//----------------------------------------END TEST BORDERS FINDER----------------------------------------
     
	/////////////====================== 360 CAMERA VIEW HANDLER GPU (END APPLY RECTIFICATION in GPU LOOP REGION) ===================/////////////
	
	return out;
}

void freeGPUbuffers(GLuint texKitten){
    glDeleteRenderbuffers(1, &rboDepthStencil);
    glDeleteTextures(1, &texColorBuffer);
    glDeleteFramebuffers(1, &frameBuffer);

    glDeleteTextures(1, &texKitten);
    //glDeleteTextures(1, &texPuppy);

    glDeleteProgram(screenShaderProgram);
    glDeleteShader(screenFragmentShader);
    glDeleteShader(screenVertexShader);

    glDeleteProgram(sceneShaderProgram);
    glDeleteShader(sceneFragmentShader);
    glDeleteShader(sceneVertexShader);

    glDeleteBuffers(1, &vboCube);
    glDeleteBuffers(1, &vboQuad);

    glDeleteVertexArrays(1, &vaoCube);
    glDeleteVertexArrays(1, &vaoQuad);
}

void cvMatToGlTex(cv::Mat tex_img, GLuint* texID) 
{
    glGenTextures(1, texID); // OpenGL will dereference the pointer and write the
                // value of the new texture ID here. The pointer value you passed in is 
                // indeterminate because you didn't assign it to anything. 
                // Dereferencing a pointer pointing to memory that you don't own is 
                // undefined behaviour. The reason it's not crashing here is because I'll 
                // bet your compiler initialised the pointer value to null, and 
                // glGenTextures() checks if it's null before writing to it. 

    glBindTexture(GL_TEXTURE_2D, *texID); // Here you dereference an invalid pointer yourself. 
                            // It has a random value, probably null, and that's 
                            // why you get a crash

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_img.cols, tex_img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, tex_img.ptr());

    return;
}

//https://gist.github.com/zhangzhensong/03f67947c22acb5ee922
void BindCVMat2GLTexture(cv::Mat image, GLuint& imageTexture, int initialize)
{
   if(image.empty()){
      std::cout << "image empty" << std::endl;
   }else{
      //glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      //if(initialize==1){
	      glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	      glGenTextures(1, &imageTexture);
	      glBindTexture(GL_TEXTURE_2D, imageTexture);
      //}
	      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	      // Set texture clamping method
	      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	      cv::cvtColor(image, image, CV_RGBA2BGRA);
	

		glTexImage2D(GL_TEXTURE_2D,         // Type of texture
		             	0,                   // Pyramid level (for mip-mapping) - 0 is the top level
				GL_RGBA,              // Internal colour format to convert to
		             	image.cols,          // Image width  i.e. 640 for Kinect in standard mode
		             	image.rows,          // Image height i.e. 480 for Kinect in standard mode
		             	0,                   // Border width in pixels (can either be 1 or 0)
				GL_BGRA,              // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
				GL_UNSIGNED_BYTE,    // Image data type
				image.ptr());        // The actual image data itself

		//glDeleteTextures(1, &imageTexture);
     }
} 
/////////////========================== 360 CAMERA VIEW HANDLER GPU (END INIT REGION) =================================================/////////////



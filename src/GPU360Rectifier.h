/////////////========================== 360 CAMERA VIEW HANDLER GPU (INIT REGION) =================================================/////////////

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
    in vec3 Color;
    in vec2 Texcoord;
    in float rotY=0;
    out vec4 outColor;
    uniform sampler2D texKitten;
    uniform sampler2D texPuppy;
    void main()
    {        
	outColor =  texture(texKitten, vec2(Texcoord.x, Texcoord.y));
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
    out vec4 outColor;
    //in float rotY;
    uniform vec3 rotY;
    uniform vec4 scaleTranslate;
    uniform sampler2D texFramebuffer;

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
        //outColor = texture(texFramebuffer, Texcoord);

	//vec2 u_translate =vec2(0.0,0.0);// vec2(scaleTranslate.z,scaleTranslate.w);//vec2(0.0,0.0);
	//vec2 u_scale = vec2(1.12,2.0);//vec2(scaleTranslate.x,scaleTranslate.y);//vec2(2.0,2.0);//vec2(rotY.z,2.0);//vec2(2.0,2.0);
	//vec2 u_rotate = vec2(0,0);// vec2(rotY.x,rotY.y);

	vec2 u_translate = vec2(scaleTranslate.z,scaleTranslate.w);//vec2(0.0,0.0);
	vec2 u_scale = vec2(scaleTranslate.x,scaleTranslate.y);//vec2(2.0,2.0);//vec2(rotY.z,2.0);//vec2(2.0,2.0);
	vec2 u_rotate = vec2(rotY.x,rotY.y);

	//ROTATE
	vec2 TexcoordA = Texcoord;
        float sin_factor = sin(rotY.z);
        float cos_factor = cos(rotY.z);
        //coord = (coord - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
	//coord = vec2((coord.x - 0.5) * (Resolution.x / Resolution.y), coord.y - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
     	vec2 coord = vec2((TexcoordA.x - 0.5) * (16 / 9), TexcoordA.y - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
        coord += 0.5;
    //    TexcoordA = coord;
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
	outColor =  texture(texFramebuffer, vec2(1-uvsProjected.x, uvsProjected.y));
	//outColor =  vec4(Color, 1.0) * texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
	//outColor =  texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
    }
)glsl";
const GLchar* screenFragmentSourcePARAM = R"glsl(
    #version 150 core
    in vec2 Texcoord;
    out vec4 outColor;
    //in float rotY;
    uniform vec3 rotY;
    uniform vec4 scaleTranslate;
    uniform sampler2D texFramebuffer;

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
        //outColor = texture(texFramebuffer, Texcoord);

	vec2 u_translate = vec2(scaleTranslate.z,scaleTranslate.w);//vec2(0.0,0.0);
	vec2 u_scale = vec2(scaleTranslate.x,scaleTranslate.y);//vec2(2.0,2.0);//vec2(rotY.z,2.0);//vec2(2.0,2.0);
	vec2 u_rotate = vec2(rotY.x,rotY.y);

	//ROTATE
	vec2 TexcoordA = Texcoord;
        float sin_factor = sin(rotY.z);
        float cos_factor = cos(rotY.z);
        //coord = (coord - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
	//coord = vec2((coord.x - 0.5) * (Resolution.x / Resolution.y), coord.y - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
     	vec2 coord = vec2((TexcoordA.x - 0.5) * (16 / 9), TexcoordA.y - 0.5) * mat2(cos_factor, sin_factor, -sin_factor, cos_factor);
        coord += 0.5;
        TexcoordA = coord;
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
	outColor =  texture(texFramebuffer, vec2(uvsProjected.x, uvsProjected.y));
	//outColor =  vec4(Color, 1.0) * texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
	//outColor =  texture(texFramebuffer, vec2(Texcoord.x, Texcoord.y));
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
unsigned char* gl_texture_bytes;
cv::Mat get_ocv_img_from_gl_img(GLuint ogl_texture_id)
{
    glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
    GLenum gl_texture_width, gl_texture_height;

    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&gl_texture_width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&gl_texture_height);

    //unsigned char* gl_texture_bytes = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*3);
    if(gl_texture_bytes == NULL){
	gl_texture_bytes = (unsigned char*) malloc(sizeof(unsigned char)*gl_texture_width*gl_texture_height*3);
    }
    glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_BGR, GL_UNSIGNED_BYTE, gl_texture_bytes);

    return cv::Mat(gl_texture_height, gl_texture_width, CV_8UC3, gl_texture_bytes);
}

Mat out;
//auto t_start;
float sourceWidth, sourceHeight;
GLuint vaoCube, vaoQuad, vboCube, vboQuad;
GLuint screenVertexShader, screenFragmentShader, screenShaderProgram;
GLuint sceneVertexShader, sceneFragmentShader, sceneShaderProgram;
GLint uniModel;
GLuint texColorBuffer, frameBuffer, rboDepthStencil, texColorBufferA;
GLint uniColor, rotY, scaleTranslate;
bool toggleForward;
GLuint texKitten;

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
    glUniform1i(glGetUniformLocation(sceneShaderProgram, "texKitten"), 0);
    //glUniform1i(glGetUniformLocation(sceneShaderProgram, "texPuppy"), 1);

    glUseProgram(screenShaderProgram);
    glUniform1i(glGetUniformLocation(screenShaderProgram, "texFramebuffer"), 0);

    uniModel = glGetUniformLocation(sceneShaderProgram, "model");

    // Create framebuffer
    //GLuint frameBuffer;
    glGenFramebuffers(1, &frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

    // Create texture to hold color buffer
    //GLuint texColorBuffer;//GLuint frameBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sourceWidth, sourceHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

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

    //CREATE FINAL RENDER TARGET FOR OPENCV //v0.2
    //texColorBufferA;
    glGenTextures(1, &texColorBufferA);
    glBindTexture(GL_TEXTURE_2D, texColorBufferA);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, sourceWidth, sourceHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

//https://www.gamedev.net/forums/topic/526339-flipping-an-image-when-drawing/
glMatrixMode(GL_TEXTURE);
glLoadIdentity();
glRotatef(180.0f,0.0f,0.0f,1.0f);
glScalef(-1.0f,1.0f,1.0f);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBufferA, 0);
    
    //rotYY  = -0.8f; rotXY  = -0.4f;  rotZY  =  0.0f; scaleX =  2.0f;  scaleY =  2.0f; transX =  0.0f; transY =  0.0f;
    rotYY  = 0.0f; rotXY  = 0.0f;  rotZY  =  0.0f; scaleX =  1.0f;  scaleY =  1.0f; transX =  0.0f; transY =  0.0f;
    
    toggleForward = false;
    /////////////========================== 360 CAMERA VIEW HANDLER GPU (END APPLY RECTIFICATION in GPU INIT REGION) ===================/////////////

}//end init

cv::Mat rectify360imageGPU(GLuint texKitten, bool running, int image360part){

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
        	if (e==65) { rotYY=rotYY+0.05f;/*cout << "Vert angle:" << rotYY << "Hor angle:" << rotXY;*/} //cout << "UP";
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

	/*
        sf::Event windowEvent;
         while (window.pollEvent(windowEvent))
        {
            switch (windowEvent.type)
            {
            case sf::Event::Closed:
                running = false;
                break;
            }
        }
	*/

	/*
        // Bind our framebuffer and draw 3D scene (spinning cube)
        glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);
        glBindVertexArray(vaoCube);
        glEnable(GL_DEPTH_TEST);
        glUseProgram(sceneShaderProgram);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texKitten);
        //glActiveTexture(GL_TEXTURE1);
        //glBindTexture(GL_TEXTURE_2D, texPuppy);

        // Clear the screen to white
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Calculate transformation
        //auto t_now = std::chrono::high_resolution_clock::now();
        //float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();

        glm::mat4 model = glm::mat4(1.0f);       
	model = glm::rotate(
            model,
            glm::radians(0.0f),//1 * glm::radians(220.0f),
            glm::vec3(0.0f, 0.0f, 1.0f)
        );
        glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));

        // Draw cube
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glEnable(GL_STENCIL_TEST);

        // Draw floor
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glDepthMask(GL_FALSE);
        glClear(GL_STENCIL_BUFFER_BIT);

        glDrawArrays(GL_TRIANGLES, 36, 6);

        // Draw cube reflection
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
        glDepthMask(GL_TRUE);

        model = glm::scale(glm::translate(model, glm::vec3(0, 0, -1)), glm::vec3(1, 1, -1));
        glUniformMatrix4fv(uniModel, 1, GL_FALSE, glm::value_ptr(model));

        glUniform3f(uniColor, 0.3f, 0.3f, 0.3f);
	*/

	
	// SHADER VARIABLES
	//vec2 u_translate =vec2(0.0,0.0);// vec2(scaleTranslate.z,scaleTranslate.w);//vec2(0.0,0.0);
	//vec2 u_scale = vec2(1.12,2.0);//vec2(scaleTranslate.x,scaleTranslate.y);//vec2(2.0,2.0);//vec2(rotY.z,2.0);//vec2(2.0,2.0);
	//vec2 u_rotate = vec2(0,0);// vec2(rotY.x,rotY.y);
	//vec2 u_translate = vec2(scaleTranslate.z,scaleTranslate.w);//vec2(0.0,0.0);
	//vec2 u_scale = vec2(scaleTranslate.x,scaleTranslate.y);//vec2(2.0,2.0);//vec2(rotY.z,2.0);//vec2(2.0,2.0);
	//vec2 u_rotate = vec2(rotY.x,rotY.y);
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


	//v0.1
	glUniform3f(rotY, rotXY, rotYY, rotZY);//0.0f);
	glUniform4f(scaleTranslate,scaleX,scaleY,transX,transY);

        glDrawArrays(GL_TRIANGLES, 0, 36);
        glUniform3f(uniColor, 1.0f, 1.0f, 1.0f);

        glDisable(GL_STENCIL_TEST);

        // Bind default framebuffer and draw contents of our framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, texColorBufferA);//glBindFramebuffer(GL_FRAMEBUFFER, 0); //v0.2
        glBindVertexArray(vaoQuad);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(screenShaderProgram);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texKitten);

        glDrawArrays(GL_TRIANGLES, 0, 6);	

	// CONVERT TO OPENCV MAT
	out = get_ocv_img_from_gl_img(texColorBufferA);	

        //namedWindow("Raytrix feed", WINDOW_AUTOSIZE); 	
//	imshow("Raytrix feed", out); waitKey(1);
	// END CONVERT TO OPENCV MAT

        // Swap buffers
        //window.display();        
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

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_img.cols, tex_img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, tex_img.ptr());

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

	      cv::cvtColor(image, image, CV_RGB2BGR);
	

		glTexImage2D(GL_TEXTURE_2D,         // Type of texture
		             	0,                   // Pyramid level (for mip-mapping) - 0 is the top level
				GL_RGB,              // Internal colour format to convert to
		             	image.cols,          // Image width  i.e. 640 for Kinect in standard mode
		             	image.rows,          // Image height i.e. 480 for Kinect in standard mode
		             	0,                   // Border width in pixels (can either be 1 or 0)
				GL_RGB,              // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
				GL_UNSIGNED_BYTE,    // Image data type
				image.ptr());        // The actual image data itself

		//glDeleteTextures(1, &imageTexture);
     }
} 
/////////////========================== 360 CAMERA VIEW HANDLER GPU (END INIT REGION) =================================================/////////////

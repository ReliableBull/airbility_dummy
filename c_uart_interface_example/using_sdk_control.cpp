#include <opencv2/dnn.hpp>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include <stdint.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>
#include <string.h>

using std::string;
using namespace std;
using namespace cv;
using namespace chrono;

#define BUF_SIZE 59

// mouse click callbackFunc
void CallBackFunc( int event, int x, int y, int flags, void* userdata );

// init position
int i_x = 320;
int i_y = 240;

int c_x = -1;
int c_y = -1;

int _flag = 1;
int xStep = 1;
float yStep = 1.0;


std::string keyword("pointCalc");
std::string tname1 = keyword + "1";

int fd;

// get current Angle command
char command[5] = {0x3e,  0x3d , 0x00 , 0x3d , 0x00};

double focalLength = 4.9 ;// mm
struct termios newtio;

void init()
{
  fd=open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );  
  assert(fd != -1);
  memset(&newtio, 0, sizeof(struct termios));
  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag    = IGNPAR ;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = BUF_SIZE;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
}

int main()
{ 
  // uart init();
  init();
  // open /dev/video0
  VideoCapture cap("/dev/video0");
  //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  if (!cap.isOpened())
  {
    printf("Can't open the camera");
    return -1;
  }

  Mat img;

  while (1)
  {
    cap >> img;

    if (img.empty())
    {
      printf("empty image");
      return 0;
    }      
    
    namedWindow("GimbalImg" , 1);
    setMouseCallback("GimbalImg" , CallBackFunc , NULL);

    // pt1 : center point
    // pt0 : click point
    
    Point pt1 = Point(i_x, i_y);
    circle(img, pt1, 1, Scalar(0, 255, 0), 0, 0, 0);


    if(c_x != -1)
    {
      Point pt0 = Point(c_x,c_y );  
      circle(img, pt0, 1, Scalar(0, 0, 255), 0, 0, 0);
    }

    double pixelSize = 8.466582 / 640;

    double d = (c_x - i_x) * pixelSize;


    //double aa = focalLength / d ;
    double theta = atan2(d,focalLength);

    imshow("GimbalImg", img);    
  
    int ch = waitKey(25);

  }  

  return 0;
}

void CallBackFunc( int event, int x, int y, int flags, void* userdata )
{
    // Mouse LeftButtonDown Event
    if ( event == EVENT_LBUTTONDOWN )
    {     
      char buf[BUF_SIZE] = {0x00,};
      
      // command
      char setAngle[21] = {0x00,};

      c_x = x;
      c_y = y;

      std::cout<<"c_x : " << c_x <<" c_y : " << c_y <<std::endl;

      fd=open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );  
      
      assert(fd != -1);
      
      memset(&newtio, 0, sizeof(struct termios));
      newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
      newtio.c_iflag    = IGNPAR ;
      newtio.c_oflag = 0;
      newtio.c_lflag = 0;
      newtio.c_cc[VTIME] = 0;
      newtio.c_cc[VMIN] = BUF_SIZE;

      tcflush(fd, TCIFLUSH);
      tcsetattr(fd, TCSANOW, &newtio);

      write(fd, command, 5); // send command
      int res;
      
      res = read(fd,buf,BUF_SIZE);

      // current angle
      int16_t pitch = (int16_t)((buf[25]<<8)) + buf[24];
      int16_t yaw = (int16_t)((buf[43]<<8)) + buf[42];

      
      // calc click angle
      double width_pixelSize = (5.566582 ) / 640;
      double tt = c_x - i_x ;
      
      tt = -tt;


      double w_d = (c_x - i_x + tt * 0.08) * width_pixelSize;
      double w_theta = atan2(w_d,focalLength);

      double height_pixelSize = 2.566582 / 480;
      double h_d = (c_y - i_y) * height_pixelSize;
      double h_theta = atan2(h_d,focalLength);

      double newPitch = ceil(pitch*0.02197f * 100.0f) /100.0f  ;

      double newYaw = ceil(yaw * 0.02197f *100.0f)/100.0f;

      newYaw += (w_theta *180 / 3.14);
      newPitch += (h_theta *180 / 3.14);

      std::cout<<"x differ : "<< (c_x - i_x) <<std::endl;
      std::cout<<"y differ : "<< (c_y - i_y) <<std::endl;
      std::cout<<"current pitch angle : " <<pitch * 0.02197 << "  new pitch angle : " << newPitch <<std::endl;
      std::cout<<"current yaw angle: " <<yaw * 0.02197 << "  new yaw angle : " << newYaw <<std::endl;
    

      newPitch = newPitch / 0.02197f;
      newYaw = newYaw / 0.02197f;

      setAngle[0] = 0xff;
      setAngle[1] = 0x01;
      setAngle[2] = 0x0f;
      setAngle[3] = 0x10;

      setAngle[5] = 0x02;
      setAngle[6] = 0x02;

      setAngle[11] = 0x20; // speed unit 0.122074

      setAngle[14] = (char)(((int16_t)newPitch>>8));
      setAngle[13] = (char)((int16_t)newPitch & 0x00ff);
      
      setAngle[15] = 0x20; // speed unit 0.122074

      setAngle[18] = (char)(((int16_t)newYaw>>8));
      setAngle[17] = (char)((int16_t)newYaw & 0x00ff);    

      int16_t tmp = 0;

      for(int i= 5 ; i < 19 ; i++)
      {        
          tmp+= setAngle[i];
      }

      // cal checksum
      tmp = tmp &0xFF;

      setAngle[19] = tmp;  
     
      std::cout<<std::endl;

      // send MSG
      write(fd, setAngle, 20);
      close(fd);
    
    } 
}
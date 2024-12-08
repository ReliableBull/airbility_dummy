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
#include <thread>
#include <chrono>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/tracking/kalman_filters.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/video/tracking.hpp"

#include <stdint.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>

#define BUF_SIZE 59

using std::string;
using namespace std;
using namespace cv;
using namespace chrono;
using namespace cv::ml;

Ptr<Tracker> tracker; 

struct initRoi {
  //initial coordination based on EVENT_LBUTTONDOWN
  int initX;
  int initY;

  // actual coordination 
  int actualX;
  int actualY;
  
  // final coordinates 
  int finalX;
  int finalY;
  int finalWidth; 
  int finalHeight;

  int displayRoi; 
  int trackerReady; 
  int initTracker;
}SelectedRoi;

static void CallBackF(int event, int x, int y, int flags, void* img) {

  Mat& imge = *((Mat*)img);

  if (event == EVENT_RBUTTONDOWN) {
    //cout << "right button " << endl;
    return;
  }

  if (event == EVENT_LBUTTONDOWN) {
    SelectedRoi.initX = x;
    SelectedRoi.initY = y;
    SelectedRoi.displayRoi = 1;   
    //cout << "left button DOWN" << endl; 
    return;
  }

  if (event == EVENT_LBUTTONUP) {
    SelectedRoi.finalWidth = x- SelectedRoi.initX ;
    SelectedRoi.finalHeight = y-SelectedRoi.initY ;
    SelectedRoi.initTracker = 1;    
    //cout << "left button UP" << endl;
    return;
  }
  if (event == EVENT_MOUSEMOVE) {     
    //cout << "event mouse move"<< endl; 
    SelectedRoi.actualX = x;
    SelectedRoi.actualY = y;
    return;
  }
}

void KeyDownEvent(int ch);
void CallBackFunc( int event, int x, int y, int flags, void* userdata );

// init position
int ix = 320;
int iy = 240;

// init position
int i_x = 320;
int i_y = 240;


int c_x = -1;
int c_y = -1;

int _flag = 1;
int xStep = 1;
float yStep = 1.0;

///PointCalc pointCalc;
std::string keyword("pointCalc");
std::string tname1 = keyword + "1";


//// sdk 
// get current Angle command
char currnetAngleCommand[5] = {0x3e,  0x3d , 0x00 , 0x3d , 0x00};
double focalLength = 4.9 ;// mm
struct termios newtio;
int fd;


void countProc();
void init();
void move_center( int x, int y );

int main()
{
 
 init();
  // open /dev/video0
  VideoCapture cap("/dev/video0");
   //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  

  SelectedRoi.displayRoi = 0;
  SelectedRoi.trackerReady = 0;
  SelectedRoi.initTracker = 0;

  TrackerCSRT::Params params ;
  params.psr_threshold = 0.00000000000001;
  params.use_channel_weights = true;
  params.use_color_names = true;
  tracker = TrackerCSRT::create(params);
  
  //tracker = TrackerCSRT::create();
  //tracker = TrackerMOSSE::create();
  // write output to file 
  VideoWriter outputVideo;
  //outputVideo.open("video.vmw", VideoWriter::fourcc('W', 'M', 'V', '2'),
  //cap.get(CAP_PROP_FPS), Size(640, 480), true);

  if (!cap.isOpened())
  {
    printf("Can't open the camera");
    return -1;
  }


  Mat img;
  float param1 = 9; //gimbal left right
  float param2 = 1100; // gimbal up down
  float param3 = 10;
  
  double pre_res = -9999;
  int time_cnt = 0;
  while (1)
  {
    cap >> img;
    
   // if(time_cnt != 0)
   //   time_cnt--;    

    if (img.empty())
    {
      printf("empty image");
      return 0;
    }
    
    namedWindow("Video", WINDOW_AUTOSIZE);
    setMouseCallback("Video", CallBackF, 0);

    c_x =640 /2;
    c_y = 480 /2;

    Point pt1 = Point(c_x, c_y);
    circle(img, pt1, 1, Scalar(0, 255, 0), 0, 0, 0);

      if (SelectedRoi.displayRoi != 0) {
        rectangle(img, Rect(SelectedRoi.initX, SelectedRoi.initY, SelectedRoi.actualX-SelectedRoi.initX, SelectedRoi.actualY - SelectedRoi.initY), Scalar(255, 255, 255), 4, 8, 0);        
      }
    
      if (SelectedRoi.initTracker == 1) {
        tracker->init(img, Rect2d(SelectedRoi.initX, SelectedRoi.initY,
        SelectedRoi.finalWidth, SelectedRoi.finalHeight));
        SelectedRoi.trackerReady = 1;
      }
      if (SelectedRoi.trackerReady == 1) {
        Rect2d track;
        tracker->update(img, track);
        //cout<<track.y<<endl;
        // draw rectangle
        rectangle(img, track, Scalar(0, 0, 255), 4, 8, 0);
        ix = (track.x + track.width/2);
        iy = (track.y + track.height/2);
        Point pt0 = Point(ix , iy);
        circle(img , pt0 , 1 , Scalar(0,0,255) , 0,0,0);
      }
  

    if(ix != 0 )
    {
      int t_x = ix;
      int t_y = iy;

      Point pt0 = Point(t_x,t_y );
      circle(img, pt0, 1, Scalar(0, 0, 255), 0, 0, 0);

      line(img, pt1, pt0, Scalar(0, 0, 255),
         2, LINE_AA);

      double res = norm(pt1 - pt0);
    
      //std::cout<<res<<":::"<<pre_res<<std::endl;
      std::cout<<time_cnt;
      if(res > 30 && time_cnt == 0)
      {
        time_cnt = 30;
        //temp = 1;
        pre_res = res;
       /* if(t_x == c_x)
        {
          // mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
          //   port->write_message(msg);
        } 
        else if(t_x > c_x)
        {
       //   cout<<"right"<<endl;
       
        }        
        else
         {
         
           cout<<"left"<<endl;

         }
       }*/
        move_center(t_x , t_y);
      }
    }    


    imshow("Video", img);
  

  int ch = waitKey(25);


  }
    
  return 0;
}

void KeyDownEvent(int ch)
{
  if(ch=='a')
    cout << "Key down => a" << endl;
  else if(ch=='b')
    cout << "Key down => b" << endl;
  else if(ch=='c')
    cout << "Key down => c" << endl;
  else if(ch=='d')
    cout << "Key down => d" << endl;
  else if(ch=='A')
    cout << "Key down => A" << endl;
  else if(ch=='B')
    cout << "Key down => B" << endl;
  else if(ch=='C')
    cout << "Key down => C" << endl;
  else if(ch=='D')
    cout << "Key down => D" << endl;
  else if(ch==8) // ASCII value of BACKSPACE 
    cout << "Key down => BACKSPACE" << endl;
  else if(ch==27) // ASCII value of ESC
    cout << "Key down => ESC" << endl;
  else if(ch==32) // ASCII value of SPACE
    cout << "Key down => SPACE" << endl;
  else if(ch==2490368) // ASCII value of UP ARROW
    cout << "Key down => UP ARROW" << endl;
  else if(ch==2621440) // ASCII value of DOWN ARROW
    cout << "Key down => DOWN ARROW" << endl;
  else
    cout << "Key down => Others" << endl;

}


void countProc()
{
  int i = 0;
  for(;;)
  {
   cout<<i++<<endl;
   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

// sdk_init
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

void move_center( int x, int y )
{
    
      // response buffer   
      char buf[BUF_SIZE] = {0x00,};
      
      // command
      char setAngle[21] = {0x00,};

      int res;

      c_x = x;
      c_y = y;

      // init
      //init();

      // send 'get current Angle command'
      write(fd, currnetAngleCommand, 5); 
      
      // get current Angle information
      res = read(fd,buf,BUF_SIZE);

      // calc current angle
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

      //std::cout<<"x differ : "<< (c_x - i_x) <<std::endl;
      //std::cout<<"y differ : "<< (c_y - i_y) <<std::endl;
      //std::cout<<"current pitch angle : " <<pitch * 0.02197 << "  new pitch angle : " << newPitch <<std::endl;
      //std::cout<<"current yaw angle: " <<yaw * 0.02197 << "  new yaw angle : " << newYaw <<std::endl;
    
      newPitch = newPitch / 0.02197f;
      newYaw = newYaw / 0.02197f;

      setAngle[0] = 0xff;
      setAngle[1] = 0x01;
      setAngle[2] = 0x0f;
      setAngle[3] = 0x10;
      setAngle[5] = 0x02;
      setAngle[6] = 0x02;
      setAngle[11] = 0x20; // speed unit 0.122074

      // new pitch angle set
      setAngle[14] = (char)(((int16_t)newPitch>>8));
      setAngle[13] = (char)((int16_t)newPitch & 0x00ff);
      
      setAngle[15] = 0x20; // speed unit 0.122074

      // new yaw angle set
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
          
      // send MSG
      write(fd, setAngle, 20);
     
      //close(fd);
     
}


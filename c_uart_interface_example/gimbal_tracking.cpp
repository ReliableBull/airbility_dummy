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
#include <common/mavlink.h>
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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <cstring>


#include "serial_port.h"
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
    //tracker->clear();
    tracker = TrackerMOSSE::create();
    SelectedRoi.initX = x;
    SelectedRoi.initY = y;
    SelectedRoi.displayRoi = 1;   
    cout << "left button DOWN" << endl; 
    return;
  }

  if (event == EVENT_LBUTTONUP) {
    SelectedRoi.finalWidth = x- SelectedRoi.initX ;
    SelectedRoi.finalHeight = y-SelectedRoi.initY ;
    //SelectedRoi.finalWidth = 80 ;
    //SelectedRoi.finalHeight = 80 ;
    SelectedRoi.initTracker = 1;   
    SelectedRoi.displayRoi = 0; 
    cout << "left button UP" << endl;
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

// thread func

void error_handling(const char *message);
void init();
void *getData(void *arg);
int sock;

int x, y;

// end thread func

// init position
int ix = 320;
int iy = 240;

int c_x;
int c_y;

int _flag = 1;
int xStep = 1;
float yStep = 1.0;

///PointCalc pointCalc;
std::string keyword("pointCalc");
std::string tname1 = keyword + "1";

// PID Control

double KP = 2.8f;
double KI = 0.0f;
double KD = 0.0000002f;
double error_last = 0.0f;


double KP_y = 2.0f;
double KI_y = 0.0f;
double KD_y = 0.0000002f;
double errorY_last = 0.0f;

// -----------


// t : tracking
// m : manual

int controlMode = -1;
int manualMode = -1;

void countProc();

int main()
{
 
  //Serial_Port *port = new Serial_Port("/dev/ttyTHS2", 921600);
  Serial_Port *port = new Serial_Port("/dev/ttyUSB0", 921600);

  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  //mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  mavlink_system.compid = MAV_COMP_ID_GIMBAL;     ///< The component sending the message is the IMU, it could be also a Linux process
 
  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
 
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t *pbuf = buf;
   
  //uint8_t target_component_id = MAV_COMP_ID_AUTOPILOT1;
  uint8_t target_component_id = MAV_COMP_ID_QX1_GIMBAL;
  uint8_t command = 183;
  uint8_t command2 = MAV_CMD_DO_MOUNT_CONFIGURE;
  uint8_t command3 = MAV_CMD_DO_MOUNT_CONTROL;
  uint8_t target_system_id = 1;
  int target_comonetId = 1;
  uint8_t confirmation = 0;
  
  // open /dev/video0
  VideoCapture cap("/dev/video0");
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  

  SelectedRoi.displayRoi = 0;
  SelectedRoi.trackerReady = 0;
  SelectedRoi.initTracker = 0;
  

  // set CSRT
  
  /*TrackerCSRT::Params params ;
  params.psr_threshold = 0.00000000000001;
  params.use_channel_weights = true;
  params.use_color_names = true;
  tracker = TrackerCSRT::create(params);
  */

  //tracker = TrackerCSRT::create();
  //tracker = TrackerMOSSE::create();
  //tracker = TrackerKCF::create();
  // write output to file 
  VideoWriter outputVideo;
  //outputVideo.open("video.vmw", VideoWriter::fourcc('W', 'M', 'V', '2'),
//  cap.get(30, Size(1920, 1080), true);

  if (!cap.isOpened())
  {
    printf("Can't open the camera");
    return -1;
  }

  // gstreamer

  string gst_out = "appsrc ! videoconvert ! v4l2src speed-preset=ultrafast bitrate=600 key-int-max=40 ! rtspclientsink location=rtsp://118.67.132.33:8554/mystream protocols=tcp";
  //string gst_out = "appsrc ! videoconvert ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=40 ! rtspclientsink location=rtsp://118.67.132.33:8554/mystream protocols=tcp";

  cv::VideoWriter writer;

  writer.open(gst_out, cv::CAP_GSTREAMER, 0, 30, cv::Size(1920,1080),true);
  //writer.open(gst_out, cv::CAP_GSTREAMER, 0, 30, cv::Size(640,480),true);

  if (!writer.isOpened()) {
      cerr <<"VideoWriter not opened"<<endl;
      exit(-1);
  }

  Mat img;
  float param1 = 9; //gimbal left right
  float param2 = 1100; // gimbal up down
  float param3 = 10;
  
  port->start();

  mavlink_msg_command_long_pack(mavlink_system.sysid,  mavlink_system.compid, &msg , target_system_id , target_comonetId , MAV_CMD_DO_MOUNT_CONFIGURE , confirmation , MAV_MOUNT_MODE_MAVLINK_TARGETING , 0 , 0 ,0,2,0,0);
  port->write_message(msg);
  double outputX , outputY;
  outputX = outputY = 1500; // init;


  // connect server
  pthread_t tA;
  
  init();

  pthread_create(&tA, NULL, getData, NULL);
    
  // end connect server
  while (1)
  {
    cap >> img;
        
    if (img.empty())
    {
      printf("empty image");
      return 0;
    }
    
    //namedWindow("Video", WINDOW_AUTOSIZE);

    //setMouseCallback("Video", CallBackF, 0);

    c_x = 640 /2;
    c_y = 480 /2;

    Point pt1 = Point(c_x, c_y);

    circle(img, pt1, 1, Scalar(0, 255, 0), 0, 0, 0);
    if(controlMode == 1)
    {
        if (SelectedRoi.displayRoi != 0) {
          printf("1");
          //rectangle(img, Rect(SelectedRoi.initX, SelectedRoi.initY, SelectedRoi.actualX-SelectedRoi.initX, SelectedRoi.actualY - SelectedRoi.initY), Scalar(255, 255, 255), 2, 30, 0);        
        }    

        if (SelectedRoi.initTracker == 1) {
          printf("2");
          tracker->init(img, Rect2d(SelectedRoi.initX, SelectedRoi.initY,
          SelectedRoi.finalWidth, SelectedRoi.finalHeight));
          SelectedRoi.trackerReady = 1;
        }

        if (SelectedRoi.trackerReady == 1) {
          
//          printf("3");
          Rect2d track;

          bool trackerCheck = tracker->update(img, track);
//          printf("return value : %d \n", trackerCheck);
          SelectedRoi.initTracker = 0;
          if(!trackerCheck)
          {
            //tracker->clear();
            SelectedRoi.displayRoi = 1;
            SelectedRoi.initTracker = 0;
            SelectedRoi.trackerReady = 0;
            ix = 0;
            
            mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
             port->write_message(msg); 
            mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
             port->write_message(msg); 

            continue;
          }
          //cout<<"tracker : " << track.x << "::::" << track.y<<endl;
          rectangle(img, track, Scalar(0, 0, 255), 4, 8, 0);

          // target center point
          ix = (track.x + track.width/2);
          iy = (track.y + track.height/2);

          Point pt0 = Point(ix , iy);
          circle(img , pt0 , 1 , Scalar(0,0,255) , 0,0,0);
        }  

      if(ix != 0)
      {
        int t_x = ix;
        int t_y = iy;

        Point pt0 = Point(t_x,t_y );

        circle(img, pt0, 1, Scalar(0, 0, 255), 0, 0, 0);

        line(img, pt1, pt0, Scalar(0, 0, 255),2, LINE_AA);

        // calc error
        double res = norm(pt1 - pt0);
        
        //std::cout<<res<<std::endl;
        
        if(fabs(t_y - c_y) > 10)
        {
          double output ;
          
          double error = t_y - c_y;

          // 221025
          double derivative_error = (error - errorY_last) ;
          errorY_last = derivative_error;

          if(error > 0) // right
          {
            output = 1500 + 100 + (KP_y*error + KD_y*derivative_error);

            if(output >= 2000)
            {
              output = 2000;
            }            
            else if(output <= 1600)
            {
              output = 1600;
            }
          }
          else // left
          {
            output = 1500 - 100 + (KP_y*error + KD_y*derivative_error);  

            if(output >= 1400)
            {
              output = 1400;
            }            
            else if(output <= 1000)
            {
              output = 1000;
            }
          }

          //cout<<"output : " << output<<endl;
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , output , 0,0,0,0,0);
          port->write_message(msg);     
        }
        else
        {
          // up, down
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
          port->write_message(msg);
        }

        if(fabs(t_x - c_x) > 10)
        {
          // x axis 
          // right 1600 ~ 2000
          // left 1000 ~ 1400
          double output ;
          
          double error = t_x - c_x;

          // 221025
          double derivative_error = (error - error_last) ;
          error_last = derivative_error;

          if(error > 0) // right
          {
            output = 1500 + 100 + (KP*error + KD*derivative_error);

            if(output >= 2000)
            {
              output = 2000;
            }            
            else if(output <= 1600)
            {
              output = 1600;
            }
          }
          else // left
          {
            output = 1500 - 100 + (KP*error + KD*derivative_error);  

            if(output >= 1400)
            {
              output = 1400;
            }            
            else if(output <= 1000)
            {
              output = 1000;
            }
          }

          //cout<<"output : " << output<<endl;
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , output , 0,0,0,0,0);
          port->write_message(msg);        
          
        }
        else
        {             
          // right, left
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
          port->write_message(msg);
          //cout<<"Done"<<endl;
        }
      }
    }
    else if(controlMode == 2)
    {
      switch(manualMode)
      {
        case 1:
                  mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1300 , 0,0,0,0,0);
                        port->write_message(msg);
        break;
  
        case 2:
                mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1700 , 0,0,0,0,0);
                      port->write_message(msg);
        break;
  
        case 3:
              mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1300 , 0,0,0,0,0);
                    port->write_message(msg);
        break;
  
        case 4:
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1700 , 0,0,0,0,0);
                port->write_message(msg);
        break;
      case 5:
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
          port->write_message(msg);
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
          port->write_message(msg);
        break;
      }
    }    

    //imshow("Video", img);
    writer.write(img);
    int ch = waitKey(25);

  }
  
  port->stop();

  return 0;
}

void CallBackFunc( int event, int x, int y, int flags, void* userdata )
{
    if ( event == EVENT_LBUTTONDOWN )
    {

      ix = x;
      iy = y;

      std::cout<<"c_x : " << c_x <<" c_y : " << c_y <<std::endl<<"t_x : " << ix <<" t_y : " <<iy << std::endl;

      //pointCalc.stop_thread(tname1);
      // create and kill thread 1
      //pointCalc.start_thread(tname1);

    
       std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == EVENT_LBUTTONUP )
    {
        //std::cout << "Left button of the mouse is released - position (" << x << ", " << y << ")" << std::endl;
    }    
 
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

// thread Func


void *getData(void *arg)
{
    char message[30];
    int str_len;
    
    while(1)
    {

      str_len=read(sock, message, sizeof(message)-1);
      if(str_len==-1)
          error_handling("read() error!");
  
      printf("Message from server: %s \n", message);
    int flag = 0;
    char *result = strtok(message , ":");


     if(!strcmp(result,"t") )
        controlMode = 1;
      else if(!strcmp(result,"m") )
        controlMode = 2;

    while(result !=NULL)
    {
      std::cout << result << std::endl;
     std::cout<<controlMode<<std::endl;

      if(controlMode == 1)
      {
        switch(flag)
        {
          case 1:
            tracker = TrackerMOSSE::create();
            SelectedRoi.initX = atoi(result);
          break;
          case 2:
            SelectedRoi.initY = atoi(result);
          break;

          case 3:
            SelectedRoi.actualX = atoi(result);
          break;

          case 4:
            SelectedRoi.actualY = atoi(result);
          break;

          case 5:
            SelectedRoi.finalWidth = atoi(result);
          break;

          case 6:
            SelectedRoi.finalHeight = atoi(result);
            SelectedRoi.initTracker = 1;   
            SelectedRoi.displayRoi = 0;
          break;
       }
      }
      else if(controlMode == 2)
      {
        if(flag == 1)
          manualMode = atoi(result);
      }
      
      
      result = strtok(NULL,":");
      flag++;
    }
       // string str1(message);
  //  std::cout<<str1<<std::endl;
  memset(message, 0, sizeof(message));
    }
    close(sock);
}

void init()
{
  char *hostID = "118.67.132.33";
  int port = 9999;

  x = y = -1;
  sock=socket(PF_INET, SOCK_STREAM, 0);
    if(sock == -1)
        error_handling("socket() error");
  
    // 클라이언트와 마찬가지로 주소정보를 초기화
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family=AF_INET;
    serv_addr.sin_addr.s_addr=inet_addr(hostID);
//    serv_addr.sin_port=htons(atoi(argv[2]));
    serv_addr.sin_port=htons(port);
    
    // 서버의 주소정보로 클라이언트 소켓이 연결요청을 한다.
    if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))==-1) 
        error_handling("connect() error!");
}

void error_handling(const char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

// end thread Func

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
#include <common/mavlink.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>


#include "serial_port.h"
using std::string;
using namespace std;
using namespace cv;
using namespace chrono;


void KeyDownEvent(int ch);
void CallBackFunc( int event, int x, int y, int flags, void* userdata );

// init position
int ix = 320;
int iy = 240;

int c_x;
int c_y;

int _flag = 1;

int xStep = 1;
float yStep = 1.0;

class PointCalc {
public:
    void sleep_for(const std::string &tname, int num)
    {
        //prctl(PR_SET_NAME,tname.c_str(),0,0,0);        
//        int cnt = 0;

        while(1)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          //sleep(1);    // sec
            //std::cout<<cnt++<<std::endl;

           if(ix != c_x)
           {
            if(ix > c_x)
            {
              if(ix - xStep <c_x)
                ix = c_x;
              else
                ix -= xStep;
            }  
            else
            {
              if(ix + xStep >c_x)
                ix = c_x;
              else
                ix +=xStep;
            }
           }
            
           /*if(iy != c_y)
           {
            if(iy > c_y)
            {
              if(iy - yStep <c_y)
                iy = c_y;
              else
                iy -= yStep;
            }              
            else
            {
              if(iy + yStep > c_y)
                iy = c_y;
              else
                iy += yStep;
            }              
           }       */
        }        
    }

    void sleep_for2(const std::string &tname, int num)
    {
        //prctl(PR_SET_NAME,tname.c_str(),0,0,0);        
//        int cnt = 0;

        while(1)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(40));
          //sleep(1);    // sec

            //std::cout<<cnt++<<std::endl;
/*
           if(ix != c_x)
           {
            if(ix > c_x)
            {
              if(ix - xStep <c_x)
                ix = c_x;
              else
                ix -= xStep;
            }  
            else
            {
              if(ix + xStep >c_x)
                ix = c_x;
              else
                ix +=xStep;
            }
           }
  */          
           if(iy != c_y)
           {
            if(iy > c_y)
            {
              if(iy - yStep <c_y)
                iy = c_y;
              else
                iy -= yStep;
            }              
            else
            {
              if(iy + yStep > c_y)
                iy = c_y;
              else
                iy += yStep;
            }              
           }          
        }        
    }



    void start_thread(const std::string &tname)
    {
        std::thread * thrd = new std::thread(&PointCalc::sleep_for, this, tname, 3600);
        std::thread * yThrd = new std::thread(&PointCalc::sleep_for2, this, "1", 3600);
        tm_[tname] = thrd->native_handle();
        tm_["1"] = yThrd->native_handle();
        thrd->detach();
        yThrd->detach();
        //tm_[tname] = thrd;
        std::cout << "Thread " << tname << " created:" << std::endl;
    }

    void stop_thread(const std::string &tname)
    {
        ThreadMap::const_iterator it = tm_.find(tname);
        if (it != tm_.end()) {
            //delete it->second; // thread not killed
            //it->second->std::thread::~thread(); // thread not killed
            pthread_cancel(it->second);
            tm_.erase(tname);
            std::cout << "Thread " << tname << " killed:" << std::endl;
        }

         it = tm_.find("1");
        if (it != tm_.end()) {
            //delete it->second; // thread not killed
            //it->second->std::thread::~thread(); // thread not killed
            pthread_cancel(it->second);
            tm_.erase(tname);
            std::cout << "Thread " << tname << " killed:" << std::endl;
        }
    }

private:
    //typedef std::unordered_map<std::string, std::thread*> ThreadMap;
    typedef std::unordered_map<std::string, pthread_t> ThreadMap;
    ThreadMap tm_;
};


PointCalc pointCalc;
std::string keyword("pointCalc");
std::string tname1 = keyword + "1";

void countProc();

int main()
{
  
  Serial_Port *port = new Serial_Port("/dev/ttyTHS2", 921600);

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
  //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  if (!cap.isOpened())
  {
    printf("Can't open the camera");
    return -1;
  }

  Mat img;
  float param1 = 9; //gimbal left right
  float param2 = 1100; // gimbal up down
  float param3 = 10;
  
  port->start();

  mavlink_msg_command_long_pack(mavlink_system.sysid,  mavlink_system.compid, &msg , target_system_id , target_comonetId , MAV_CMD_DO_MOUNT_CONFIGURE , confirmation , MAV_MOUNT_MODE_MAVLINK_TARGETING , 0 , 0 ,0,2,0,0);
  port->write_message(msg);
     

   
  while (1)
  {
    cap >> img;
        
    if (img.empty())
    {
      printf("empty image");
      return 0;
    }
    

    Mat img_hsv;

    cvtColor(img , img_hsv , COLOR_BGR2HSV);

    Mat yellow_mask , yellow_image;

    Scalar lower_yellow = Scalar(20, 20, 100);
    Scalar upper_yellow = Scalar(32, 255, 255);

    inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
    bitwise_and(img, img, yellow_image, yellow_mask);

    erode(yellow_mask, yellow_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(yellow_mask, yellow_mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    //라벨링 
    Mat img_labels, stats, centroids;
    int numOfLables = connectedComponentsWithStats(yellow_mask, img_labels,
      stats, centroids, 8, CV_32S);

    //영역박스 그리기
    int max = -1, idx = 0;
    for (int j = 1; j < numOfLables; j++) {
      int area = stats.at<int>(j, CC_STAT_AREA);
      if (max < area)
      {
        max = area;
        idx = j;
      }
    }

    int left = stats.at<int>(idx, CC_STAT_LEFT);
    int top = stats.at<int>(idx, CC_STAT_TOP);
    int width = stats.at<int>(idx, CC_STAT_WIDTH);
    int height = stats.at<int>(idx, CC_STAT_HEIGHT);


    int t_x = -1;
    int t_y = -1;
   
   
    //cout<<left << " right : " << right <<endl;
    
    
    namedWindow("GimbalImg" , 1);
    setMouseCallback("GimbalImg" , CallBackFunc , NULL);

//    c_x =img.cols/2;
 //   c_y = img.rows/2;

    c_x =640 /2;
    c_y = 480 /2;

    Point pt1 = Point(c_x, c_y);
    circle(img, pt1, 1, Scalar(0, 255, 0), 0, 0, 0);

    /*if(ix != -1)
    {
      Point pt1 = Point(ix, iy);
      circle(img, pt1, 1, Scalar(0, 0, 255), 0, 0, 0);


    }*/


    if(ix != 0)
    {
     
     /*if(ix != c_x)
     {
      if(ix > c_x)
          ix --;
      else
          ix ++;
     }
      
     if(iy != c_y)
     {
      if(iy > c_y)
          iy --;
      else
          iy++;
     }*/
       //iy --;
     // rectangle(img, Point(left, top), Point(left + width, top + height),
      //Scalar(0, 0, 255), 1);

      t_x = ix;
      t_y = iy;

      Point pt0 = Point(t_x,t_y );
      circle(img, pt0, 1, Scalar(0, 0, 255), 0, 0, 0);

      line(img, pt1, pt0, Scalar(0, 0, 255),
         2, LINE_AA);

      double res = norm(pt1 - pt0);


      

      if(res > 0)
      {
        if(t_x == c_x)
        {
          std::cout<<"test"<<std::endl;  
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
          port->write_message(msg);
        } 
        else if(t_x > c_x)
        {
          cout<<"right"<<endl;
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1900 , 0,0,0,0,0);
          port->write_message(msg);        
        }        
        else
         {
           mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1400 , 0,0,0,0,0);
           port->write_message(msg); 
           cout<<"left"<<endl;

         }

        if( t_y == c_y)
        {
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
          port->write_message(msg);

        }
        else if(t_y > c_y)
        {
          cout<<"down"<<endl;

          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1800 , 0,0,0,0,0);
          port->write_message(msg);
        }
        else
        {
          cout<<"up"<<endl;
          mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1200 , 0,0,0,0,0);
          port->write_message(msg);
        }
      }
      else
      {
        mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
        port->write_message(msg);
        
        mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
        port->write_message(msg);
        cout<<"Done"<<endl;
      }
    }    
    imshow("GimbalImg", img);
    imshow("yellow_image", yellow_image);
    imshow("yellow_mask", yellow_mask);

    /*if (waitKey(25) == 27)
      break;*/

 int ch = waitKey(25);

    //port->read_message(msg);

  }
  
  //thread1.join();
  port->stop();



  /*
  float param1 = 9; //gimbal left right
  float param2 = 1100; // gimbal up down

  port->start();
  mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , param2 , 0,0,0,0,0);

  port->write_message(msg);
  
  sleep(10);
  param2 = 1900;
  mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , param2 , 0,0,0,0,0);
  port->write_message(msg);

  port->stop();
*/

  return 0;
}

void CallBackFunc( int event, int x, int y, int flags, void* userdata )
{
    if ( event == EVENT_LBUTTONDOWN )
    {

      ix = x;
      iy = y;

      std::cout<<"c_x : " << c_x <<" c_y : " << c_y <<std::endl<<"t_x : " << ix <<" t_y : " <<iy << std::endl;



      pointCalc.stop_thread(tname1);
      // create and kill thread 1
      pointCalc.start_thread(tname1);

    
       // std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
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
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

#include "serial_port.h"
#include <common/mavlink.h>

using std::string;
using namespace std;
using namespace cv;
using namespace chrono;

#define BUF_SIZE 59

// mouse click callbackFunc
void CallBackFunc( int event, int x, int y, int flags, void* userdata );
void setInitial(int inputPitch , int inputYaw);


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
char get_current_angle_command[5] = {0x3e,  0x3d , 0x00 , 0x3d , 0x00};

double focalLength = 4.9 ;// mm
struct termios newtio;


// Mavlink 
Serial_Port *port;


// PID Control
double KP = 2.8f;
double KI = 0.0f;
double KD = 0.0000002f;
double error_last = 0.0f;

double KP_y = 5.0f;
double KI_y = 0.0f;
double KD_y = 0.0000002f;
double errorY_last = 0.0f;


void init()
{
 
  port = new Serial_Port("/dev/ttyUSB0", 921600);
  fd=open("/dev/ttyUSB1", O_RDWR | O_NOCTTY );  
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


int main(int argc , char* argv[])
{ 
  // uart init();
  init();
  int setPitch = 0;
  int setYaw = 0;
  setPitch = atoi(argv[1]);
  setYaw = atoi(argv[2]);

  printf("%d %d\n" , setPitch , setYaw);

  //return 0;
  // open /dev/video0
/*  VideoCapture cap("/dev/video0");
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
    
    /*Point pt1 = Point(i_x, i_y);
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
*/

      setInitial(setPitch , setYaw);  
  return 0;
}


void CallBackFunc( int event, int x, int y, int flags, void* userdata )
{
    // Mouse LeftButtonDown Event
    if ( event == EVENT_LBUTTONDOWN )
    {     
      /*char buf[BUF_SIZE] = {0x00,};
      
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

      write(fd, get_current_angle_command, 5); // send command
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

      newYaw   += (w_theta *180 / 3.14);
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
      
      close(fd);  */

      //usleep(100000);

 //     setInitial();  
    } 
}


void setInitial(int inputPitch , int inputYaw)
{
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
  uint8_t _buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t *pbuf = _buf;
   
  //uint8_t target_component_id = MAV_COMP_ID_AUTOPILOT1;
  uint8_t target_component_id = MAV_COMP_ID_QX1_GIMBAL;
  uint8_t command = 183;
  uint8_t command2 = MAV_CMD_DO_MOUNT_CONFIGURE;
  uint8_t command3 = MAV_CMD_DO_MOUNT_CONTROL;
   uint8_t target_system_id = 1;
  int target_comonetId = 1;
  uint8_t confirmation = 0;


  // get current angle
  char buf[BUF_SIZE] = {0x00,};
      
      // command
  char setAngle[21] = {0x00,};

  c_x = 0;
  c_y = 0;

  //std::cout<<"c_x : " << c_x <<" c_y : " << c_y <<std::endl;

  fd=open("/dev/ttyUSB1", O_RDWR | O_NOCTTY );  
  //port = new Serial_Port("/dev/ttyTHS2", 921600);
  port->start();

  mavlink_msg_command_long_pack(mavlink_system.sysid,  mavlink_system.compid, &msg , target_system_id , target_comonetId , MAV_CMD_DO_MOUNT_CONFIGURE , confirmation , MAV_MOUNT_MODE_MAVLINK_TARGETING , 0 , 0 ,0,2,0,0);
  port->write_message(msg);
    
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
  float param1 = 9; // gimbal up down
  float param2 = 1100; 
  float param3 = 10;//gimbal left right

  std::cout<<"OPEN"<<std::endl;

  int pitch_flag = 0;
  int yaw_flag = 0;

  while(1)
  {
    std::cout<<"OPEN1"<<std::endl;
    usleep(1000);
    write(fd, get_current_angle_command, 5); // send command

    std::cout<<"OPEN2"<<std::endl;
    int res;
      
    res = read(fd,buf,BUF_SIZE);
  std::cout<<"OPEN3"<<std::endl;
    // current angle
    int16_t pitch = (int16_t)((buf[25]<<8)) + buf[24];
    int16_t yaw = (int16_t)((buf[43]<<8)) + buf[42];
      
    //std::cout<<"x differ : "<< (c_x - i_x) <<std::endl;
    //std::cout<<"y differ : "<< (c_y - i_y) <<std::endl;
    //std::cout<<"current pitch angle : " <<pitch * 0.02197 << "  new pitch angle : " << newPitch <<std::endl;
    //std::cout<<"current yaw angle: " <<yaw * 0.02197 << "  new yaw angle : " << newYaw <<std::endl;

    std::cout<<"current pitch angle : " <<pitch * 0.02197  <<std::endl;
    std::cout<<"current yaw angle: " <<yaw * 0.02197 <<std::endl;

    double current_pitch = pitch * 0.02197;
    double current_yaw = yaw * 0.02197;
        
    std::cout<<"cal pitch angle : " <<current_pitch  <<std::endl;
    std::cout<<"cal yaw angle: " <<current_yaw <<std::endl;

    double output ; 

    double error    = inputPitch - current_pitch;
    double errorYaw = inputYaw - current_yaw;

    std::cout<<"error : " << error<<std::endl;

    if(fabs(error) < 0.05 && pitch_flag == 0)
    {
      //mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
      mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , 1500 , 0,0,0,0,0);
      port->write_message(msg);   
      pitch_flag = 1;
      //break;
    }

    if(fabs(errorYaw) < 0.05 && yaw_flag == 0)
    {
      //mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
      mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , 1500 , 0,0,0,0,0);
       port->write_message(msg);   
      yaw_flag = 1;
      //break;
    }


    if(pitch_flag == 1 && yaw_flag == 1)
      break;


   
    double derivative_error = (error - error_last) ;
        error_last = derivative_error;

    double derivative_errorYaw = (errorYaw - errorY_last) ;
        errorY_last = derivative_errorYaw;

    // control pitch
    if(pitch_flag != 1)
    {
      if(error > 0) // down
        {
  
          output = 1500 + 100 + (KP_y*(error) + KD_y*(derivative_error));
          std::cout << "first out : " << output << std::endl;
          if(output >= 2000)
          {
            output = 2000;
          }            
          else if(output <= 1600)
          {
            output = 1600;
          }
        }
        else // up
        {
          output = 1500 - 100 + (KP_y*(error) + KD_y*derivative_error);  

          std::cout << "first out : " << output << std::endl;
          if(output >= 1400)
          {
            output = 1400;
          }            
          else if(output <= 1000)
          {
            output = 1000;
          }
        }

        cout<<"output : " << output<<endl;
        mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param1 , (int)output , 0,0,0,0,0);
        port->write_message(msg);   
    }

    // control yaw
    if(yaw_flag != 1)
    {
      if(errorYaw > 0) // down
        {
  
          output = 1500 + 100 + (KP*(errorYaw) + KD*(derivative_errorYaw));
          std::cout << "first out : " << output << std::endl;
          if(output >= 2000)
          {
            output = 2000;
          }            
          else if(output <= 1600)
          {
            output = 1600;
          }
        }
        else // up
        {
          output = 1500 - 100 + (KP*(errorYaw) + KD*derivative_errorYaw);  

          std::cout << "first out : " << output << std::endl;
          if(output >= 1400)
          {
            output = 1400;
          }            
          else if(output <= 1000)
          {
            output = 1000;
          }
        }

        cout<<"output : " << output<<endl;
        mavlink_msg_command_long_pack(mavlink_system.sysid , mavlink_system.compid, &msg , target_system_id , target_comonetId , command , confirmation , param3 , (int)output , 0,0,0,0,0);
        port->write_message(msg);   
    }
  }
  port->stop();
  close(fd);     
}

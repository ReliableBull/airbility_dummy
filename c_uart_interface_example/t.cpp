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
#include <iostream>

using namespace cv;
using namespace cv::ml;
using namespace std; 

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

double CLOCK()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,  &t);
    return (t.tv_sec * 1000)+(t.tv_nsec*1e-6);
}

int main() {

  //VideoCapture cap("tr.mov");
  VideoCapture cap("/dev/video0");
  SelectedRoi.displayRoi = 0;
  SelectedRoi.trackerReady = 0;
  SelectedRoi.initTracker = 0;
  double fps = cap.get(CAP_PROP_FPS);
  tracker = TrackerCSRT::create();
  // write output to file 
  VideoWriter outputVideo;
  //outputVideo.open("video.vmw", VideoWriter::fourcc('W', 'M', 'V', '2'),
  //  cap.get(CAP_PROP_FPS), Size(640, 480), true);
  int num_frames = 1;
  clock_t start;
  clock_t end;

  for (;;)
  {
    if (!cap.isOpened()) {
      cout << "Video Capture Fail" << endl;
      break;
    }

    else {
      Mat img;

      //double start=CLOCK();

      start = clock();
      cap >> img;


      resize(img, img, Size(640, 480));
      namedWindow("Video", WINDOW_AUTOSIZE);
      setMouseCallback("Video", CallBackF, 0);

      if (SelectedRoi.displayRoi != 0) {
        rectangle(img, Rect(SelectedRoi.initX, SelectedRoi.initY, SelectedRoi.actualX 
        - SelectedRoi.initX, SelectedRoi.actualY - SelectedRoi.initY), Scalar(255, 255, 255), 4, 8, 0);
      }
    

    
        //process(frame);
      
      if (SelectedRoi.initTracker == 1) {
        tracker->init(img, Rect2d(SelectedRoi.initX, SelectedRoi.initY,
         SelectedRoi.finalWidth, SelectedRoi.finalHeight));
        SelectedRoi.trackerReady = 1;
      }
      if (SelectedRoi.trackerReady == 1) {
        Rect2d track;
        tracker->update(img, track);
        cout<<track.y<<endl;
        //rectangle(img, track, Scalar(0, 0, 255), 4, 8, 0);
        int tx = (track.x + track.width/2);
        int ty = (track.y + track.height/2);
        Point pt0 = Point(tx , ty);
        circle(img , pt0 , 1 , Scalar(0,0,255) , 0,0,0);

        //Mat roi = img(track);
        /*if (roi.cols > 0) {
        resize(roi, roi, Size(320, 460));
        roi.copyTo(img(Rect(1, 1, 320, 460)));
        }*/
      }
      //outputVideo << img;
      imshow("Video", img);
      //double dur = CLOCK()-start;
      end = clock();

      double seconds = (double(end) - double(start)) / double(CLOCKS_PER_SEC);
      std::cout << "Time taken : " << seconds << " seconds" << std::endl;


      double fpsLive = double(num_frames) / double(seconds);

      std::cout << "Estimated frames per second : " << fpsLive <<std::endl;
      int key2 = waitKey(20);
    }
  }
  return 0;
}
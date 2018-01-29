// tracking light-marker 
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <sstream>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace cv;

// filtering option (Red light examble)
int low_r=200, low_g=0, low_b=0;
int high_r=256, high_g=256, high_b=256;
const Scalar GREEN = Scalar(0,255,0);
const Scalar BLUE = Scalar(255,0,0);
const Scalar RED = Scalar(0,0,255);

static Rect2d rect; // main rectangle to be tracked
static Mat matOriginal; 
static int NoFrame=0;

// for dilation process
static    int erosion_size=6;
static    int morghType=MORPH_ELLIPSE;
static Mat element = getStructuringElement( morghType,
                                      Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

/////////////



static void fixRect2dBetween(Rect2d & r , int x, int y, int maxw,int maxh)
 {
     if(r.x<x) r.x=x;
     if(r.x>(x+maxw-r.width)) r.x=x+maxw-r.width;

     if(r.y<y) r.y=y;
     if(r.y>(y+maxh-r.height)) r.y=y+maxh-r.height;
}

static void initRect( Rect2d _rect)
{
    int max=0;
    if(_rect.width>=_rect.height)
        max=_rect.width;
    else
        max=_rect.height;
   
    if(max%2==0)
        max++;
    _rect.width=_rect.height=max;
    rect=_rect;
}


static bool searchAroundRec(int dx,int dy)
{
    Rect2d _rect(rect.x+dx, rect.y+dy, rect.width, rect.height);
cout << "Trying in _rectx= "<<_rect.x<<",_recty= "<<_rect.y << endl;
	fixRect2dBetween(_rect,0,0,matOriginal.cols,matOriginal.rows) ; 
    Mat src(matOriginal,_rect);
    Mat matProcessed,_image;
      dilate( src, matProcessed, element);

    inRange(matProcessed,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),_image);

    if (countNonZero(_image) < 1)
    {
        return false;
    }else {
        Moments mu; mu=moments(_image, false);
        double cx=    (mu.m10/mu.m00);
        double cy=    (mu.m01/mu.m00);
        _rect.x+=(int)(cx-(_rect.width/2));// should check if out of array
        _rect.y+=(int)(cy-(_rect.height/2));
        rect = _rect; 
	fixRect2dBetween(rect,0,0,matOriginal.cols,matOriginal.rows) ; 
        return true;
    }
}


static bool updateRect( Mat& _image)
{
    if( _image.empty())
        return false;
    //_image.at<uchar>(0 , 0) = 1;
    //_image.at<uchar>(_image.cols-1 , 0) = 1;
    //_image.at<uchar>(0 , _image.rows-1) = 1;

   
if (countNonZero(_image) < 1)
{
    cout << "Cannot see it in this frameNo="<< NoFrame << endl;
    int dx=(rect.width/2);
    int dy=(rect.height/2);
    if(searchAroundRec(dx,dy)||searchAroundRec(-dx,-dy)||searchAroundRec(-dx,dy)||searchAroundRec(dx,-dy))
    {
cout << "Find One" << endl;
        return true;   
    }
    else{
cout << "Cannot Find any !!!" << endl;   
        return false;
    }
}
   


    Moments mu; mu=moments(_image, false);
    double cx=    (mu.m10/mu.m00);
    double cy=    (mu.m01/mu.m00);
   
   
   
    rect.x+=(int)(cx-(rect.width/2));// should check if out of array
    rect.y+=(int)(cy-(rect.height/2));
	fixRect2dBetween(rect,0,0,matOriginal.cols,matOriginal.rows) ; 
cout <<"NoFrame= "<< NoFrame << " !! CentreRec:Cx= "<<rect.x+cx<<",Cy= "<<rect.y+cy << endl;
        return true;

}


int main(int argc, char** argv) {


	Mat fr;

  std::string videoName = argv[1];
  VideoCapture capture(videoName);

    if(!capture.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
        
    }

const string NAME = "Output.avi";  
    int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));     

char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};

    Size S = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH),    
                  (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    VideoWriter outputVideo;                                        
outputVideo.open(NAME,ex, capture.get(CV_CAP_PROP_FPS), S, true);

    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: "  << endl;
        return -1;
    }

// go to frame N
 // capture.set(CV_CAP_PROP_POS_FRAMES, 2600);

    cv::VideoCapture capWebcam=capture;


    cv::Mat matProcessed;


    cv::namedWindow("Processed",WINDOW_NORMAL);
    cv::resizeWindow("Processed", 600,800); 
  
   cv::namedWindow("Original",WINDOW_NORMAL);cv::resizeWindow("Original", 600,800);

 capWebcam>>matOriginal;


      cout  << "Please select ROI-Rect from original window to track it: "  << endl;

    initRect(selectROI("Original",matOriginal, false));
    char charCheckForEscKey = 0;

cout  << "Begin tracking loop... "  << endl;
    while (charCheckForEscKey != 27) {
    capWebcam>>matOriginal;


    Mat src(matOriginal,rect);

  dilate( src, matProcessed, element);
    //red only//cv::inRange(matProcessed,cv::Scalar(0, 0, 200),cv::Scalar(255, 255, 255),matProcessed);
inRange(matProcessed,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),matProcessed);
bool toto=updateRect(matProcessed);// go to next position
   

if(toto)// if we got it then green rectangle will be if not Red one should be
rectangle( matOriginal, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), GREEN, 2);
else
rectangle( matOriginal, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), RED, 2);
 
        cv::imshow("Original", matOriginal);
    outputVideo << matOriginal;
        cv::imshow("Processed", matProcessed);
   
    NoFrame++;
        charCheckForEscKey = cv::waitKey(10);
    }

    return(0);


}

#if 0
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;
Mat dst,src;
int hue1=76,hue2=161;
int sat1=30,sat2=255;
int val1=80,val2=220;
#if 0
int hue1=76,hue2=161;
int sat1=42,sat2=255;
int val1=52,val2=220;
#endif

int main(){
    VideoCapture cap("write.mp4");
    src=imread("./region3.png");
//    cvtColor(src,dst,CV_RGB2HSV);
//    inRange(src, Scalar(hue1, 135, 135), Scalar(hue2, 255, 255), dst);
//    cvtColor(src,dst,CV_HSV2RGB);
    namedWindow("Image");
    createTrackbar( "TopH", "Image", &hue1, 255);
    createTrackbar( "BotH", "Image", &hue2, 255);
    createTrackbar( "TopS", "Image", &sat1, 255);
    createTrackbar( "BotS", "Image", &sat2, 255);
    createTrackbar( "TopV", "Image", &val1, 255);
    createTrackbar( "BotV", "Image", &val2, 255);
    while (true)
         {
              Mat dst,out,put;
              namedWindow("Image");

//              cap>>src;
//              src=src.colRange(180,540);
              cvtColor(src,dst,CV_RGB2HSV);
              inRange(dst, Scalar(hue1, sat1, val1), Scalar(hue2, sat2, val2),out);
              src.copyTo(put,out);
//              morphologyEx(put,put,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
              Mat tepl(src.size(),src.type());
//              tepl=Scalar(255,255,255);
              tepl=Scalar(217,184,180);
              bitwise_not(out,out);
              morphologyEx(out,out,MORPH_OPEN,getStructuringElement(MORPH_ELLIPSE,Size(5,5)),Point(1,1),3);
              morphologyEx(put,put,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
              dst.copyTo(tepl,out);

              imshow( "Image", tepl );

              int iKey = waitKey(100);

              //if user press 'ESC' key
              if (iKey == 27)
              {
                   break;
              }
         }
//    imshow("Image",dst);
    waitKey();
}
#endif

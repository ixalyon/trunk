#if 0
#include "datamining.h"
#include "warputility.h"
#include "utility.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "/cvBlob/cvblob.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <chrono>
using namespace cv;
using namespace std;

Mat src,src_out; Mat src_gray;
int thresh = 70;
int max_thresh = 255;
int size=2,k=0;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( )
{
    VideoCapture cap("1.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    time_t start,end;
    for(k = 0;;k++)
    {
        auto t1 = chrono::high_resolution_clock::now();
        cap >> src;
//                src=src.colRange(120,600);
        if(src.empty())
            continue;
//        src=src.colRange(240,480);
//        src=src.rowRange(100,300);
//              std::ostringstream name;
//              name << "./2/data" << i << ".jpg";
//              imwrite(name.str(),src);
//              waitKey();

        /// Load source image and convert it to gray
        /// Convert image to gray and blur it
        cvtColor( src, src_gray, CV_BGR2GRAY );
//          blur( src_gray, src_gray, Size(3,3) );


          /// Create Window
      if(src.size().height==0||src.size().width==0)
          continue;
          imshow( "Source", src );

          createTrackbar( " Canny thresh:", "Source", &thresh, 500, thresh_callback );
          thresh_callback( 0, 0 );
          auto t2 = std::chrono::high_resolution_clock::now();
          cout<<" Time taken -"<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count()<<endl;
          waitKey();
    }

//  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;


  Canny( src_gray, canny_output, thresh, thresh*2, 3 );

  findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0) );
    imshow("tests",canny_output);

  OCV::remove_contours(contours,1500,12000);
    if(contours.size()==0)
        return;
  RotatedRect rect;

  Mat M, rotated, roi;
  Mat mask=Mat::zeros( canny_output.size(), CV_8UC1);
    vector<Point> approx;
  for( auto i = 0; i<contours.size(); i++ )
     {

 cout<<"Contour : "<<contours.size()<<" Area : "<<contourArea(contours[i])<<" Corners : "<<i<<endl;

//      cout/*<<rect_size.height<<" - "<<rect_size.width<<" - "*/<<i;
      rect = minAreaRect( Mat(contours[i]));
      float angle = rect.angle;
      Size rect_size = rect.size;

      if (rect.angle < -45.) {
          angle += 90.0;
          swap(rect_size.width, rect_size.height);
      }

      M = getRotationMatrix2D(rect.center, angle, 1.0);

      warpAffine(canny_output, canny_output, M, src.size(), INTER_CUBIC);
      warpAffine(src, rotated, M, src.size(), INTER_CUBIC);


      vector<Vec2f> lines;
      HoughLines(canny_output, lines, 1, CV_PI/4,100, 0, 0 );

          for( auto i = 0; i < lines.size()-1; i++ )
              if(lines[i][1] >CV_PI/3/*||theta>CV_PI*2/3*/)

//      cout<<endl;

      for( size_t i = 0; i < lines.size(); i++ )
      {
         float rho = lines[i][0], theta = lines[i][1];
//         if(theta ==0/*||theta==CV_PI*/)
//             if(lines[i][0]<300&&lines[i][0]>200&&i!=lines.size()-1){
//                 cout<<" Distances : "<<lines[i][0]<<" - "<<theta<<endl;
//             }
//            else
//                 continue;

         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
//         line( rotated, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
      }
    imshow("sss",rotated);

      getRectSubPix(rotated, rect_size, rect.center, roi);


//      cout<<roi.size().height/10.0<<" -------------- "<<roi.size().width/28.0<<endl;
      resize(roi,roi,Size(84,30));
      Mat roix=roi.clone();
      cvtColor(roi,roi,CV_BGR2GRAY);
//      adaptiveThreshold(roi,roi,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,3,0);
      threshold(roi,roi,0,255,CV_THRESH_BINARY|CV_THRESH_OTSU);
        cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx( roi, roi, cv::MORPH_OPEN, structuringElement ,Point(1,1));
        cv::morphologyEx( roi, roi, cv::MORPH_CLOSE, structuringElement ,Point(1,1));
        cv::morphologyEx( roi, roi, cv::MORPH_OPEN, structuringElement ,Point(1,1));
        blur(roi,roi,Size(3,3));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//        cv::drawContours(roi, contours, -1, cv::Scalar(128), CV_FILLED);
        int j=0;
      for(auto contour:contours)
          if(contourArea(contour)>100){
                std::ostringstream name;
                name << "./4/data" <<k<<"-"<<i << "-"<<++j<<".png";
                cout<<" ---- "<<contourArea(contour)<<endl;
                Rect boundary = boundingRect(contour);
                Mat roin = roix(boundary);
                imwrite(name.str(),roin);
                imshow("Cropped",roin);
//            waitKey();
          }

//      int w=roi.size().width/28;
//      std::ostringstream name;
//      name << "./1/data" <<k<<"-"<<i << "-1.png";
//      Mat roi1(roi,Rect(0,0,w*9,roi.size().height));
//      imwrite(name.str(),roi1);
//      name.str("");
//      name << "./1/data" <<k<<"-"<<i  << "-2.png";
//      Mat roi2(roi,Rect(w*9,0,w*9,roi.size().height));
//      imwrite(name.str(),roi2);
//      name.str("");
//      name << "./1/data" << k<<"-"<<i  << "-3.png";
//      Mat roi3(roi,Rect(w*18,0,w*9,roi.size().height));
//      imwrite(name.str(),roi3);
//      waitKey();

     }
//waitKey();


  /// Show in a window
//  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
contours.clear();
src_out.release();


}


#endif


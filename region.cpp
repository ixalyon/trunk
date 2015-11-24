#if 1
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
void get_rect_points(cv::Rect const &rect, Point points[]);
void regionSelect();
void sort_rect_corners(RotatedRect &rct, Point2f rectPoints[]);
Point2f corners_center(std::vector<Point2f> const &corners);
bool containsCheck(Rect &rc, int begin, int end);

/** @function main */
int main( )
{
    VideoCapture cap("./write3.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
//    time_t start,end;
    for(k = 0;;k++)
    {
        auto t1 = chrono::high_resolution_clock::now();
        cap >> src;
//                src=src.colRange(120,600);
        if(src.empty())
            continue;
        src=src.colRange(180,540);
//        src=src.rowRange(100,300);

        cvtColor( src, src_gray, CV_BGR2GRAY );
//          blur( src_gray, src_gray, Size(3,3) );


          /// Create Window
      if(src.size().height==0||src.size().width==0)
          continue;
            regionSelect();
             auto t2 = std::chrono::high_resolution_clock::now();
//          cout<<" Time taken -"<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count()<<endl;
//          waitKey();
    }

//  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void regionSelect()
{


      Mat canny_output;
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      bool flag=true;


      Canny( src_gray, canny_output, thresh, thresh*2, 3 );

      findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0) );
    //    imshow("tests",canny_output);

      OCV::remove_contours(contours,1500,12000);
        if(contours.size()==0)
            return;
      RotatedRect rect;

      Mat M, rotated, roi,correctedImage;
      Mat mask=Mat::zeros( canny_output.size(), CV_8UC1);
      vector<Point> approx;
      vector<RotatedRect> selectedPattern;
      vector<Point2f> scale;
      for( auto i = 0; i<contours.size(); i++ )
      {
    //      cout/*<<rect_size.height<<" - "<<rect_size.width<<" - "*/<<i;
          rect = minAreaRect( Mat(contours[i]));
          Point2f vertices[4];
          rect.points(vertices);
          float angle = rect.angle;
          Size rect_size = rect.size;

          if (rect.angle < -45.) {
              angle += 90.0;
              swap(rect_size.width, rect_size.height);
          }

          M = getRotationMatrix2D(rect.center, angle, 1.0);

          warpAffine(canny_output, canny_output, M, src.size(), INTER_CUBIC);
          warpAffine(src, rotated, M, src.size(), INTER_CUBIC);

//          imshow("Srcs",src);
          auto rec=rect.center;
          int heightRect=rect_size.height;
          int widthRect=rect_size.width;
          Rect rc((int)rec.x-widthRect/2,(int)rec.y-heightRect/2,widthRect,heightRect);

          cout<<"Contour : "<<rect.center<<" Area : "<<contourArea(contours[i])<<" Corners : "<<rect.size<<endl;
          getRectSubPix(rotated, rect_size, rect.center, roi);
          imshow("sss",roi);
//          waitKey();
          if(containsCheck(rc,10,270)){
    ////          cout<<"hit"<<endl;
    //          Point points[4];
    //          get_rect_points(rc,points);
    //          line(rotated, {points[0].x, points[0].y}, {points[1].x, points[1].y}, Scalar(0,0,255), 2);
    //          line(rotated, {points[1].x, points[1].y}, {points[2].x, points[2].y}, Scalar(0,0,255), 2);
    //          line(rotated, {points[2].x, points[2].y}, {points[3].x, points[3].y}, Scalar(0,0,255), 2);
    //          line(rotated, {points[3].x, points[3].y}, {points[0].x, points[0].y}, Scalar(0,0,255), 2);
              for (int i = 0; i < 4; i++)
                  line(rotated, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//              imshow( "Source", rotated );
//              waitKey();
              if(flag){
                  flag=false;
                  correctedImage=rotated.clone();
              }
              scale.push_back(Point2f(widthRect/27.0,heightRect/10.0));
              selectedPattern.push_back(rect);
//              cout<<" x - "<<rc.x<<" y- "<<rc.y<<endl;
          }

    }
      if(selectedPattern.size()<2)
          return;
{
      Point2f rctP[2][4];
      int i = 0 ;
      for (RotatedRect rct:selectedPattern) {
        if(i<2){
            vector<Point2f> pointVector;
            sort_rect_corners(rct,rctP[i]);
            rct.points(rctP[i]);
            for(auto rectPoint:rctP[i])
                pointVector.push_back(rectPoint);

            int j=0;
            std::ostringstream name;
            for(Point2f point:pointVector){
                name<<++j;
                if(rct.angle<-45.)
                    name<<"-";
//                putText( src,name.str(), Point(int(point.x+5.0),int(point.y)),  FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
//                circle(src,Point(int(point.x),int(point.y)),3,Scalar(255,0,0),2);
                name.str("");
            }
            cout<<" Pattern Number : "<<i<<" Center : "<<rct.center<<endl;

        }
        i++;

      }
      line(src,rctP[0][0],rctP[1][1],Scalar(255,255,0));
      line(src,rctP[0][0],rctP[0][3],Scalar(255,255,0));
      line(src,rctP[1][2],rctP[1][1],Scalar(255,255,0));
      line(src,rctP[1][2],rctP[0][3],Scalar(255,255,0));
      vector<Point2f> oPoints
      {
          rctP[0][0],rctP[0][3],rctP[1][2],rctP[1][1]
      };
      Mat target(108,96,src.type());
      std::vector<cv::Point2f> target_points
      {
       {0, 0}, {target.cols - 1, 0},
       {target.cols - 1, target.rows - 1},
       {0, target.rows - 1}
      };
      Mat const trans_mat = cv::getPerspectiveTransform(oPoints, target_points);
      warpPerspective(src, target, trans_mat, target.size());

//      imwrite("region.png",src);
      imshow("Region",src);
      waitKey();
}
      contours.clear();
      src_out.release();
}

void get_rect_points(cv::Rect const &rect, Point points[])
{
    points[0].x = rect.x;
    points[0].y = rect.y;
    points[1].x = rect.x + rect.width - 1;
    points[1].y = rect.y;
    points[2].x = points[1].x;
    points[2].y = rect.y + rect.height - 1;
    points[3].x = rect.x;
    points[3].y = points[2].y;
}
void sort_rect_corners(RotatedRect &rct,Point2f rectPoints[])
{
#if 1
    float angle = rct.angle;
    Size rect_size = rct.size;
    Point2f center = rct.center;

    if (rct.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    rct = RotatedRect(center,rect_size,angle);
//    rct.points(rectPoints);
#else
    std::vector<Point2f> left, right;
    Point2f const center = rct.center;
    for (auto i = 0; i < 4; i++){
        if (rectPoints[i].x < center.x)
            left.emplace_back(rectPoints[i]);
        else
            right.emplace_back(rectPoints[i]);
    }

    if(rct.angle<-45.){
        rectPoints[0] = left[0].y  > left[1].y  ? left[1]  : left[0];
        rectPoints[1] = left[0].y  > left[1].y  ? left[0]  : left[1];
        rectPoints[2] = right[0].y > right[1].y ? right[1] : right[0];
        rectPoints[3] = right[0].y > right[1].y ? right[0] : right[1];
    }else{
        rectPoints[1] = left[0].y  > left[1].y  ? left[1]  : left[0];
        rectPoints[2] = left[0].y  > left[1].y  ? left[0]  : left[1];
        rectPoints[3] = right[0].y > right[1].y ? right[1] : right[0];
        rectPoints[0] = right[0].y > right[1].y ? right[0] : right[1];
    }
#endif

}

bool containsCheck(Rect &rc, int begin, int end){
    bool contains=false;
    for (int i = begin; i <= end; i+=30){
        contains=contains||rc.contains(Point(180,i));
        cout<<contains;
    }
//    waitKey();
    return contains;
}




#endif


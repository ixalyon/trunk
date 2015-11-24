#if 0
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

using namespace cv;
using namespace std;

Mat src,src_out; Mat src_gray;
int thresh = 70;
int max_thresh = 255;
int size=2;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( )
{
    VideoCapture cap("./2.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    waitKey();
//    VideoCapture cap(0); // open the default camera
//    if(!cap.isOpened())  // check if we succeeded
//        return -1;
    char* source_window = "Source";
    namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    for(;;)
    {
          /// Load source image and convert it to gray
//          src = imread(  "sample8.png");
        cap >> src;
        src=src.colRange(120,600);

//        imwrite("sample9.png",src);
          /// Convert image to gray and blur it
          cvtColor( src, src_gray, CV_BGR2GRAY );
          blur( src_gray, src_gray, Size(3,3) );
//          Laplacian(src_gray,src_gray,src_gray.depth());
//        bilateralFilter(src_gray,src_gray,5,10.0,10.0);
          /// Create Window
          imshow( source_window, src_gray );
waitKey();
          createTrackbar( " Canny thresh:", "Source", &thresh, 500, thresh_callback );
          thresh_callback( 0, 0 );
          waitKey();
    }
  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
//  adaptiveThreshold(src_gray,src_gray,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,3,5);
//  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//  cv::morphologyEx( src_gray, src_gray, cv::MORPH_CLOSE, structuringElement ,Point(1,1));
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
//  bitwise_not(canny_output,canny_output);
//  imshow("Canny",src_gray);
//  waitKey();
//  Scharr( src_gray, canny_output,-1,0,1,1);

//  imshow("tet",canny_output);
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0) );

#if 0
  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  {
      approxPolyDP( Mat(contours[i]), contours_poly[i], 10, false );
      boundRect[i] = boundingRect( Mat(contours_poly[i]) );
  }

  /// Draw polygonal contour + bonding rects
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0,j = 0; i< contours.size(); i++ )
  {
      if(contourArea(contours[i])<500)
          continue;
      Scalar color = Scalar(255,0,255);
//      drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
//      rectangle( src, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
      Mat roi(src,boundRect[i]);
      imshow("polygon",roi);
      std::ostringstream name;
      name << "./1/data" << ++j << "-1.png";
      Mat roi1(roi,Rect(6,0,20,roi.size().height));
      imwrite(name.str(),roi1);
      name.str("");
      name << "./1/data" << j << "-2.png";
      Mat roi2(roi,Rect(28,0,20,roi.size().height));
      imwrite(name.str(),roi2);
      name.str("");
      name << "./1/data" << j << "-3.png";
      Mat roi3(roi,Rect(50,0,20,roi.size().height));
      imwrite(name.str(),roi3);
//      waitKey();
  }
#endif
 #if 0
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  Mat mask=Mat::zeros( canny_output.size(), CV_8UC1);
    vector<Point> approx;
  for( int i = 0; i< contours.size(); i++ )
     {
      if(arcLength(contours[i],true)<50||contourArea(contours[i])<1000)
          continue;

      Point2f rect_points[4];
      // rect is the RotatedRect (I got it from a contour...)
      RotatedRect rect;
      // matrices we'll use
      Mat M, rotated, cropped;
      // get angle and size from the bounding box
      rect = minAreaRect( Mat(contours[i]));
      float angle = rect.angle;
      Size rect_size = rect.size;

      if (rect.angle < -45.) {
          angle += 90.0;
          swap(rect_size.width, rect_size.height);
      }
      // get the rotation matrix
      M = getRotationMatrix2D(rect.center, angle, 1.0);
      // perform the affine transformation
      warpAffine(src, rotated, M, src.size(), INTER_CUBIC);
      // crop the resulting image
      getRectSubPix(rotated, rect_size, rect.center, cropped);
      resize(cropped,cropped,Size(81,30));
      imshow("Cropped",cropped);
      waitKey();


//      minAreaRect( Mat(contours[i])).points(rect_points);

      cv::Point vertices[4];
      for(int i = 0; i < 4; ++i){
        vertices[i] = rect_points[i];
      }
      approxPolyDP(Mat(contours[i]), approx,arcLength(Mat(contours[i]), true)*0.02, true);
      int vtc = approx.size();
      cv::fillConvexPoly(mask,vertices,4,255);
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      mask=Mat::zeros( canny_output.size(), CV_8UC1);
      for( int j = 0; j < 4; j++ )
          line( mask, rect_points[j], rect_points[(j+1)%4], color, 2, 8 );
//       drawContours( mask, contours, i, color, 2, 8, hierarchy, 0, Point() );
       drawContours(mask, contours, i, Scalar(255), CV_FILLED);
#if 0
       vector<Vec4i> lines;
       HoughLinesP(mask, lines, 1, CV_PI/180, 150, 50, 10 );
       for( size_t i = 0; i < lines.size(); i++ )
       {
         Vec4i l = lines[i];
         line( mask, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 1, CV_AA);

  }
#endif
       Mat src_out_temp;
       std::ostringstream name;
//       name << "image" << i << ".png";
//       imwrite(name.str(),src_out_temp);
       src.copyTo(src_out_temp,mask);
       src.copyTo(src_out,mask);
        imshow("source", src_out);

        cout<<"Contour : "<<i<<" Area : "<<contourArea(contours[i])<<" Corners : "<<vtc<<endl;
//        waitKey(0);
//       structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
//       morphologyEx( mask, mask, cv::MORPH_OPEN, structuringElement );
//       structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
//       morphologyEx( mask, mask, cv::MORPH_OPEN, structuringElement,Point(1,1),1 );
     }
//  drawing contours
#endif
  Mat BW;
//    circle(mask, Point(5,5), 3, CV_RGB(255,255,255), -1);
//    threshold(mask,mask,0.5,1.0,CV_THRESH_BINARY);

//  watershed(src, mask);

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//  imshow( "Contours", src_out );
//  imshow("source", mask);
contours.clear();
src_out.release();


}
#endif



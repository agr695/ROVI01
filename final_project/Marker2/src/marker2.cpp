#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
using namespace cv;
using namespace std;

/******************************************************************************
 ****************************constant values***********************************
 ******************************************************************************/
//path to the sequence
const string normal_sequence_path = "/home/student/Downloads/marker_thinline/marker_thinline_";
const string hard_sequence_path = "/home/student/Downloads/marker_thinline_hard/marker_thinline_hard_";
//canny detector values
#define Canny_blur_kernel_size 3
#define Canny_detector_kernel_size 3
#define Canny_lowThreshold 150
#define Canny_upperThreashold 200
//houhglines detector value
#define Hough_threashold 100
#define Hough_minLinLength 200
#define Hough_maxLineGap 30
#define theta_margin_parallel 1*CV_PI/180
#define theta_margin_perpendicular 5*CV_PI/180
//minimum value of denominator to be consider not parallel line
#define minimum_denominator_value 0.001 // 1e-3
//minimum distance to be considerated the same point in pixels
#define minimum_distance_between_points 30
//minimum number of points around a position to be part of the marker
#define minimum_number_of_points 10

/******************************************************************************
 *******************************functions used ********************************
 ******************************************************************************/
void get_marker2(Mat img_original, int image_number);

cv::Mat CannyThreshold(cv::Mat img, int blur_kernel_size, int canny_kernel_size,
                                          int lowThreshold, int upperThreashold);

vector<Point2i> get_intersections(vector<Vec4i> lines);

Point2i get_intersection_point(Vec4i line1, Vec4i line2);

vector<Point2i> get_square_points(vector<Point2i> intersections);

int main(int argc, char *argv[]) {
  std::string sequence;
  if(argc>1){
    sequence=argv[1];
  }
  else{
    sequence="normal";
  }
  std::cout << sequence << '\n';

  unsigned int image_number = 1;
  cv::Mat img_original;

  std::string filepath;

  if(sequence=="hard"){
    while(true) {
      // usleep(1000*1000);
      // Load new image
      if(image_number<10){
        filepath = hard_sequence_path + "0" + to_string(image_number) + ".png";
      }else{
        filepath = hard_sequence_path + to_string(image_number) + ".png";
      }

      img_original = cv::imread(filepath, CV_LOAD_IMAGE_GRAYSCALE);

      if(img_original.empty()){
          break;
      }

      get_marker2(img_original,image_number);

      image_number++;
    }
  }else if(sequence=="normal"){
    while(true) {
      // Load new image
      if(image_number<10){
        filepath = normal_sequence_path + "0" + to_string(image_number) + ".png";
      }else{
        filepath = normal_sequence_path + to_string(image_number) + ".png";
      }

      img_original = cv::imread(filepath, CV_LOAD_IMAGE_GRAYSCALE);

      if(img_original.empty()){
          break;
      }

      get_marker2(img_original,image_number);

      image_number++;
    }
  }
  while (cv::waitKey() != 27 && image_number>1); // (do nothing)

  return 0;
}

void get_marker2(Mat img_original, int image_number){
  cv::Mat img_canny;
  cv::Mat img_hough;
  vector<Vec4i> lines;
  vector<Point2i> square_points;
  vector<Point2i> intersections;

  // edge detection using Canny detector
  img_canny = CannyThreshold(img_original, Canny_blur_kernel_size,
                        Canny_detector_kernel_size, Canny_lowThreshold,
                        Canny_upperThreashold);

  dilate( img_canny, img_canny, Mat(), Point(-1,-1), 4);
  erode( img_canny, img_canny, Mat(), Point(-1,-1), 4);
  cvtColor(img_canny, img_hough, CV_GRAY2BGR);
  cvtColor(img_original, img_original, CV_GRAY2BGR);

  HoughLinesP(img_canny, lines, 1, CV_PI/180, Hough_threashold, Hough_minLinLength, Hough_maxLineGap);
  intersections=get_intersections(lines);
  // vector<Point2i> square_points=intersections;
  if(intersections.size()>0){
    square_points=get_square_points(intersections);
  }

  /****************************************************************************
   ************************ Presentation of results****************************
   ****************************************************************************/
  // cvtColor(img_canny, img_canny, CV_GRAY2BGR);
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( img_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
  }

  for( size_t i = 0; i < square_points.size(); i++ )
  {
    Point2f square_point = square_points[i];
    circle(img_original, square_point, 10, Scalar(0,0,255));
  }
  imshow("image"+to_string(image_number)+" detection", img_original);
}

// https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
cv::Mat CannyThreshold(cv::Mat img, int blur_kernel_size, int canny_kernel_size,
                                          int lowThreshold, int upperThreashold){
  cv::Mat img_edged=img.clone();
  // Reduce noise with a kernel 3x3
  blur( img, img_edged, Size(blur_kernel_size,blur_kernel_size) );

  // Canny detector
  Canny( img_edged, img_edged, lowThreshold, upperThreashold, canny_kernel_size );

  return img_edged;
 }

vector<Point2i> get_intersections(vector<Vec4i> lines){
  vector<Point2i> intersections;
  for (size_t i = 0; i < lines.size()-1; i++){
    for (size_t j = i+1; j < lines.size(); j++) {
      Point2f point=get_intersection_point(lines[i], lines[j]);
      if(point.x!=-1 && point.y!=-1){
        intersections.push_back(point);
      }
    }
  }
  return intersections;
}

//https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
Point2i get_intersection_point(Vec4i line1, Vec4i line2){
  int P1x=line1[0];
  int P1y=line1[1];
  int P2x=line1[2];
  int P2y=line1[3];

  int P3x=line2[0];
  int P3y=line2[1];
  int P4x=line2[2];
  int P4y=line2[3];

  Point2i ret;

  double denominator = (P1x-P2x)*(P3y-P4y) - (P1y-P2y)*(P3x-P4x);
  if(fabs(denominator)>minimum_denominator_value){
    ret.x=((P1x*P2y-P1y*P2x)*(P3x-P4x)-(P1x-P2x)*(P3x*P4y-P3y*P4x))/denominator;
    ret.y=((P1x*P2y-P1y*P2x)*(P3y-P4y)-(P1y-P2y)*(P3x*P4y-P3y*P4x))/denominator;
    if(!(min(P1x,P2x)<=ret.x && ret.x<=max(P1x,P2x) &&
         min(P3x,P4x)<=ret.x && ret.x<=max(P3x,P4x) &&
         min(P1y,P2y)<=ret.y && ret.y<=max(P1y,P2y) &&
         min(P3y,P4y)<=ret.y && ret.y<=max(P3y,P4y))){
          ret.x=-1;
          ret.y=-1;
    }
  }else{
    ret.x=-1;
    ret.y=-1;
  }
  return ret;
}

vector<Point2i> get_square_points(vector<Point2i> intersections){
  vector<Point2i> ret;
  int count;
  double distance;
  for (size_t i = 0; i < intersections.size()-1; i++) {
    if(intersections[i].x==-1 && intersections[i].y==-1)
      continue;
    for (size_t j = i+1; j < intersections.size(); j++) {
      if(intersections[j].x==-1 && intersections[j].y==-1)
        continue;
      distance = norm(intersections[i]-intersections[j]);
      if(distance<=minimum_distance_between_points){
        intersections[j].x=-1;
        intersections[j].y=-1;
        count++;
      }
    }
    if(count>=minimum_number_of_points){
      ret.push_back(intersections[i]);
      count = 0;
    }
  }
  return ret;
}

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
#define Canny_upperThreashold 250
//houhglines detector value
#define Hough_threashold 125
#define theta_margin_parallel 0*CV_PI/180
#define theta_margin_perpendicular 5*CV_PI/180

cv::Mat CannyThreshold(cv::Mat img, int blur_kernel_size, int canny_kernel_size,
                                          int lowThreshold, int upperThreashold);

vector<Vec2f> find_parallel_perpendicular_lines(vector<Vec2f> lines);

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

  std::string filepath;

  cv::Mat img_original;
  cv::Mat img_canny;
  cv::Mat img_hough;

  vector<Vec2f> lines;
  vector<Vec2f> parallel_perpendicular_lines;

  if(sequence=="hard"){
    while(true) {
      usleep(1000*1000);
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

      // edge detection using Canny detector
      img_canny = CannyThreshold(img_original, Canny_blur_kernel_size,
      Canny_detector_kernel_size, Canny_lowThreshold,
      Canny_upperThreashold);

      // detection of lines using HoughLines
      cvtColor(img_canny, img_hough, CV_GRAY2BGR);
      HoughLines(img_canny, lines,1, CV_PI/180, Hough_threashold, 0, 0 );
      cvtColor(img_canny, img_canny, CV_GRAY2BGR);


      //filter non parallel or perpendicular lines
      parallel_perpendicular_lines=find_parallel_perpendicular_lines(lines);

      for( size_t i = 0; i < parallel_perpendicular_lines.size(); i++ )
      {
         float rho = parallel_perpendicular_lines[i][0];
         float theta = parallel_perpendicular_lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( img_hough, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
      }

      for( size_t i = 0; i < lines.size(); i++ )
      {
         float rho = lines[i][0];
         float theta = lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( img_canny, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
      }

      //image show
      // imshow("image"+to_string(image_number)+" edged", img_canny);
      imshow("image"+to_string(image_number)+" lines", img_hough);

      //find next image in the sequence
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

      // edge detection using Canny detector
      img_canny = CannyThreshold(img_original, Canny_blur_kernel_size,
                            Canny_detector_kernel_size, Canny_lowThreshold,
                            Canny_upperThreashold);

      // detection of lines using HoughLines
      cvtColor(img_canny, img_hough, CV_GRAY2BGR);
      HoughLines(img_canny, lines,1, CV_PI/180, Hough_threashold, 0, 0 );
      cvtColor(img_canny, img_canny, CV_GRAY2BGR);


      //filter non parallel or perpendicular lines
      parallel_perpendicular_lines=find_parallel_perpendicular_lines(lines);

      for( size_t i = 0; i < parallel_perpendicular_lines.size(); i++ )
      {
         float rho = parallel_perpendicular_lines[i][0];
         float theta = parallel_perpendicular_lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( img_hough, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
      }

      for( size_t i = 0; i < lines.size(); i++ )
      {
         float rho = lines[i][0];
         float theta = lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( img_canny, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
      }

      //image show
      // imshow("image"+to_string(image_number)+" edged", img_canny);
      imshow("image"+to_string(image_number)+" lines", img_hough);

      //find next image in the sequence
      image_number++;
    }
  }
  while (cv::waitKey() != 27 && image_number>1); // (do nothing)

  return 0;
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

vector<Vec2f> find_parallel_perpendicular_lines(vector<Vec2f> lines){
  vector<Vec2f> parallel_lines;
  vector<Vec2f> return_lines;

  /*check parallelism*/
  for (size_t i = 0; i < lines.size()-1; i++){
    if(lines[i][0]==NAN)
      continue;
    for (size_t j = i+1; j < lines.size(); j++) {
      if(lines[i][1]>=lines[j][1]-theta_margin_parallel && lines[i][1]<=lines[j][1]+theta_margin_parallel){
        if(lines[i][0]!=NAN){
          parallel_lines.push_back(lines[i]);
          lines[i][0]=NAN;
        }
        parallel_lines.push_back(lines[j]);
        lines[j][0]=NAN;
      }
    }
  }

  /*check perpendicularity*/
  for (size_t i = 0; i < parallel_lines.size()-1; i++){
    if(parallel_lines[i][0]==NAN)
      continue;
    for (size_t j = i+1; j < parallel_lines.size(); j++) {
      if(fabs(parallel_lines[i][1]-parallel_lines[j][1])>=CV_PI/2-theta_margin_perpendicular
          && fabs(parallel_lines[i][1]-parallel_lines[j][1])<=CV_PI/2+theta_margin_perpendicular){
        if(parallel_lines[i][0]!=NAN){
          return_lines.push_back(parallel_lines[i]);
          parallel_lines[i][0]=NAN;
        }
        return_lines.push_back(parallel_lines[j]);
        parallel_lines[j][0]=NAN;
      }
    }
  }

  return return_lines;
}

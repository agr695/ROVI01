//
// Created by alex on 12/9/18.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;

/** @function main */
int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
            // name  | default value    | help message
                                 "{help   |                  | print this message}"
                                 "{@image | ../marker_color_01.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    Mat src, src_gray;
    int images =  30;
    std::string filepath;

    for (int sel = 1; sel <= images; sel++) {
        /// Read the image
        if (std::to_string(sel).length() == 1) {
            filepath = "../easy/marker_color_0" + std::to_string(sel) + ".png";
        }
        else {
            filepath = "../easy/marker_color_" + std::to_string(sel) + ".png";
        }
        src = imread(filepath, 1);

        if (!src.data) { return -1; }

        /// Convert it to gray
        cvtColor(src, src_gray, CV_BGR2GRAY);
//    imshow( "Hough Circle Transform gray", src_gray );

        /// Reduce the noise so we avoid false circle detection
        GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);

        std::vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 6, 15, 85, 0, 0);
        /// Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }

        /// Show your results
        namedWindow("Hough Circle " + filepath, CV_WINDOW_AUTOSIZE);
        imshow("Hough Circle " + filepath, src);
        waitKey(0);
    }
    return 0;
}


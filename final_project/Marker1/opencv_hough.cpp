//
// Created by alex on 12/6/18.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include "extract_points.h"

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
    int images =  52;
    std::string filepath;

    for (int sel = 1; sel <= images; sel++) {
        /// Read the image
        if (std::to_string(sel).length() == 1) {
            filepath = "../hard/marker_color_hard_0" + std::to_string(sel) + ".png";
        }
        else {
            filepath = "../hard/marker_color_hard_" + std::to_string(sel) + ".png";
        }
        src = imread(filepath, 1);

        if (!src.data) { return -1; }


        vector<Point> true_centers = get_points(src);


        /// Draw chosen circle centers
        for (auto &center : true_centers) {
            circle(src, center, 10, cvScalar(0, 0, 255, 0), -1);
        }

        /// Draw the circles detected
//        for (size_t i = 0; i < circles.size(); i++) {
//            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//            int radius = cvRound(circles[i][2]);
//             circle center
//            circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//             circle outline
//            circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//        }

        /// Show your results
//        namedWindow("Hough Circle " + filepath, CV_WINDOW_AUTOSIZE);
//        imshow("Hough Circle " + filepath, src);

        /// Save results
        if (std::to_string(sel).length() == 1) {
            imwrite("../Results/marker_color_hard_0" + std::to_string(sel) + ".png", src);
        }
        else {
            imwrite("../Results/marker_color_hard_" + std::to_string(sel) + ".png", src);
        }
//        waitKey(0);
    }
    return 0;
}


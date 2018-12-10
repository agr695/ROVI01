//
// Created by alex on 12/8/18.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

vector<Point> get_points(Mat src) {
    Mat src_gray;
    /// Get binary image
    for (int r = 0; r < src.rows; r++) {
        for (int c = 0; c < src.cols; c++) {
            Vec3b color = src.at<Vec3b>(Point(c,r));
            if ((color.val[2] + color.val[1]) * 0.8 < color.val[0]) {
                color.val[0] = 0;
                color.val[1] = 0;
                color.val[2] = 0;
            }
            else {
                color.val[0] = 255;
                color.val[1] = 255;
                color.val[2] = 255;
            }
            src.at<Vec3b>(Point(c, r)) = color;
        }
    }

    /// Convert it to gray
    cvtColor(src, src_gray, CV_BGR2GRAY);
    imshow( "Hough Circle Transform gray", src_gray );
    imwrite("../Results/Binary.png", src_gray);


    /// Reduce the noise so we avoid false circle detection
    GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);

    std::vector<Vec3f> circles;


    /// Apply the Hough Transform to find the circles
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 6, 255, 25, 0, 0); // 5 and 60

    /// Filter circles
    std::vector<Point> centers;
    for (auto &circle : circles) {
        centers.emplace_back(cvRound(circle[0]), cvRound(circle[1]));
    }

    std::vector<Point> true_centers;
    bool found = false;
    for (auto &center1 : centers) {
        for (auto &center2 : centers) {
            for (auto &center3 : centers) {
                if ((center1 != center2) and (center2 != center3) and (center3 != center1) and !found) {
                    if ((std::max(abs(center1.x - center2.x), abs(center1.y - center2.y)) > 50) and
                        (std::max(abs(center1.x - center2.x), abs(center1.y - center2.y)) < 300)) {
                        if ((std::max(abs(center2.x - center3.x), abs(center2.y - center3.y)) > 50) and
                            (std::max(abs(center2.x - center3.x), abs(center2.y - center3.y)) < 300)) {
                            if ((std::max(abs(center3.x - center1.x), abs(center3.y - center1.y)) > 50) and
                                (std::max(abs(center3.x - center1.x), abs(center3.y - center1.y)) < 300)) {
                                true_centers.push_back(center1);
                                true_centers.push_back(center2);
                                true_centers.push_back(center3);
                                found = true;
                            }
                        }
                    }
                }
            }
        }
    }

    if  (!found) {
        for (auto &center1 : centers) {
            for (auto &center2 : centers) {
                if ((center1 != center2) and (!found)) {
                    if ((std::max(abs(center1.x - center2.x), abs(center1.y - center2.y)) > 50) and
                        (std::max(abs(center1.x - center2.x), abs(center1.y - center2.y)) < 300)) {
                        true_centers.push_back(center1);
                        true_centers.push_back(center2);
                        found = true;
                    }
                }
            }
        }
    }

    return true_centers;
}
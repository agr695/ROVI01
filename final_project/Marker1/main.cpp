#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <utility>


using namespace cv;
using namespace std;

std::pair<double, double> compute_theta(Mat img, int r, int c) {

    int i, j;
    double gxx = 0, gyy = 0, gxy = 0, theta, F_theta;
    int sobelx[3][3] = {
            {-1, 0, 1},
            {-2, 0, 2},
            {-1, 0, 1}
    };
    int sobely[3][3] = {
            {-1, -2, -1},
            {0,  0,  0},
            {1,  2,  1}
    };
/// Find circles of all colors (Better idea might be to just search for blue circles):
    for (i = -1; i <= 1; i++) {
        for (j = -1; j <= 1; j++) {
            gxy += img.at<cv::Vec3b>(r + i, c + j)[0] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[0] * sobely[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobely[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobely[i + 1][j + 1];
            gxx += img.at<cv::Vec3b>(r + i, c + j)[0] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[0] * sobelx[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobelx[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobelx[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobelx[i + 1][j + 1];
            gyy += img.at<cv::Vec3b>(r + i, c + j)[0] * sobely[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[0] * sobely[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobely[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[1] * sobely[i + 1][j + 1] +
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobely[i + 1][j + 1] *
                   img.at<cv::Vec3b>(r + i, c + j)[2] * sobely[i + 1][j + 1];
        }
    }
    theta = 1. / 2 * atan((2 * gxy) / (gxx - gyy));
    F_theta = sqrt(1. / 2 * ((gxx + gyy) + (gxx - gyy) * cos(2 * theta) + 2 * gxy * sin(2 * theta)));
    return std::make_pair(theta, F_theta);
}

Point get_minimum(int a[768][1024], vector<Point> points) {
    int min = 10000;
    Point min_point;
    for (auto &point : points) {
        if (min > a[point.x][point.y]) {
            min = a[point.x][point.y];
            min_point = point;
        }
    }
    return min_point;
}

vector<Point> replace(int r, int c, const Point &p, vector<Point> points) {
    for (auto &point : points) {
        if (point == p) {
            point = Point(r, c);
            break;
        }
    }
    return points;
}

Point get_max(vector<Point> points, int a[768][1024]) {
    int max = 0;
    Point max_point;
    for (auto &point : points) {
        if (a[point.x][point.y] > max) {
            max_point = point;
            max = a[point.x][point.y];
        }
    }
    return max_point;
}

bool is_valid(Point evaluated, Point reference, int proximity_limit) {
    if (evaluated == reference) return false;
    return !((abs(evaluated.x - reference.x) < proximity_limit) or (abs(evaluated.y - reference.y) < proximity_limit));
}

vector<Point> remove_too_close_to(const Point &selected, vector<Point> all, int proximity_limit) {
    vector<Point> difference;
    for (auto &point : all) {
        if (is_valid(selected, point, proximity_limit)) {
            difference.push_back(point);
        }
    }
    return difference;
}

vector<Point> remove_too_close(vector<Point> points, int a[768][1024], int proximity_limit) {
    vector<Point> final_points;
    Point selected_point;
    do {
        selected_point = get_max(points, a);
        final_points.push_back(selected_point);
        points = remove_too_close_to(selected_point, points, proximity_limit);
    } while (!points.empty());
    return final_points;
}

/// Get the pixel most relevant as circle center
Point max_strength(int a[768][1024]) {
    Point max_point;
    int max = 0;
    for (int i = 0; i < 768; i++) {
        for (int j = 0; j < 1024; j++) {
            if (max < a[i][j]) {
                max = a[i][j];
                max_point = Point(i, j);
            }
        }
    }
    return max_point;
}

int main(int argc, char *argv[]) {
    cv::CommandLineParser parser(argc, argv,
            // name  | default value    | help message
                                 "{help   |                  | print this message}"
                                 "{@image | ../easy/marker_color_01.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }


    /// Load image file
    std::string filepath = "../easy/marker_color_01.png";
    cv::Mat img_original = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);

    if (img_original.empty()) {
        std::cout << "Input image not found at '" << filepath << "'\n";
        return 1;
    }

    Mat img_processed = img_original;

    double theta, F_theta;
    unsigned long counter = 0;
    std::cout << img_original.rows << std::endl;
    std::cout << img_original.cols << std::endl;
    int r0, c0, r, c, rad, rimg = img_original.rows, cimg = img_original.cols;
    int ac[768][1024];
    for (auto &rac : ac) {
        for (int &cac : rac) {
            cac = 0;
        }
    }
    for (r = 1; r < rimg; r++) {
        for (c = 1; c < cimg; c++) {
            std::tie(theta, F_theta) = compute_theta(img_original, r, c);
            rad = int(round(F_theta));
            r0 = int(round(r - rad * cos(theta)));
            c0 = int(round(c - rad * sin(theta)));

            if ((r0 > 0) and (c0 > 0) and (r0 < rimg) and (c0 < cimg)) {
                counter += 1;
                /// Increase circle center probability
                ac[r0][c0] += 1;
            }
        }
    }

    std::cout << counter << std::endl;

    int processed_ac[768][1024];
    for (int rac = 0; rac < 768; rac++) {
        for (int cac = 0; cac < 1024; cac++) {
            processed_ac[rac][cac] = ac[rac][cac];
        }
    }

    /// Set pixel value to the sum of adjacent pixels (try to find clustered high strength pixels)
    int averaging_area = 21;
    int averaging_constraint = (averaging_area - 1) / 2;
    int total_strength = 0;

    for (r = averaging_constraint; r < (rimg - averaging_constraint); r++) {
        for (c = averaging_constraint; c < (cimg - averaging_constraint); c++) {
            total_strength = 0;
            for (int i = -averaging_constraint; i <= averaging_constraint; i++) {
                for (int j = -averaging_constraint; j <= averaging_constraint; j++) {
                    total_strength += ac[r + i][c + j];
                }
            }
            processed_ac[r][c] = total_strength;
        }
    }

    ///Idea for next approach:
    /// 1. Find max strength pixel in processed_ac
    /// 2. Add it as circle center
    /// 3. Set pixel and adjacent pixels strength to 0 to eliminate the already found circle center cluster
    /// 4. Repeat for 3 more points
    /// 5. Check if all 4 circle centers are on the marker

///Possibly reusable code (delete at final version)

//    vector<Point> best_points(100);
//    Point min_point;
//    int min;
//    int min_distance = int(round(rimg / 8));
//
//    for (r = 0; r < rimg; r++) {
//        for (c = 0; c < cimg; c++) {
//            min_point = get_minimum(processed_ac, best_points);
//            min = processed_ac[min_point.x][min_point.y];
//            if (processed_ac[r][c] > min) {
//                best_points = replace(r, c, min_point, best_points);
//            }
//        }
//    }
//
//    best_points = remove_too_close(best_points, processed_ac, min_distance);
//
//    vector<int> top_strengths;
//    int best = 100;
//    for (int j = 0; j < best; j++) {
//        top_strengths.push_back(ac[0][j]);
//    }
//    for (int i = 0; i < img_original.rows; i++) {
//        for (int j = best; j < img_original.cols; j++) {
//            std::sort(top_strengths.begin(), top_strengths.end());
//            if (top_strengths.at(0) < processed_ac[i][j]) {
//                top_strengths.at(0) = processed_ac[i][j];
//            }
//        }
//    }

//    for (size_t i = 0; i < 20; i++) {
//        Point center(, cvRound(circles[i][1]));
//        int radius = 50;
//         circle center
//        circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//         circle outline
//        circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//    }

//                P[r0 + cimg * (c0 + 101 * rad)].push_back({r, c});
//            }
//        }
//    }
//

// Make points black?
//    for (int row = 0; row <= img_original.rows; row++) {
//        for (int col = 0; col <= img_original.cols; col++) {
//            for (rad = 50; rad <= 100; rad++) {
//                if (A[row + img_original.rows * (col + 101 * rad)]>=50) {
//                    Vec3b color = img_original.at<Vec3b>(Point(row, col));
//                    color.val[0] = 255;
//                    color.val[1] = 255;
//                    color.val[2] = 255;
//                    img_processed.at<Vec3b>(Point(row, col)) = color;
//                }
//            }
//        }
//    }
//
//    imwrite("../Results/Marker1_Detection.png", img_processed);

    return 0;
}
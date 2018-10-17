/*
  RoVi1
    miniproject image2
*/

#include "../../lib/functions.h"

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
    // Parse command line arguments -- the first positional argument expects an
    // image path (the default is ./book_cover.jpg)
    cv::CommandLineParser parser(argc, argv,
            // name  | default value    | help message
                                 "{help   |                  | print this message}"
                                 "{@image | ../Image1.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image file
    std::string filepath = parser.get<std::string>("@image");
    cv::Mat img_original = cv::imread(filepath, CV_LOAD_IMAGE_GRAYSCALE);

    // Check that the image file was actually loaded
    if (img_original.empty()) {
        std::cout << "Input image not found at '" << filepath << "'\n";
        return 1;
    }
    /**************************************************************************
     ************************Problem detection*********************************
     **************************************************************************/

    Mat hist_orig = histogram_creation(img_original);   //histogram original

    /**************************************************************************
     ************************image correction**********************************
     **************************************************************************/

//    int z_min = 50;
//    int z_max = 175;
//    int max_win_size = 15;
    Mat filter_img1 = contraharmonic_filter(img_original, 5, 0.01);
    Mat filter_img2 = contraharmonic_filter(img_original, 5, 0.1);
    Mat filter_img3 = contraharmonic_filter(img_original, 5, 0.5);
//    Mat img_int_trans=intesity_transf(filter_mean_img,50);
    Mat img_trans_eq;
    equalizeHist(filter_img1, img_trans_eq);
//    Mat kernel = (Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1) / 9;
//    Point anchor = Point(-1, -1);
//    filter2D(img_original, img_trans_eq, CV_8U, kernel, anchor, 0, BORDER_DEFAULT);
//    Mat filter_mean_img = mean_filter(img_trans_eq);

    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/

    imshow_res("original image", img_original, img_original.cols / 2, img_original.rows / 2);
    imshow_res("filtered image0.01", filter_img1, filter_img1.cols / 2, filter_img1.rows / 2);
    imshow_res("filtered image0.1", filter_img2, filter_img2.cols / 2, filter_img2.rows / 2);
    imshow_res("filtered image0.5", filter_img3, filter_img3.cols / 2, filter_img3.rows / 2);
//    imshow_res("filtered image mean", filter_mean_img, filter_mean_img.cols / 2, filter_mean_img.rows / 2);
//    imshow_res("filtered image brighter", img_int_trans,img_int_trans.cols/2, img_int_trans.rows/2);
    imshow_res("filtered image equalize", img_trans_eq, img_trans_eq.cols / 2, img_trans_eq.rows / 2);
    imshow("original image histogram", hist_orig);

    // Wait for escape key press before returning
    while (cv::waitKey() != 27); // (do nothing)

    return 0;
}

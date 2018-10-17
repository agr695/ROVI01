/*
  RoVi1
    miniproject image2
*/

#include "../src/functions.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    // Parse command line arguments -- the first positional argument expects an
    // image path (the default is ./book_cover.jpg)
    cv::CommandLineParser parser(argc, argv,
        // name  | default value    | help message
        "{help   |                  | print this message}"
        "{@image | ../Image2.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image file
    std::string filepath = parser.get<std::string>("@image");
    cv::Mat img_original = cv::imread(filepath,CV_LOAD_IMAGE_GRAYSCALE);

    // Check that the image file was actually loaded
    if (img_original.empty()) {
        std::cout << "Input image not found at '" << filepath << "'\n";
        return 1;
    }
    /**************************************************************************
     ************************Problem detection*********************************
     **************************************************************************/
    Mat hist_orig=histogram_creation(img_original);   //histogram original

    /**************************************************************************
     ************************image correction**********************************
     **************************************************************************/
     int z_min=50;
     int z_max=225;
     int max_win_size=15;
     Mat filter_img=adapt_med_filter(img_original,z_min,z_max,max_win_size);
     Mat hist_filter=histogram_creation(filter_img);
     Mat filter_mean_img=mean_filter(filter_img,3);
     Mat hist_filter_mean=histogram_creation(filter_mean_img);
     Mat img_trans_eq;
     equalizeHist(filter_mean_img,img_trans_eq);
    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/
    imshow_res("original image", img_original,img_original.cols/2, img_original.rows/2);
    imshow_res("filtered image", filter_img,filter_img.cols/2, filter_img.rows/2);
    imshow_res("filtered image mean", filter_mean_img,filter_mean_img.cols/2, filter_mean_img.rows/2);
    imshow_res("filtered image equalize", img_trans_eq,img_trans_eq.cols/2, img_trans_eq.rows/2);

    imshow("original image histogram", hist_orig);
    imshow("median filtered image histogram", hist_filter);
    imshow("mean filtered image histogram", hist_filter_mean);

    // Wait for escape key press before returning
    while (cv::waitKey() != 27)
        ; // (do nothing)

    return 0;
}

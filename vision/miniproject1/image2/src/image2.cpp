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
                                 "{@image | ../Image2.png | image path}"
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
    cv::Mat padded;
    int opt_rows = cv::getOptimalDFTSize(img_original.rows * 2 - 1);
    int opt_cols = cv::getOptimalDFTSize(img_original.cols * 2 - 1);
    cv::copyMakeBorder(img_original, padded, 0, opt_rows - img_original.rows, 0, opt_cols - img_original.cols,
                       cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {
        cv::Mat_<float>(padded),
        cv::Mat_<float>::zeros(padded.size())
    };
    cv::Mat complex;
    cv::merge(planes, 2, complex);

    // Compute DFT of image
    cv::dft(complex, complex);

    // Split real and complex planes
    cv::split(complex, planes);

    // Compute the magnitude and phase
    cv::Mat mag, phase;
    cv::cartToPolar(planes[0], planes[1], mag, phase);

    // Shift quadrants so the Fourier image origin is in the center of the image
    dftshift(mag);
    Mat mag_orig=mag.clone();
    mag_orig += cv::Scalar::all(1);
    cv::log(mag_orig, mag_orig);
    normalize(mag_orig, mag_orig, 0, 1, cv::NORM_MINMAX);
    imshow_res("Original Magnitude", mag_orig,mag_orig.rows/4,mag_orig.cols/4);

    /**************************************************************************
     ************************image correction**********************************
     **************************************************************************/
    int z_min = 50;
    int z_max = 225;
    int max_win_size = 15;
    Mat filter_img1 = adapt_med_filter(img_original, z_min, z_max, max_win_size);

    z_min = 50;
    z_max = 225;
    max_win_size = 5;
    Mat filter_img2 = adapt_med_filter(img_original, z_min, z_max, max_win_size);

    z_min = 50;
    z_max = 225;
    max_win_size = 21;
    Mat filter_img3 = adapt_med_filter(img_original, z_min, z_max, max_win_size);

    z_min = 0;
    z_max = 255;
    max_win_size = 15;
    Mat filter_img4 = adapt_med_filter(img_original, z_min, z_max, max_win_size);

    Mat hist_filter = histogram_creation(filter_img1);
    Mat filter_mean_img = mean_filter(filter_img1, 3);
    Mat hist_filter_mean = histogram_creation(filter_mean_img);
    Mat img_trans_eq;
    equalizeHist(filter_mean_img, img_trans_eq);
    Mat hist_filter_mean_eq = histogram_creation(img_trans_eq);
    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/
    imshow_res("original image", img_original, img_original.cols / 2, img_original.rows / 2);
    imshow_res("filtered image", filter_img1, filter_img1.cols / 2, filter_img1.rows / 2);
    imshow_res("filtered image mean", filter_mean_img, filter_mean_img.cols / 2, filter_mean_img.rows / 2);
    imshow_res("filtered image equalize", img_trans_eq, img_trans_eq.cols / 2, img_trans_eq.rows / 2);
    imshow("original image histogram", hist_orig);
    imshow("median filtered image histogram", hist_filter);
    imshow("mean filtered image histogram", hist_filter_mean);

    /**************************************************************************
     ***********************save images and histograms*************************
     **************************************************************************/
    // imwrite("../Results/Image2_Filter1.png", filter_img1);
    // imwrite("../Results/Image2_Filter2.png", filter_img2);
    // imwrite("../Results/Image2_Filter3.png", filter_img3);
    // imwrite("../Results/Image2_Filter4.png", filter_img4);
    // imwrite("../Results/Image2_Filter1_mean.png", filter_mean_img);
    // imwrite("../Results/Image2_Filter1_mean_equalize.png", img_trans_eq);
    // imwrite("../Results/Image2_Histogram_original.png", hist_orig);
    // imwrite("../Results/Image2_Histogram_filter1.png", hist_filter);
    // imwrite("../Results/Image2_Histogram_filter1_mean.png", hist_filter_mean);
    // imwrite("../Results/Image2_Histogram_filter1_mean_equalize.png", hist_filter_mean_eq);
    // mag_orig.convertTo(mag_orig, CV_8UC1, 255);
    // imwrite("../Results/Image2_Magnitude.png", mag_orig);

    std::vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(25);
    imwrite("../Results/Image2_Filter1.jpeg", filter_img1,params);
    imwrite("../Results/Image2_Filter2.jpeg", filter_img2,params);
    imwrite("../Results/Image2_Filter3.jpeg", filter_img3,params);
    imwrite("../Results/Image2_Filter4.jpeg", filter_img4,params);
    imwrite("../Results/Image2_Filter1_mean.jpeg", filter_mean_img,params);
    imwrite("../Results/Image2_Filter1_mean_equalize.jpeg", img_trans_eq,params);
    imwrite("../Results/Image2_Histogram_original.jpeg", hist_orig,params);
    imwrite("../Results/Image2_Histogram_filter1.jpeg", hist_filter,params);
    imwrite("../Results/Image2_Histogram_filter1_mean.jpeg", hist_filter_mean,params);
    imwrite("../Results/Image2_Histogram_filter1_mean_equalize.jpeg", hist_filter_mean_eq,params);
    mag_orig.convertTo(mag_orig, CV_8UC1, 255);
    imwrite("../Results/Image2_Magnitude.jpeg", mag_orig,params);
    
    // Wait for escape key press before returning
    while (cv::waitKey() != 27); // (do nothing)

    return 0;
}

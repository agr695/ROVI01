/*
  RoVi1
    miniproject image1
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

    cv::Mat padded;
    int opt_rows = cv::getOptimalDFTSize(img_original.rows * 2 - 1);
    int opt_cols = cv::getOptimalDFTSize(img_original.cols * 2 - 1);
    cv::copyMakeBorder(img_original, padded, 0, opt_rows - img_original.rows, 0, opt_cols - img_original.cols,
                       cv::BORDER_CONSTANT, cv::Scalar::all(0));

    // Make place for both the real and complex values by merging planes into a
    // cv::Mat with 2 channels.
    // Use float element type because frequency domain ranges are large.
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

    Mat filter_img1 = contraharmonic_filter(img_original, 5, 0.01);
    Mat filter_img2 = contraharmonic_filter(img_original, 5, 0.1);
    Mat filter_img3 = contraharmonic_filter(img_original, 5, 0.5);
    Mat img_trans_eq;
    equalizeHist(filter_img1, img_trans_eq);
    Mat hist_filter1_eq = histogram_creation(img_trans_eq);
    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/

    imshow_res("original image", img_original, img_original.cols / 2, img_original.rows / 2);
    imshow_res("filtered image0.01", filter_img1, filter_img1.cols / 2, filter_img1.rows / 2);
    imshow_res("filtered image0.1", filter_img2, filter_img2.cols / 2, filter_img2.rows / 2);
    imshow_res("filtered image0.5", filter_img3, filter_img3.cols / 2, filter_img3.rows / 2);
    imshow_res("filtered image equalize", img_trans_eq, img_trans_eq.cols / 2, img_trans_eq.rows / 2);
    imshow("original image histogram", hist_orig);
    hist_filter1_eq.convertTo(hist_filter1_eq, CV_8UC1, 255.0);
    imshow("original image histogram", hist_filter1_eq);

    /**************************************************************************
     ***********************save images and histograms*************************
     **************************************************************************/
    imwrite("../Results/Image1_Filter1.png", filter_img1);
    imwrite("../Results/Image1_Filter2.png", filter_img2);
    imwrite("../Results/Image1_Filter3.png", filter_img3);
    imwrite("../Results/Image1_Filter1_equalize.png", img_trans_eq);
    imwrite("../Results/Image1_Histogram_original.png", hist_orig);
    imwrite("../Results/Image1_Histogram_filter1.png", hist_filter1_eq);
    mag_orig.convertTo(mag_orig, CV_8UC1, 255);
    imwrite("../Results/Image1_Magnitude.png", mag_orig);


    // Wait for escape key press before returning
    while (cv::waitKey() != 27); // (do nothing)

    return 0;
}

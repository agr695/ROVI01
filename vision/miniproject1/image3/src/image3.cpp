/*
  RoVi1
    miniproject image3
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
                                 "{@image | ../Image3.png | image path}"
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
    Mat filter_img = harmonic_filter(img_original, 3);
    Mat hist_filter = histogram_creation(filter_img);

    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/
    imshow_res("original image", img_original, img_original.cols / 2, img_original.rows / 2);
    imshow_res("filtered image", filter_img, filter_img.cols / 2, filter_img.rows / 2);
    imshow("original image histogram", hist_orig);
    imshow("median filtered image histogram", hist_filter);

    // Wait for escape key press before returning
    while (cv::waitKey() != 27); // (do nothing)

    return 0;
}

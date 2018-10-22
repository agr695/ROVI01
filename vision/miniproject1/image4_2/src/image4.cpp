/*
  RoVi1
    miniproject image4
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
                                 "{@image | ../Image4_2.png | image path}"
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
     mag=remove_circunference(mag,150,825);
     // Shift back quadrants of the spectrum
     dftshift(mag);

     // Compute complex DFT planes from magnitude/phase
     cv::polarToCart(mag, phase, planes[0], planes[1]);

     // Merge into one image
     cv::merge(planes, 2, complex);

     // Restore by performing inverse DFT
     cv::Mat filtered;
     cv::idft(complex, filtered, (cv::DFT_SCALE | cv::DFT_REAL_OUTPUT));

     // Crop image (remove padded borders)
     filtered = cv::Mat(filtered, cv::Rect(cv::Point(0, 0), img_original.size()));

     // Shift mag quadrants again for visualization purposes
     dftshift(mag);
     cv::normalize(filtered, filtered, 0, 1, cv::NORM_MINMAX);
     imshow_res("Filtered image", filtered,filtered.rows/2,filtered.cols/2);

    /**************************************************************************
     ***********************show images and histograms*************************
     **************************************************************************/
    mag += cv::Scalar::all(1);
    cv::log(mag, mag);
    normalize(mag, mag, 0, 1, cv::NORM_MINMAX);
    imshow_res("original image", img_original, img_original.cols / 2, img_original.rows / 2);
    imshow_res("Magnitude", mag,img_original.rows/4,img_original.cols/4);

    /**************************************************************************
     ***********************save images and histograms*************************
     **************************************************************************/
    filtered.convertTo(filtered, CV_8UC1, 255);
    imwrite("../Results/Image4_2_Filter.png", filtered);
    imwrite("../Results/Image4_2_Histogram_original.png", hist_orig);
    mag_orig.convertTo(mag_orig, CV_8UC1, 255);
    imwrite("../Results/Image4_2_Magnitude_original.png", mag_orig);
    mag.convertTo(mag, CV_8UC1, 255);
    imwrite("../Results/Image4_2_Magnitude_filtered.png", mag);
    // Wait for escape key press before returning
    while (cv::waitKey() != 27); // (do nothing)

    return 0;
}

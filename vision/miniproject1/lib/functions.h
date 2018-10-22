#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>


using namespace cv;
using namespace std;


/*Function to show an image in a resize window*/
void imshow_res(const string winname, Mat img, int width, int height) {
    namedWindow(winname, WINDOW_NORMAL);
    resizeWindow(winname, width, height);
    imshow(winname, img);
}

/*function to calculate the histogram of an image*/
/*https://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html*/
Mat histogram_creation(Mat img_orig) {
    Mat img = img_orig.clone();
    // Set histogram bins count
    int bins = 1024;
    int histSize[] = {bins};
    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float *ranges[] = {lranges};
    // create matrix for histogram
    Mat hist;
    int channels[] = {0};
    // create matrix for histogram visualization
    int const hist_height = 512;
    Mat hist_image(hist_height, bins,CV_8UC1,255);

    calcHist(&img, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);
    double max_val = 0;
    minMaxLoc(hist, 0, &max_val);

    // visualize each bin
    for (int b = 0; b < bins; b++) {
        float const binVal = hist.at<float>(b);
        int const height = cvRound(binVal * hist_height / max_val);
        line(hist_image,
             Point(b, hist_height - height),
             Point(b, hist_height),
             Scalar::all(0),
             2);
    }
    return hist_image;
}

/*Function to calculate the median of a vector*/
unsigned char median(vector<unsigned char> vect) {
    int length = vect.size();
    sort(vect.begin(), vect.begin() + length);
    return vect[(length - 1) / 2];
}

/*Function to compute an adaptive median filter*/
Mat adapt_med_filter(Mat img, const int z_min, const int z_max, const int max_win_size) {
    Mat ret(img.rows-max_win_size,img.cols-max_win_size,CV_8UC1);
    int win_size; //current windows size
    int M = ret.cols;
    int N = ret.rows;
    bool cont;//flag to indicate if the value is correct
    unsigned int median_value;
    int i, j, k, t;
    unsigned char *neighbours=new unsigned char[max_win_size * max_win_size];
    vector<unsigned char> vector_median;
    int index = 0;

    /*loops to follow the image*/
    for (i = 0; i < M; i++) {
        for (j = 0; j < N; j++) {
            win_size = 1;
            cont = true;
            while (cont && win_size <=  static_cast<int>(max_win_size)) {
                index = 0;
                /*loops to explore the neighbours of the current pixel*/
                for (k = -(win_size - 1) / 2; k <= (win_size - 1) / 2; k++) {
                    for (t = -(win_size - 1) / 2; t <= (win_size - 1) / 2; t++) {
                        neighbours[index] = img.at<uchar>(Point(i+max_win_size + k, j+max_win_size + t));
                        index++;
                    }
                }
                /*get the median value of the neighbours*/
                vector_median.assign(neighbours, neighbours + win_size);
                median_value = median(vector_median);
                if ( static_cast<int>(median_value) > z_min &&  static_cast<int>(median_value) < z_max) {
                    cont = false;
                }
                win_size += 2;
            }
            ret.at<uchar>(Point(i, j)) = median_value;
        }
    }
    delete [] neighbours;
    return ret;
}

/*Function to applied a mean filter*/
Mat mean_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    int counter = 0;
    unsigned char value;
    int M = img.cols - (size - 1) / 2;
    int N = img.rows - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            /*loops to explore the neighbours of the current pixel*/
            for (k = -(size - 1) / 2; k <= (size - 1) / 2; k++) {
                for (l = -(size - 1) / 2; l <= (size - 1) / 2; l++) {
                    counter += img.at<uchar>(Point(i + k, j + l));
                }
            }
            value = counter / (size * size);
            counter = 0;
            filtered.at<uchar>(Point(i, j)) = value;
        }
    }
    return filtered;
}

Mat wiener_filter(Mat mag, float k, int a) {
    Mat filtered = mag.clone();
    int i, j;
    float H;
    float value;
    int M = mag.cols;
    int N = mag.rows;
//    int T = M * N;
//    float arg;
    float rad2deg = static_cast<float>(M_PI / 180);

    for (i = 1; i < M; i++) {
        for (j = 0; j < N; j++) {
//            arg = static_cast<float>(M_PI * (i * a + j * b));
//            H = (1/2) * (T/arg) * sin(2 * arg * rad2deg);
            if (i == 0) {
                H = 1;
            }
            else {
                H = static_cast<float>(sin(M_PI * i * a * rad2deg));
            }
            value = H / (H*H + k) * mag.at<float>(Point(i, j));
            filtered.at<float>(Point(i, j)) = value;
        }

    }

    return filtered;
}

/*Function to applied a contraharmonic filter*/
Mat contraharmonic_filter(Mat img, int size, double Q) {
    Mat filtered = img.clone();
    int i, j, k, l;
    double num = 0;
    double nom = 0;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            /*loops to explore the neighbours of the current pixel*/
            for (k = -((size - 1) / 2); k <= (size - 1) / 2; k++) {
                for (l = -((size - 1) / 2); l <= (size - 1) / 2; l++) {
                    num += pow(img.at<uchar>(Point(i + k, j + l)), Q + 1);
                    nom += pow(img.at<uchar>(Point(i + k, j + l)), Q);
                }
            }
            if (num == 0) {
                num = 0.01;
            }
            value = static_cast<unsigned char>(num / nom);
            num = 0;
            nom = 0;
            filtered.at<uchar>(Point(i, j)) = value;
        }
    }
    return filtered;
}

/*Function to applied an armonic filter*/
Mat harmonic_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    double counter = 0;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            /*loops to explore the neighbours of the current pixel*/
            for (k = -((size - 1) / 2); k <= (size - 1) / 2; k++) {
                for (l = -((size - 1) / 2); l <= (size - 1) / 2; l++) {
                    if (img.at<uchar>(Point(i + k, j + l)) == 0) {
                        counter += 100.0;
                    } else {
                        counter += 1.0 / img.at<uchar>(Point(i + k, j + l));
                    }
                }
            }
            value = static_cast<unsigned char>((size * size) / counter);
            filtered.at<uchar>(Point(i, j)) = value;
            counter = 0;
        }
    }
    return filtered;
}

/*Function to applied a geometric mean filter*/
Mat geom_mean_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    long long int counter = 1;
    unsigned char value;
    int M = img.cols - (size - 1) / 2;
    int N = img.rows - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
          /*loops to explore the neighbours of the current pixel*/
            for (k = -(size - 1) / 2; k <= (size - 1) / 2; k++) {
                for (l = -(size - 1) / 2; l <= (size - 1) / 2; l++) {
                    counter *= img.at<uchar>(Point(i + k, j + l));
                }
            }
            value = pow(counter, (1.0 / (size * size)));
            counter = 1;
            filtered.at<uchar>(Point(i, j)) = value;
        }
    }
    return filtered;
}

/*Function to applied an intensity transformation*/
Mat intesity_transf(Mat img, int intens_inc) {
    Mat trans = img.clone();
    int i, j;
    int M = img.rows;
    int N = img.cols;

    /*loops to follow the image*/
    for (i = 0; i < M; i++)
        for (j = 0; j < N; j++)
            trans.at<uchar>(i, j) = saturate_cast<uchar>(img.at<uchar>(i, j) + intens_inc);

    return trans;
}

/*Function to shift the DFT magnitude*/
/*https://docs.opencv.org/3.1.0/d8/d01/tutorial_discrete_fourier_transform.html*/
void dftshift(cv::Mat &mag) {
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;

    cv::Mat tmp;
    cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));

    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}

/*Function to remove a circunference in the magnitude DFT spectrum*/
Mat remove_circunference(Mat mag, int size, int rad) {
    Mat ret = mag.clone();
    int N = ret.rows;
    int M = ret.cols;

    circle(ret,
           Point(M / 2, N / 2),
           rad,
           Scalar(0, 0, 0),
           size);

    return ret;

}

/*Function to remove a point in the magnitude DFT spectrum*/
Mat remove_point(Mat mag, int rad, int center_x, int center_y) {
    Mat ret = mag.clone();

    circle(ret,
           Point(center_x, center_y),
           rad,
           Scalar(0, 0, 0),
           -1);

    return ret;
}

/*Function to applied a laplacian filter*/
Mat laplacian_filter(Mat img) {
    Mat filtered = img.clone();
    int i, j, k, l;
    int counter = 0;
    int size = 3;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            /*loops to explore the neighbours of the current pixel*/
            for (k = -((size - 1) / 2); k <= (size - 1) / 2; k++) {
                for (l = -((size - 1) / 2); l <= (size - 1) / 2; l++) {
                    if (k == 0 && l == 0) { //the current pixel
                        counter += 9 * img.at<uchar>(Point(i + k, j + l));
                    } else {
                        counter -= img.at<uchar>(Point(i + k, j + l));
                    }
                }
            }
            value = static_cast<unsigned char> (counter);
            filtered.at<uchar>(Point(i, j)) = value;
            counter = 0;
        }
    }
    return filtered;
}

/*Function to applied a sobel filter*/
Mat sobel_filter(Mat img) {
    Mat filtered = img.clone();
    int i, j, k, l;
    int counter = 0;
    int size = 3;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    /*loops to follow the image*/
    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            /*loops to explore the neighbours of the current pixel*/
            for (k = -((size - 1) / 2); k <= (size - 1) / 2; k++) {
                for (l = -((size - 1) / 2); l <= (size - 1) / 2; l++) {
                    if (l == 0) { //Central column
                        counter += k * 2 * img.at<uchar>(Point(i + k, j + l));
                    } else {
                        counter += k * img.at<uchar>(Point(i + k, j + l));
                    }
                }
            }
            value = static_cast<unsigned char> (counter);
            filtered.at<uchar>(Point(i, j)) = value + img.at<uchar>(Point(i, j));
            counter = 0;
        }
    }
    return filtered;
}

#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>


using namespace cv;
using namespace std;


/*Function to show an image in a resize window*/
void imshow_res(const string winname, Mat img, int width, int height) {
    namedWindow(winname, WINDOW_NORMAL);
    resizeWindow(winname, width, height);
    imshow(winname, img);
}

/*function to calculate the histogram of an image*/
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
    Mat3b hist_image = Mat3b::zeros(hist_height, bins);

    calcHist(&img, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);
    double max_val = 0;
    minMaxLoc(hist, 0, &max_val);

    // visualize each bin
    for (int b = 0; b < bins; b++) {
        float const binVal = hist.at<float>(b);
        int const height = cvRound(binVal * hist_height / max_val);
        line
                (hist_image, Point(b, hist_height - height), Point(b, hist_height), Scalar::all(255)
                );
    }
    return hist_image;
}

unsigned char median(vector<unsigned char> vect) {
    int length = vect.size();
    sort(vect.begin(), vect.begin() + length);
    return vect[(length - 1) / 2];
}

Mat adapt_med_filter(Mat img, const int z_min, const int z_max, const int max_win_size) {
    Mat ret = img.clone();
    int win_size;
    int start = max_win_size;
    int stop_i = ret.cols - max_win_size;
    int stop_j = ret.rows - max_win_size;
    bool cont;
    unsigned int median_value;
    int i, j, k, t;
    unsigned char neighbours[max_win_size * max_win_size];
    vector<unsigned char> vector_median;
    int counter = 0;


    for (i = start; i < stop_i; i++) {
        for (j = start; j < stop_j; j++) {
            win_size = 1;
            cont = true;
            while (cont && win_size <= max_win_size) {
                counter = 0;
                for (k = -(win_size - 1) / 2; k <= (win_size - 1) / 2; k++) {
                    for (t = -(win_size - 1) / 2; t <= (win_size - 1) / 2; t++) {
                        neighbours[counter] = img.at<uchar>(Point(i + k, j + t));
                        counter++;
                        // std::cout << int(neighbours[counter]) << '\n';
                    }
                }
                // cout<<endl;
                vector_median.assign(neighbours, neighbours + win_size);
                median_value = median(vector_median);
                if (median_value > z_min && median_value < z_max) {
                    cont = false;
                }
                win_size += 2;
            }
            ret.at<uchar>(Point(i, j)) = median_value;
            // std::cout <<int(ret.at<uchar >(j,i))<<"   "<< int(median_value) << "  " <<j<< "   " << i << "   "<< '\n';
        }
    }

    return ret;
}


Mat mean_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    int cont = 0;
    unsigned char value;
    int M = img.cols - (size - 1) / 2;
    int N = img.rows - (size - 1) / 2;

    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            for (k = -(size - 1) / 2; k <= (size - 1) / 2; k++) {
                for (l = -(size - 1) / 2; l <= (size - 1) / 2; l++) {
                    cont += img.at<uchar>(Point(i + k, j + l));
                }
            }
            value = cont / (size * size);
            cont = 0;
            filtered.at<uchar>(Point(i, j)) = value;
        }
    }
    return filtered;
}

Mat contraharmonic_filter(Mat img, int size, double Q) {
    Mat filtered = img.clone();
    int i, j, k, l;
    double num = 0;
    double nom = 0;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
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

Mat harmonic_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    double cont = 0;
    unsigned char value;
    int N = img.rows - (size - 1) / 2;
    int M = img.cols - (size - 1) / 2;

    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            for (k = -((size - 1) / 2); k <= (size - 1) / 2; k++) {
                for (l = -((size - 1) / 2); l <= (size - 1) / 2; l++) {
                    if (img.at<uchar>(Point(i + k, j + l)) == 0) {
                        cont += 100.0;
                    }
                    else {
                        cont += 1.0 / img.at<uchar>(Point(i + k, j + l));
                    }
                }
            }
            value = static_cast<unsigned char>((size * size)/ cont);
            filtered.at<uchar>(Point(i, j)) = value;
            cont = 0;
        }
    }
    return filtered;
}

Mat geom_mean_filter(Mat img, int size) {
    Mat filtered = img.clone();
    int i, j, k, l;
    long long int cont = 1;
    unsigned char value;
    int M = img.cols - (size - 1) / 2;
    int N = img.rows - (size - 1) / 2;

    for (i = (size - 1) / 2; i < M; i++) {
        for (j = (size - 1) / 2; j < N; j++) {
            for (k = -(size - 1) / 2; k <= (size - 1) / 2; k++) {
                for (l = -(size - 1) / 2; l <= (size - 1) / 2; l++) {
                    cont *= img.at<uchar>(Point(i + k, j + l));
                }
            }
            value = pow(cont, (1.0 / (size * size)));
            cont = 1;
            filtered.at<uchar>(Point(i, j)) = value;
        }
    }
    return filtered;
}

Mat intesity_transf(Mat img, int intens_inc) {
    Mat trans = img.clone();
    int i, j;
    int M = img.rows;

    int N = img.cols;

    for (i = 0; i < M; i++)
        for (j = 0; j < N; j++)
            trans.at<uchar>(i, j) = saturate_cast<uchar>(img.at<uchar>(i, j) + intens_inc);

    return trans;
}




void dftshift(cv::Mat& mag)
{
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


Mat remove_ellipse(Mat mag,int size,int rad){
  Mat ret=mag.clone();
  int N=ret.rows;
  int M=ret.cols;


  ellipse( ret,
          Point(M/2,N/2),
          Size(rad,rad/3),
          -45,
          0,
          360,
          Scalar(0,0,0),
          size);

  return ret;

}


Mat remove_circunference(Mat mag,int size,int rad){
  Mat ret=mag.clone();
  int N=ret.rows;
  int M=ret.cols;

  circle( ret,
          Point(M/2,N/2),
          rad,
          Scalar(0,0,0),
          size);

  return ret;

}

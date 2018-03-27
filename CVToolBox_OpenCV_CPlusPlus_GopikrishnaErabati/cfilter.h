#ifndef CFILTER_H
#define CFILTER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Cfilter
{
private:
    cv::Mat image_filtered; // to store filtered image
    cv::Mat sobelX, sobelY;
    cv::Mat sobel;
    cv::Mat sobelImage;
    double sobmin, sobmax;

public:
    Cfilter();
    // function to get box blurred image
    cv::Mat getbox(cv::Mat image, int kerSize);

    // function to get gausiian blurred image
    cv::Mat getGaussianBlurr(cv::Mat image, int kerSize, double alongX, double alongY);

    // function to get median blurr
    cv::Mat getMedianBlurr(cv::Mat image, int kerSize);

    //function to get bilateral blurr
    cv::Mat getBilateralBlurr(cv::Mat image, int kerSize, double sdColor, double sdSpatial);

    // function to get sobel filter
    cv::Mat getSobel(cv::Mat image, int kerSize, double derAlongX, double derAlongY);

    // function to get scharr filter
    cv::Mat getScharr(cv::Mat image, double derAlongX, double derAlongY);

    // function to get laplacian filter
    cv::Mat getLaplacian(cv::Mat image, int kerSize);

    // function to get edges using sobel
    cv::Mat getEdgesSobel(cv::Mat image, double threshold_value);

};

#endif // CFILTER_H

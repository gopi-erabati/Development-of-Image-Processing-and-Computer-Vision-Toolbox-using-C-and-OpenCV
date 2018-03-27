#include "cfilter.h"

Cfilter::Cfilter()
{

}

// function to get box blurr
cv::Mat Cfilter::getbox(cv::Mat image, int kerSize){
    cv::blur(image,image_filtered, cv::Size(2*kerSize + 1, 2*kerSize + 1), cv::Point(kerSize, kerSize));
    return image_filtered;
}

// function to get gausiian blurred image
cv::Mat Cfilter::getGaussianBlurr(cv::Mat image, int kerSize, double alongX, double alongY){
    cv::GaussianBlur(image,image_filtered, cv::Size(2*kerSize + 1, 2*kerSize + 1), alongX, alongY);
    return image_filtered;
}

// function to get median blurr
cv::Mat Cfilter::getMedianBlurr(cv::Mat image, int kerSize){
    cv::medianBlur(image, image_filtered, kerSize);
    return image_filtered;
}

//function to get bilateral blurr
cv::Mat Cfilter::getBilateralBlurr(cv::Mat image, int kerSize, double sdColor, double sdSpatial){
    cv::bilateralFilter(image, image_filtered, kerSize, sdColor, sdSpatial);
    return image_filtered;
}

// function to get sobel filter
cv::Mat Cfilter::getSobel(cv::Mat image, int kerSize, double derAlongX, double derAlongY){
    cv::Sobel(image, image_filtered, CV_8U, derAlongX, derAlongY, kerSize, 0.4, 128);
    return image_filtered;
}

// function to get scharr filter
cv::Mat Cfilter::getScharr(cv::Mat image, double derAlongX, double derAlongY){
    cv::Scharr(image, image_filtered, CV_8U, derAlongX, derAlongY, 0.4, 128);
    return image_filtered;
}

// function to get laplacian filter
cv::Mat Cfilter::getLaplacian(cv::Mat image, int kerSize){
    cv::Laplacian(image, image_filtered, CV_8U, kerSize, 0.01, 128);
    return image_filtered;
}

// function to get edges using sobel
cv::Mat Cfilter::getEdgesSobel(cv::Mat image, double threshold_value){

    // Compute norm of Sobel
    cv::Sobel(image,sobelX,CV_16S,1,0);
    cv::Sobel(image,sobelY,CV_16S,0,1);

    //compute the L1 norm
    sobel= abs(sobelX)+abs(sobelY);

    cv::minMaxLoc(sobel,&sobmin,&sobmax);

    // Conversion to 8-bit image
    // sobelImage = -alpha*sobel + 255
    sobel.convertTo(sobelImage,CV_8U,-255./sobmax,255);

    // Apply threshold to Sobel norm
    cv::threshold(sobelImage, image_filtered, threshold_value, 255, cv::THRESH_BINARY);

    return image_filtered;
}



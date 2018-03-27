#ifndef LAPLACIANEDGE_H
#define LAPLACIANEDGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class LaplacianEdge
{
private:

    // original image
    cv::Mat img;

    // 32-bit float image containing the Laplacian
    cv::Mat laplace;

    double lapmin, lapmax;
    cv::Mat laplaceImage;

    // Aperture size of the laplacian kernel
    int kerSize;

    //creatre iterators for laplacian binary
    cv::Mat_<float>::const_iterator it;
    cv::Mat_<float>::const_iterator itend;
    cv::Mat_<float>::const_iterator itup;
    cv::Mat_<uchar>::iterator itout;

    //initialise binary image
    cv::Mat binary;



public:
    LaplacianEdge();

    // Compute the floating point Laplacian
    cv::Mat computeLaplacian(const cv::Mat& image, int kerSize);

    cv::Mat getLaplacianImage(double scale=-1.0);

    // Get a binary image of the zero-crossings
    // if the product of the two adjascent pixels is
    // less than threshold then this zero-crossing will be ignored
    cv::Mat getZeroCrossings(double threshold_value=1.0);

};

#endif // LAPLACIANEDGE_H

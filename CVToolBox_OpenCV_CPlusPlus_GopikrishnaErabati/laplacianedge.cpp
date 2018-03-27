#include "laplacianedge.h"

LaplacianEdge::LaplacianEdge()
{

}

// Compute the floating point Laplacian
cv::Mat LaplacianEdge::computeLaplacian(const cv::Mat& image, int kerSize){

    // Compute Laplacian
    cv::Laplacian(image,laplace,CV_32F,kerSize);

    return laplace;
}

// Get the Laplacian result in 8-bit image
// zero corresponds to gray level 128
// if no scale is provided, then the max value will be
// scaled to intensity 255
cv::Mat LaplacianEdge::getLaplacianImage(double scale) {

    if (scale<0) {
        cv::minMaxLoc(laplace,&lapmin,&lapmax);
        scale= 127/ std::max(-lapmin,lapmax);
    }

    laplace.convertTo(laplaceImage,CV_8U,scale,128);

    return laplaceImage;
}

// Get a binary image of the zero-crossings
// if the product of the two adjascent pixels is
// less than threshold then this zero-crossing will be ignored
cv::Mat LaplacianEdge::getZeroCrossings(double threshold_value){

    // Create the iterators
    it= laplace.begin<float>()+laplace.step1();
    itend= laplace.end<float>();
    itup= laplace.begin<float>();

    // Binary image initialize to white
    cv::Mat binary(laplace.size(), CV_8U, cv::Scalar(255));
    itout = binary.begin<uchar>()+binary.step1();

    // negate the input threshold value
    threshold_value *= -1.0;

    for ( ; it!= itend; ++it, ++itup, ++itout) {

        // if the product of two adjascent pixel is negative
        // then there is a sign change
        if (*it * *(it-1) < threshold_value)
            *itout= 0; // horizontal zero-crossing
        else if (*it * *itup < threshold_value)
            *itout= 0; // vertical zero-crossing
    }

    return binary;
}

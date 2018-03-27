#include "harriscorner.h"

HarrisCorner::HarrisCorner()
{

}

// to get Harris corners
cv::Mat HarrisCorner::getHarrisCorners(cv::Mat image, int neighbourhoodSize, int kernalSize, double parameterK, double qualityLevel, int neighbourhoodSizeNonMax){

    imageCorners = image.clone();

    setLocalMaxWindowSize(neighbourhoodSizeNonMax); //set nonmax supression kernal
    detectCorners(imageCorners, neighbourhoodSize, kernalSize, parameterK); // detect corners
    getCorners(points, qualityLevel ); // get corners by nonmax supression
    drawOnImage(imageCorners, points); //draw corners on image

    return imageCorners;

}

// to get good feature corners (uniform)
cv::Mat HarrisCorner::getUniformCorners(cv::Mat image, int maxCorners, double qualityLevel, double minDistance){

    // get uniform corners
    cv::goodFeaturesToTrack(image, cornersUniform, maxCorners, qualityLevel, minDistance);

    imageCorners = image.clone();
    // draw uniform corners
    drawOnImage(imageCorners, cornersUniform);

    return imageCorners;
}

// to set kernal for nonmax suppression
void HarrisCorner::setLocalMaxWindowSize(int size){
    nonMaxSize= size;
    kernel.create(nonMaxSize,nonMaxSize,CV_8U);
}

// Compute Harris corners
void HarrisCorner::detectCorners(const cv::Mat& image, int neighbourhoodSize, int kernalSize, double parameterK){

    // Harris computation
    cv::cornerHarris(image,cornerStrength,
               neighbourhoodSize,// neighborhood size
               kernalSize,     // aperture size
               parameterK);           // Harris parameter

    // internal threshold computation
    cv::minMaxLoc(cornerStrength,&minStrength,&maxStrength);

    // local maxima detection
    cv::dilate(cornerStrength,dilated,cv::Mat());
    cv::compare(cornerStrength,dilated,localMax,cv::CMP_EQ);
}

// Get the feature points vector from the computed Harris values
void HarrisCorner::getCorners(std::vector<cv::Point> &points, double qualityLevel){
    // Get the corner map
    cv::Mat cornerMap= getCornerMap(qualityLevel);
    // Get the corners
    getCorners(points, cornerMap);
}

// Get the corner map from the computed Harris values
cv::Mat HarrisCorner::getCornerMap(double qualityLevel){

    // thresholding the corner strength
    threshold = qualityLevel * maxStrength;
    cv::threshold(cornerStrength, cornerTh, threshold, 255, cv::THRESH_BINARY);

    // convert to 8-bit image
    cornerTh.convertTo(cornerMap,CV_8U);

    // non-maxima suppression
    cv::bitwise_and(cornerMap,localMax,cornerMap);

    return cornerMap;
}

// Get the feature points vector from the computed corner map
void HarrisCorner::getCorners(std::vector<cv::Point> &points, const cv::Mat& cornerMap){

    // Iterate over the pixels to obtain all feature points
    for( int y = 0; y < cornerMap.rows; y++ ) {
        rowPtr = cornerMap.ptr<uchar>(y);
        for( int x = 0; x < cornerMap.cols; x++ ) {
            // if it is a feature point
            if (rowPtr[x]) {
                points.push_back(cv::Point(x,y));
            }
        }
    }
}

// Draw circles at feature point locations on an image
void HarrisCorner::drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color, int radius, int thickness){

    itPoints = points.begin();

    // for all corners
    while (itPoints!=points.end()) {

        // draw a circle at each corner location
        cv::circle(image,*itPoints,radius,color,thickness);
        ++itPoints;
    }
}

// Draw circles at feature point locations on an image
void HarrisCorner::drawOnImage(cv::Mat &image, const std::vector<cv::Point2f> &points, cv::Scalar color, int radius, int thickness){

    itPoints2f = points.begin();

    // for all corners
    while (itPoints2f!=points.end()) {

        // draw a circle at each corner location
        cv::circle(image,*itPoints2f,radius,color,thickness);
        ++itPoints2f;
    }
}

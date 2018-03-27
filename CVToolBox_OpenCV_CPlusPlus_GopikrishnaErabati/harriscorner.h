#ifndef HARRISCORNER_H
#define HARRISCORNER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HarrisCorner
{
private:

    // 32-bit float image of corner strength
    cv::Mat cornerStrength;
    // 32-bit float image of thresholded corners
    cv::Mat cornerTh;
    // image of local maxima (internal)
    cv::Mat localMax;
    double minStrength; // not used
    // maximum strength for threshold computation
    double maxStrength;
    // calculated threshold (internal)
    double threshold;
    // kernel for non-max
    int nonMaxSize;
    cv::Mat kernel;
    cv::Mat dilated;  // temporary image
    cv::Mat cornerMap; //temporary image
    const uchar* rowPtr; // for points row pointer
    std::vector<cv::Point>::const_iterator itPoints; //iterator to iterate on points to draw circles
    std::vector<cv::Point2f>::const_iterator itPoints2f; //iterator to iterate on points to draw circles for floating points
    std::vector<cv::Point> points; // to store corner points (internal)
    cv::Mat imageCorners; // temporary image
    std::vector<cv::Point2f> cornersUniform; // for uniform corners

public:
    HarrisCorner();

    // to set local window size for non max supression
    void setLocalMaxWindowSize(int size);

    // Compute Harris corners
    void detectCorners(const cv::Mat& image, int neighbourhoodSize, int kernalSize, double parameterK);

    // Get the feature points vector from the computed Harris values
    void getCorners(std::vector<cv::Point> &points, double qualityLevel);

    // Get the corner map from the computed Harris values
    cv::Mat getCornerMap(double qualityLevel);

    // Get the feature points vector from the computed corner map
    void getCorners(std::vector<cv::Point> &points, const cv::Mat& cornerMap);

    // Draw circles at feature point locations on an image
    void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color= cv::Scalar(255,255,255), int radius=3, int thickness=2);

    // Draw circles at feature point locations on an image with floating point corners
    void drawOnImage(cv::Mat &image, const std::vector<cv::Point2f> &points, cv::Scalar color= cv::Scalar(255,255,255), int radius=3, int thickness=2);

    // to get Harris corners
    cv::Mat getHarrisCorners(cv::Mat image, int neighbourhoodSize, int kernalSize, double parameterK, double qualityLevel, int neighbourhoodSizeNonMax);


    // to get good feature corners (uniform)
    cv::Mat getUniformCorners(cv::Mat image, int maxCorners, double qualityLevel, double minDistance);

};

#endif // HARRISCORNER_H

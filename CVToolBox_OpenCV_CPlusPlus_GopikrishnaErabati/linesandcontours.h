#ifndef LINESANDCONTOURS_H
#define LINESANDCONTOURS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDebug>

#define PI 3.14

class LinesAndContours
{
private:
    cv::Mat edges; // to store edges of canny

    // vector containing the end points
    // of the detected lines
    std::vector<cv::Vec4i> lines;

    //cv::Point point1, point2; // to store points to draw lines

    std::vector<cv::Vec3f> circles; // to store circles;
    cv::Mat imageBlur;

    std::vector<std::vector<cv::Point>> contours; // to store contours
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imageContours;

    cv::Mat imageRectangle;
    cv::Rect rectangleBox; // to store bounding rectangle

    float radius;
    cv::Point2f center; // for min enclsoing circle

    //std::vector<cv::Point> polygon; // to store polygon points for polygon approax

    std::vector<cv::Point> hull; // to store points of convex hull

    cv::Moments moments; // to store moments

    std::vector<cv::Vec4i>::const_iterator it_lines; // iterator to iterate on line
    std::vector<cv::Vec3f>::const_iterator it_circles; // iterator to iteare on circles
    std::vector<std::vector<cv::Point>>::const_iterator it_contours; //ietartor to iteartae on contours
    std::vector<cv::Point>::const_iterator it_polygon; //ietartor to iterate on polygon segments
    std::vector<cv::Point>::const_iterator it_hull; //ietartor to iterate on hull segments

public:
    LinesAndContours();

    //function to detect edges using canny edge detector
    cv::Mat drawEdgeCanny(cv::Mat image, double lowThreshold, double highThreshold, int kernalSize);

    //function to get lines using hough transform
    cv::Mat drawLinesHough(cv::Mat image, double lowThreshold, double highThreshold, double distStepSize, double angleStepSize, int votes, double minLength, double maxGap);

    //function to draw circles using hough transform
    cv::Mat drawCirclesHough(cv::Mat image, double accumulatorResolution, double minDist, double highThreshold, double minVotes, int minRadius, int maxRadius);

    //function to get contours
    std::vector<std::vector<cv::Point>> getContours(cv::Mat image, int minSize, int maxSize);

    // function to draw contours
    cv::Mat myDrawContours(cv::Mat image, int minSize, int maxSize);

    // function to draw bounding box
    cv::Mat drawBoundingBox(const cv::Mat &image, int minSize, int maxSize);

    //function to draw minimum enclsoing circle
    cv::Mat drawMinEnclosingCircle(const cv::Mat &image, int minSize, int maxSize);

    //function to draw polygon approximation
    cv::Mat drawPolygonApprox(const cv::Mat &image, int minSize, int maxSize);

    //function to draw convex hull
    cv::Mat drawConvexHull(const cv::Mat &image, int minSize, int maxSize);

    //function to draw centre of moments
    cv::Mat drawMoments(const cv::Mat &image, int minSize, int maxSize);
};

#endif // LINESANDCONTOURS_H

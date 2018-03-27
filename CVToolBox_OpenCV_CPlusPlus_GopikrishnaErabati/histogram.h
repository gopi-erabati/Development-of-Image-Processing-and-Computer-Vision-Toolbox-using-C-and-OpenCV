#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>

class Histogram
{
private:

  int histSize[1];
  float hranges[2];
  const float* ranges[1];
  int channels[1];
  cv::MatND hist;
  cv::Mat histImg;
  double maxVal;
  double minVal;

public:
    Histogram();

    // Computes the 1D histogram.
    cv::MatND getHistogram(const cv::Mat &image);

    // Computes the 1D histogram and returns an image of it.
    cv::Mat getHistogramImage(const cv::Mat &image);

    // Equalizes the source image.
    cv::Mat equalize(const cv::Mat &image);

    // Stretches the source image.
    cv::Mat stretch(const cv::Mat &image, int minValue=0);

    // Applies a lookup table transforming an input image into a 1-channel image
    cv::Mat applyLookUp(const cv::Mat& image, const cv::MatND& lookup);


};

#endif // HISTOGRAM_H

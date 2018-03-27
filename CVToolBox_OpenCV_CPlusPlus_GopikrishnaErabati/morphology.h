#ifndef MORPHOLOGY_H
#define MORPHOLOGY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Morphology
{
private:
    cv::Mat image_morph;

public:
    Morphology();

    //function to get a structuring element
    cv::Mat getstrele(int type, int str_size = 1);

    // function to get morphology
    cv::Mat getMorph(cv::Mat image,int type, cv::Mat strele, int str_size, int morph_iter);
};

#endif // MORPHOLOGY_H

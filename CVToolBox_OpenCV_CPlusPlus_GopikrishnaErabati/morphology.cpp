#include "morphology.h"

Morphology::Morphology()
{

}

cv::Mat Morphology::getstrele(int type, int str_size){
    switch(type){
    case 1 :
        return cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*str_size + 1, 2*str_size+1 ), cv::Point(str_size, str_size) );
        break;
    case 2:
        return cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size( 2*str_size + 1, 2*str_size+1 ), cv::Point(str_size, str_size) );
        break;
    case 3:
        return cv::getStructuringElement(cv::MORPH_CROSS,cv::Size( 2*str_size + 1, 2*str_size+1 ), cv::Point(str_size, str_size) );

    }

}

cv::Mat Morphology::getMorph(cv::Mat image, int type, cv::Mat strele, int str_size, int morph_iter){
    switch(type){
    case 1:
        cv::dilate(image, image_morph, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 2:
        cv::erode(image, image_morph, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 3:
        cv::morphologyEx(image, image_morph, cv::MORPH_OPEN, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 4:
        cv::morphologyEx(image, image_morph, cv::MORPH_CLOSE, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 5:
        cv::morphologyEx(image, image_morph, cv::MORPH_GRADIENT, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 6:
        cv::morphologyEx(image, image_morph, cv::MORPH_TOPHAT, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 7:
        cv::morphologyEx(image, image_morph, cv::MORPH_BLACKHAT, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;
        break;
    case 8:
        cv::morphologyEx(image, image_morph, cv::MORPH_HITMISS, strele, cv::Point(str_size, str_size), morph_iter);
        return image_morph;

    }

}

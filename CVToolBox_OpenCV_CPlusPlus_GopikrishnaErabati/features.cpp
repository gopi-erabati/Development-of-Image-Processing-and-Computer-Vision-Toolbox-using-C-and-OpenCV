#include "features.h"

Features::Features()
{

}

// function to get FAST features
cv::Mat Features::getFastFeatures(cv::Mat image, int thresholdFast){

    imageFeatures = image.clone(); // copy image to image features

    // fastfeature detector create
    fastDetector = cv::FastFeatureDetector::create(thresholdFast);

    // to detect key points
    fastDetector->detect(imageFeatures, keypoints);

    //to draw keypoints
    cv::drawKeypoints(imageFeatures, keypoints, imageFeatures, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    return imageFeatures;
}


//function to get SURF features
cv::Mat Features::getSurfFeatures(cv::Mat image, double thresholdSurf){

    imageFeatures = image.clone(); // copy image to image features

    // SURF detector create
    surfDetector = cv::xfeatures2d::SURF::create(thresholdSurf);

    // to detcect key points
    surfDetector->detect(imageFeatures, keypoints);

    // to draw keypoints
    cv::drawKeypoints(imageFeatures, keypoints, imageFeatures, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    return imageFeatures;
}

//function to get SIFT features
cv::Mat Features::getSiftFeatures(cv::Mat image, double contrastThreshold, double edgeThreshold){

    imageFeatures = image.clone();

    //SIFT detector create
    siftDetector = cv::xfeatures2d::SIFT::create(0, 3, contrastThreshold, edgeThreshold);

    // to detect keypoints
    siftDetector->detect(imageFeatures, keypoints);

    // to draw keypoints
    cv::drawKeypoints(imageFeatures, keypoints, imageFeatures, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    return imageFeatures;
}

//function to get matched features bewteen two images
cv::Mat Features::getMatchFeatures(cv::Mat image, cv::Mat image2, double thresholdSurfMatches, int firstNMatches){

    //compute keypoints for image
    surfDetector = cv::xfeatures2d::SURF::create(thresholdSurfMatches);
    surfDetector->detect(image, keypoints);

    //compute keypoints for image2
    surfDetector->detect(image2, keypoints2);

    //compute descriptors for firsta and second images
    surfDetector->compute(image, keypoints, descriptors);
    surfDetector->compute(image2, keypoints2, descriptors2);

    //detect matches
    flannBasedMatcher.match(descriptors, descriptors2, matches);

    // deleter matches after first 'n' matches
    std::nth_element(matches.begin(), matches.begin() + firstNMatches - 1, matches.end());
    matches.erase(matches.begin() + firstNMatches, matches.end());

    //to draw matches
    cv::drawMatches(image, keypoints, image2, keypoints2, matches, imageFeatures, cv::Scalar(255, 255, 255));


    return imageFeatures;

}

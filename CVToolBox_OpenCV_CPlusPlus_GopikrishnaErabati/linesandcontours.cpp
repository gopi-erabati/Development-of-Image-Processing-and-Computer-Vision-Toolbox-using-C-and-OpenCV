#include "linesandcontours.h"

LinesAndContours::LinesAndContours()
{

}

//function to detect edges using canny edge detector
cv::Mat LinesAndContours::drawEdgeCanny(cv::Mat image, double lowThreshold, double highThreshold, int kernalSize){
    cv::Canny(image, edges, lowThreshold, highThreshold, kernalSize  );
    cv::threshold(edges, edges, 128, 255, cv::THRESH_BINARY_INV); // to changes edges to black and background as white
    return edges;
}

//function to get lines using hough transform
cv::Mat LinesAndContours::drawLinesHough(cv::Mat image, double lowThreshold, double highThreshold, double distStepSize, double angleStepSize, int votes, double minLength, double maxGap){

    // detect edges using canny
    cv::Canny(image, edges, lowThreshold, highThreshold);

    angleStepSize = angleStepSize * PI/180;

    // find lines
    cv::HoughLinesP(edges, lines, distStepSize, angleStepSize, votes, minLength, maxGap );

    // draw lines on image
    it_lines = lines.begin();
    while(it_lines != lines.end()){
        cv::Point point1((*it_lines)[0], (*it_lines)[1]);
        cv::Point point2((*it_lines)[2], (*it_lines)[3]);

        // draw line with above points
        cv::line(image, point1, point2, cv::Scalar(255, 255, 255));

        //increment iterator
        ++it_lines;
    }

    return image;
}

//function to draw circles using hough transform
cv::Mat LinesAndContours::drawCirclesHough(cv::Mat image, double accumulatorResolution, double minDist, double highThreshold, double minVotes, int minRadius, int maxRadius){

    imageBlur = image.clone(); // make a copy of image

    // perform gaussian blur
    cv::GaussianBlur(imageBlur, imageBlur, cv::Size(5, 5), 1.5);

    // to detect circles using hough transform
    cv::HoughCircles(imageBlur, circles, CV_HOUGH_GRADIENT, accumulatorResolution, minDist, highThreshold, minVotes, minRadius, maxRadius );

    // draw circles on image
    it_circles = circles.begin();
    while(it_circles != circles.end()){
        cv::circle(imageBlur, cv::Point((*it_circles)[0], (*it_circles)[1]), (*it_circles)[2], cv::Scalar(255), 2);
        ++it_circles;
    }

    return imageBlur;

}

//function to get contours
std::vector<std::vector<cv::Point>> LinesAndContours::getContours(cv::Mat image, int minSize, int maxSize){
    //to find contours
    cv::findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // eliminate too short and too long contours
    it_contours = contours.begin();
    while(it_contours != contours.end()){
        if( it_contours->size() < minSize || it_contours->size() > maxSize)
            it_contours = contours.erase(it_contours);
        else
            ++it_contours;
    }

    return contours;

}

// function to draw contours
cv::Mat LinesAndContours::myDrawContours(cv::Mat image, int minSize, int maxSize){

    // a white image
    cv::Mat imageContours(image.rows, image.cols, CV_8U, cv::Scalar(255));

    //draw contours on image
    cv::drawContours(imageContours, getContours(image, minSize, maxSize) , -1, cv::Scalar(0), 2);

    return imageContours;
}

// function to draw bounding box
cv::Mat LinesAndContours::drawBoundingBox(const cv::Mat &image, int minSize, int maxSize){


    // get image contours to draw bounding box
    imageContours = myDrawContours(image, minSize, maxSize);

    //draw bounding box for contours
    it_contours = contours.begin();
    while(it_contours != contours.end()){
        rectangleBox = cv::boundingRect(cv::Mat(*it_contours++));
        cv::rectangle(imageContours, rectangleBox, cv::Scalar(0), 2);
    }

    return imageContours;
}

//function to draw minimum enclsoing circle
cv::Mat LinesAndContours::drawMinEnclosingCircle(const cv::Mat &image, int minSize, int maxSize){

    // get image contours to draw bounding box
    imageContours = myDrawContours(image, minSize, maxSize);

    //draw min enclsoing circle for contours
    it_contours = contours.begin();
    while(it_contours != contours.end()){
        cv::minEnclosingCircle(cv::Mat(*it_contours++), center, radius);
        cv::circle(imageContours, cv::Point(center), static_cast<int>(radius), cv::Scalar(0), 2);
    }

    return imageContours;
}

//function to draw polygon approximation
cv::Mat LinesAndContours::drawPolygonApprox(const cv::Mat &image, int minSize, int maxSize){

    // get image contours to draw bounding box
    imageContours = myDrawContours(image, minSize, maxSize);

    //drawpoly approax for contours
//    it_contours = contours.begin();
//    while(it_contours != contours.end()){
//        //poly approax
//        cv::approxPolyDP(cv::Mat(*it_contours++), polygon, 3, true);

//        it_polygon = polygon.begin();
//        while(it_polygon != (polygon.end()-1)){
//            cv::line(imageContours, *it_polygon, *(it_polygon+1), cv::Scalar(0), 2);
//        }
//        // to connect fisrta nd last points
//        cv::line(imageContours, *(polygon.begin()), *(polygon.end()-1), cv::Scalar(0), 2);
//        qDebug() << "out of second while" ;
//    }
    std::vector<std::vector<cv::Point>> polygon(contours.size());

    for(int i = 0; i < contours.size(); i++){
        cv::approxPolyDP(cv::Mat(contours[i]), polygon[i], 20, true);
        cv::drawContours(imageContours, polygon, i, cv::Scalar(255), 2);
    }
    return imageContours;
}

//function to draw convex hull
cv::Mat LinesAndContours::drawConvexHull(const cv::Mat &image, int minSize, int maxSize){

    // get image contours to draw bounding box
    imageContours = myDrawContours(image, minSize, maxSize);

    //draw min enclsoing circle for contours
//    it_contours = contours.begin();
//    while(it_contours != contours.end()){
//        //poly approax
//        cv::convexHull(cv::Mat(*it_contours++), hull);

//        it_hull = hull.begin();
//        while(it_hull != (hull.end() - 1)){
//            cv::line(imageContours, *it_hull, *(it_hull + 1), cv::Scalar(0), 2);
//        }
//        // to connect fisrta nd last points
//        cv::line(imageContours, *(hull.begin()), *(hull.end() - 1), cv::Scalar(0), 2);
//    }

    std::vector<std::vector<cv::Point>> polygon(contours.size());

    for(int i = 0; i < contours.size(); i++){
        cv::convexHull(cv::Mat(contours[i]), polygon[i]);
        cv::drawContours(imageContours, polygon, i, cv::Scalar(255), 2);
    }

    return imageContours;
}

//function to draw centre of moments
cv::Mat LinesAndContours::drawMoments(const cv::Mat &image, int minSize, int maxSize){

    // get image contours to draw bounding box
    imageContours = myDrawContours(image, minSize, maxSize);

    it_contours = contours.begin();
    while(it_contours != contours.end()){
        // compute moments
        moments = cv::moments(cv::Mat(*it_contours++));

        // draw center
        cv::circle(imageContours, cv::Point(moments.m10/moments.m00, moments.m01/moments.m00), 2, cv::Scalar(0), 2);
    }

    return imageContours;
}

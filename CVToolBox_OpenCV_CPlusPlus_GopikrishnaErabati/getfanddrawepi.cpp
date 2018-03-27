#include "getfanddrawepi.h"

GetFAndDrawEpi::GetFAndDrawEpi()
{

}

// function to get fundamental matrix from 7 point algo
cv::Mat GetFAndDrawEpi::getFundamental7PointAndDrawEpilines(cv::Mat image, cv::Mat image2){

    //compute keypoints for image
    surfDetector = cv::xfeatures2d::SURF::create(2500);
    surfDetector->detect(image, keypoints1);

    //compute keypoints for image2
    surfDetector->detect(image2, keypoints2);

    //compute descriptors for firsta and second images
    surfDetector->compute(image, keypoints1, descriptors);
    surfDetector->compute(image2, keypoints2, descriptors2);

    //detect matches
    flannBasedMatcher.match(descriptors, descriptors2, matches);

    /* between church01 and church03 */
    selMatches.push_back(matches[14]);
    selMatches.push_back(matches[16]);
    selMatches.push_back(matches[141]);
    selMatches.push_back(matches[146]);
    selMatches.push_back(matches[235]);
    selMatches.push_back(matches[238]);
    selMatches.push_back(matches[274]);

    //to draw matches
    cv::drawMatches(image, keypoints1, image2, keypoints2, selMatches, imageFeatures, cv::Scalar(255, 255, 255));

    //show matches
    cv::namedWindow("Matches");
    cv::imshow("Matches", imageFeatures);

    // Convert 1 vector of keypoints into
    // 2 vectors of Point2f
    for (std::vector<cv::DMatch>::const_iterator it= selMatches.begin();
         it!= selMatches.end(); ++it) {
             // Get the indexes of the selected matched keypoints
             pointIndexes1.push_back(it->queryIdx);
             pointIndexes2.push_back(it->trainIdx);
    }

    // Convert keypoints into Point2f
    cv::KeyPoint::convert(keypoints1,selPoints1,pointIndexes1);
    cv::KeyPoint::convert(keypoints2,selPoints2,pointIndexes2);

    // check by drawing the points
    std::vector<cv::Point2f>::const_iterator it= selPoints1.begin();
    while (it!=selPoints1.end()) {

        // draw a circle at each corner location
        cv::circle(image,*it,3,cv::Scalar(255,255,255),2);
        ++it;
    }

    it= selPoints2.begin();
    while (it!=selPoints2.end()) {

        // draw a circle at each corner location
        cv::circle(image2,*it,3,cv::Scalar(255,255,255),2);
        ++it;
    }

    // Compute F matrix from 7 matches
    cv::Mat fundamental= cv::findFundamentalMat(
        cv::Mat(selPoints1), // points in first image
        cv::Mat(selPoints2), // points in second image
        CV_FM_7POINT);       // 7-point method


    // draw the left points corresponding epipolar lines in right image
    cv::computeCorrespondEpilines(
        cv::Mat(selPoints1), // image points
        1,                   // in image 1 (can also be 2)
        fundamental, // F matrix
        lines1);     // vector of epipolar lines

    // for all epipolar lines
    for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin();
         it!=lines1.end(); ++it) {

             // draw the epipolar line between first and last column
             cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }

    // draw the left points corresponding epipolar lines in left image
    cv::computeCorrespondEpilines(cv::Mat(selPoints2),2,fundamental,lines2);
    for (std::vector<cv::Vec3f>::const_iterator it= lines2.begin();
         it!=lines2.end(); ++it) {

             // draw the epipolar line between first and last column
             cv::line(image,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image.cols,-((*it)[2]+(*it)[0]*image.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }
    qDebug() << "after epilines";
    // Display the images with points and epipolar lines
    cv::namedWindow("Right Image Epilines");
    cv::imshow("Right Image Epilines",image);
    cv::namedWindow("Left Image Epilines");
    cv::imshow("Left Image Epilines",image2);

    return fundamental;
}

// function to get fundamental matrix from 7 point algo
cv::Mat GetFAndDrawEpi::getFundamental8PointAndDrawEpilines(cv::Mat image, cv::Mat image2){

    //compute keypoints for image
    surfDetector = cv::xfeatures2d::SURF::create(2500);
    surfDetector->detect(image, keypoints1);

    //compute keypoints for image2
    surfDetector->detect(image2, keypoints2);

    //compute descriptors for firsta and second images
    surfDetector->compute(image, keypoints1, descriptors);
    surfDetector->compute(image2, keypoints2, descriptors2);

    //detect matches
    flannBasedMatcher.match(descriptors, descriptors2, matches);

    /* between church01 and church03 */
    selMatches.push_back(matches[14]);
    selMatches.push_back(matches[16]);
    selMatches.push_back(matches[141]);
    selMatches.push_back(matches[146]);
    selMatches.push_back(matches[235]);
    selMatches.push_back(matches[238]);
    selMatches.push_back(matches[274]);
    selMatches.push_back(matches[60]);

    //to draw matches
    cv::drawMatches(image, keypoints1, image2, keypoints2, selMatches, imageFeatures, cv::Scalar(255, 255, 255));

    cv::namedWindow("Matches");
    cv::imshow("Matches",imageFeatures);

    // Convert 1 vector of keypoints into
    // 2 vectors of Point2f
    for (std::vector<cv::DMatch>::const_iterator it= selMatches.begin();
         it!= selMatches.end(); ++it) {

             // Get the indexes of the selected matched keypoints
             pointIndexes1.push_back(it->queryIdx);
             pointIndexes2.push_back(it->trainIdx);
    }

    // Convert keypoints into Point2f
    cv::KeyPoint::convert(keypoints1,selPoints1,pointIndexes1);
    cv::KeyPoint::convert(keypoints2,selPoints2,pointIndexes2);

    // check by drawing the points
    std::vector<cv::Point2f>::const_iterator it= selPoints1.begin();
    while (it!=selPoints1.end()) {

        // draw a circle at each corner location
        cv::circle(image,*it,3,cv::Scalar(255,255,255),2);
        ++it;
    }

    it= selPoints2.begin();
    while (it!=selPoints2.end()) {

        // draw a circle at each corner location
        cv::circle(image2,*it,3,cv::Scalar(255,255,255),2);
        ++it;
    }

    // Compute F matrix from 7 matches
    cv::Mat fundamental= cv::findFundamentalMat(
        cv::Mat(selPoints1), // points in first image
        cv::Mat(selPoints2), // points in second image
        CV_FM_8POINT);       // 7-point method


    // draw the left points corresponding epipolar lines in right image
    cv::computeCorrespondEpilines(
        cv::Mat(selPoints1), // image points
        1,                   // in image 1 (can also be 2)
        fundamental, // F matrix
        lines1);     // vector of epipolar lines

    // for all epipolar lines
    for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin();
         it!=lines1.end(); ++it) {

             // draw the epipolar line between first and last column
             cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }

    // draw the left points corresponding epipolar lines in left image
    cv::computeCorrespondEpilines(cv::Mat(selPoints2),2,fundamental,lines2);
    for (std::vector<cv::Vec3f>::const_iterator it= lines2.begin();
         it!=lines2.end(); ++it) {

             // draw the epipolar line between first and last column
             cv::line(image,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image.cols,-((*it)[2]+(*it)[0]*image.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }

    // Display the images with points and epipolar lines
    cv::namedWindow("Right Image Epilines");
    cv::imshow("Right Image Epilines",image);
    cv::namedWindow("Left Image Epilines");
    cv::imshow("Left Image Epilines",image2);

    return fundamental;
}

// function to get fundamental matrix from 7 point algo
cv::Mat GetFAndDrawEpi::getFundamentalRANSACAndDrawEpilines(cv::Mat image, cv::Mat image2){

    //compute keypoints for image
    surfDetector = cv::xfeatures2d::SURF::create(2500);
    surfDetector->detect(image, keypoints1);

    //compute keypoints for image2
    surfDetector->detect(image2, keypoints2);

    //compute descriptors for firsta and second images
    surfDetector->compute(image, keypoints1, descriptors);
    surfDetector->compute(image2, keypoints2, descriptors2);

    //detect matches
    flannBasedMatcher.match(descriptors, descriptors2, matches);

    //to draw matches
    cv::drawMatches(image, keypoints1, image2, keypoints2, matches, imageFeatures, cv::Scalar(255, 255, 255));

    cv::namedWindow("Matches");
    cv::imshow("Matches",imageFeatures);


    // Convert keypoints into Point2f
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
         it!= matches.end(); ++it) {

             // Get the position of left keypoints
             float x= keypoints1[it->queryIdx].pt.x;
             float y= keypoints1[it->queryIdx].pt.y;
             points1.push_back(cv::Point2f(x,y));
             // Get the position of right keypoints
             x= keypoints2[it->trainIdx].pt.x;
             y= keypoints2[it->trainIdx].pt.y;
             points2.push_back(cv::Point2f(x,y));
    }

    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(),0);
    fundamental= cv::findFundamentalMat(
        cv::Mat(points1),cv::Mat(points2), // matching points
        inliers,      // match status (inlier ou outlier)
        CV_FM_RANSAC, // RANSAC method
        1,            // distance to epipolar line
        0.98);        // confidence probability

    //get selected matches
    selMatches.push_back(matches[14]);
    selMatches.push_back(matches[16]);
    selMatches.push_back(matches[141]);
    selMatches.push_back(matches[146]);
    selMatches.push_back(matches[235]);
    selMatches.push_back(matches[238]);
    selMatches.push_back(matches[274]);

    //get selected points
    for (std::vector<cv::DMatch>::const_iterator it= selMatches.begin();
         it!= selMatches.end(); ++it) {

             // Get the indexes of the selected matched keypoints
             pointIndexes1.push_back(it->queryIdx);
             pointIndexes2.push_back(it->trainIdx);
    }

    // Convert keypoints into Point2f
    std::vector<cv::Point2f> selPoints1, selPoints2;
    cv::KeyPoint::convert(keypoints1,selPoints1,pointIndexes1);
    cv::KeyPoint::convert(keypoints2,selPoints2,pointIndexes2);

//	// check by drawing the points
//	std::vector<cv::Point2f>::const_iterator it= selPoints1.begin();
//	while (it!=selPoints1.end()) {

//		// draw a circle at each corner location
//		cv::circle(image,*it,3,cv::Scalar(255,255,255),2);
//		++it;
//	}

//	it= selPoints2.begin();
//	while (it!=selPoints2.end()) {

//		// draw a circle at each corner location
//		cv::circle(image2,*it,3,cv::Scalar(255,255,255),2);
//		++it;
//	}


    // Draw the epipolar line of few points
    cv::computeCorrespondEpilines(cv::Mat(selPoints1),1,fundamental,lines1);
    for (std::vector<cv::Vec3f>::const_iterator it= lines1.begin();
         it!=lines1.end(); ++it) {

             cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }

    cv::computeCorrespondEpilines(cv::Mat(selPoints2),2,fundamental,lines2);
    for (std::vector<cv::Vec3f>::const_iterator it= lines2.begin();
         it!=lines2.end(); ++it) {

             cv::line(image,cv::Point(0,-(*it)[2]/(*it)[1]),
                             cv::Point(image.cols,-((*it)[2]+(*it)[0]*image.cols)/(*it)[1]),
                             cv::Scalar(255,255,255));
    }

    // Draw the inlier points
    std::vector<cv::Point2f>::const_iterator itPts= points1.begin();
    std::vector<uchar>::const_iterator itIn= inliers.begin();
    while (itPts!=points1.end()) {

        // draw a circle at each inlier location
        if (*itIn) {
            cv::circle(image,*itPts,3,cv::Scalar(255,255,255),2);
            points1In.push_back(*itPts);
        }
        ++itPts;
        ++itIn;
    }

    itPts= points2.begin();
    itIn= inliers.begin();
    while (itPts!=points2.end()) {

        // draw a circle at each inlier location
        if (*itIn) {
            cv::circle(image2,*itPts,3,cv::Scalar(255,255,255),2);
            points2In.push_back(*itPts);
        }
        ++itPts;
        ++itIn;
    }

    // Display the images with points
    cv::namedWindow("Right Image Epilines (RANSAC)");
    cv::imshow("Right Image Epilines (RANSAC)",image);
    cv::namedWindow("Left Image Epilines (RANSAC)");
    cv::imshow("Left Image Epilines (RANSAC)",image2);

    return fundamental;

}

//function to get homography
cv::Mat GetFAndDrawEpi::getHomography(cv::Mat image1, cv::Mat image2){


    robustMatcher.setConfidenceLevel(0.98);
    robustMatcher.setMinDistanceToEpipolar(1.0);
    robustMatcher.setRatio(0.65f);

    // Match the two images
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat fundemental= robustMatcher.match(image1,image2,matches, keypoints1, keypoints2);

    // draw the matches

    cv::drawMatches(image1,keypoints1,  // 1st image and its keypoints
                    image2,keypoints2,  // 2nd image and its keypoints
                    matches,			// the matches
                    imageMatches,		// the image produced
                    cv::Scalar(255,255,255)); // color of the lines
    cv::namedWindow("Matches");
    cv::imshow("Matches",imageMatches);


    // Convert keypoints into Point2f
    for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
         it!= matches.end(); ++it) {

             // Get the position of left keypoints
             float x= keypoints1[it->queryIdx].pt.x;
             float y= keypoints1[it->queryIdx].pt.y;
             points1.push_back(cv::Point2f(x,y));
             // Get the position of right keypoints
             x= keypoints2[it->trainIdx].pt.x;
             y= keypoints2[it->trainIdx].pt.y;
             points2.push_back(cv::Point2f(x,y));
    }

    // Find the homography between image 1 and image 2
    std::vector<uchar> inliers(points1.size(),0);
    cv::Mat homography= cv::findHomography(
        cv::Mat(points1),cv::Mat(points2), // corresponding points
        inliers,	// outputed inliers matches
        CV_RANSAC,	// RANSAC method
        1.);	    // max distance to reprojection point

    // Draw the inlier points
    std::vector<cv::Point2f>::const_iterator itPts= points1.begin();
    std::vector<uchar>::const_iterator itIn= inliers.begin();
    while (itPts!=points1.end()) {

        // draw a circle at each inlier location
        if (*itIn)
            cv::circle(image1,*itPts,3,cv::Scalar(255,255,255),2);

        ++itPts;
        ++itIn;
    }

    itPts= points2.begin();
    itIn= inliers.begin();
    while (itPts!=points2.end()) {

        // draw a circle at each inlier location
        if (*itIn)
            cv::circle(image2,*itPts,3,cv::Scalar(255,255,255),2);

        ++itPts;
        ++itIn;
    }

    // Display the images with points
    cv::namedWindow("Image 1 Homography Points");
    cv::imshow("Image 1 Homography Points",image1);
    cv::namedWindow("Image 2 Homography Points");
    cv::imshow("Image 2 Homography Points",image2);

    // Warp image 1 to image 2
    cv::Mat result;
    cv::warpPerspective(image1, // input image
        result,			// output image
        homography,		// homography
        cv::Size(2*image1.cols,image1.rows)); // size of output image

    // Copy image 1 on the first half of full image
    cv::Mat half(result,cv::Rect(0,0,image2.cols,image2.rows));
    image2.copyTo(half);

    // Display the warp image
    cv::namedWindow("After warping");
    cv::imshow("After warping",result);


    return homography;
}

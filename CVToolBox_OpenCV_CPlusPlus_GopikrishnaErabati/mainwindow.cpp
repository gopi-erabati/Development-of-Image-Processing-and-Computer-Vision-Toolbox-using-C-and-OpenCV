#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "inoutsave.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    current_index_main_tab = 0;
    current_index_line_tab = 0;
    current_index_filter_tab = 0;
    current_index_feature_tab = 0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

// to getui pointer from other class
Ui::MainWindow* MainWindow::getui(){
    return ui;
}

void MainWindow::on_loadImage_push_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.bmp)"));

    image_inp= cv::imread(fileName.toStdString());
    cv::cvtColor(image_inp,image_inp,CV_BGR2RGB);  // change color channel ordering
    cv::cvtColor(image_inp,image_inp_gray,CV_RGB2GRAY); // to change input image to grayscale


    if (image_inp.data) {

        ui->process_push->setEnabled(true);
    }

    showInput(image_inp_gray);
//    imageView = QImage((const unsigned char*)(image_inp_gray.data),  // Qt image structure
//                       image_inp_gray.cols,image_inp_gray.rows,image_inp_gray.step,QImage::Format_Indexed8);

//    // to know the label width and height to scale the image
//    int width = ui->input_lab->width();
//    int height = ui->input_lab->height();


//    // convert QImage to QPixmap and show on QLabel
//    ui->input_lab->setPixmap(QPixmap::fromImage(imageView).scaled(width,height,Qt::KeepAspectRatio));

}

//to show input image
void MainWindow::showInput(cv::Mat image_inp_gray){

    imageView = QImage((const unsigned char*)(image_inp_gray.data),  // Qt image structure
                       image_inp_gray.cols,image_inp_gray.rows,image_inp_gray.step,QImage::Format_Indexed8);

    // to know the label width and height to scale the image
    int width = ui->input_lab->width();
    int height = ui->input_lab->height();


    // convert QImage to QPixmap and show on QLabel
    ui->input_lab->setPixmap(QPixmap::fromImage(imageView).scaled(width,height,Qt::KeepAspectRatio));
}

void MainWindow::on_process_push_clicked()
{

    // General Tab
    if (current_index_main_tab == 0){

        // to add salt and pepper noise
        if (ui->salt_radio->isChecked() == true){
        image_op = salt(image_inp_gray, saltAndPepper_value);
        showOutput(image_op);
        }

        // to add logo
        else if (ui->logo_radio->isChecked() == true){
            cv::Mat image_input = image_inp_gray.clone();
            // define ROI
            cv::Mat imageROI= image_input(cv::Rect(image_inp_gray.cols-image_logo.cols,image_inp_gray.rows-image_logo.rows,image_logo.cols,image_logo.rows));
            image_logo.copyTo(imageROI,image_logo);
            showOutput(image_input);
        }

        //to change color spaces
        else {
            convert_color();
        }

    }

    //to do histograms
    else if(current_index_main_tab == 1){
        plotHistograms();

    }

    //to do morphology
    else if(current_index_main_tab == 2){
        getMorphology();

    }

    //to do filtering
    else if(current_index_main_tab == 3){

        getFilter();

    }

    // to detect lines, circles, contours
    else if(current_index_main_tab == 4) {
        getEdgesLinesContours();
    }

    //to get and match features
    else if(current_index_main_tab == 5){
        getFeaturesAndMatches();
    }

    // for calibration and Fundamnetal matrix and homography
    else if(current_index_main_tab == 6){
        getCalibrateAndParameters();
    }

    ui->images_radio->setEnabled(false);

}

cv::Mat MainWindow::salt(const cv::Mat image, int n) {
    cv::Mat image1 = image.clone();
    int i,j;
    int z = n/2;
    //add salt
    for (int k=0; k<z; k++) {

        // rand() is the MFC random number generator
        i= rand()%image1.cols;
        j= rand()%image1.rows;


        if (image1.channels() == 1) { // gray-level image

            image1.at<uchar>(j,i)= 255;

        } else if (image1.channels() == 3) { // color image

            image1.at<cv::Vec3b>(j,i)[0]= 255;
            image1.at<cv::Vec3b>(j,i)[1]= 255;
            image1.at<cv::Vec3b>(j,i)[2]= 255;
        }
    }
    //add pepper
    for (int k=0; k<z; k++) {

        // rand() is the MFC random number generator
        i= rand()%image1.cols;
        j= rand()%image1.rows;


        if (image1.channels() == 1) { // gray-level image

            image1.at<uchar>(j,i)= 0;

        } else if (image1.channels() == 3) { // color image

            image1.at<cv::Vec3b>(j,i)[0]= 0;
            image1.at<cv::Vec3b>(j,i)[1]= 0;
            image1.at<cv::Vec3b>(j,i)[2]= 0;
        }
    }
    return image1;
}

// function to convert color spaces
void MainWindow::convert_color(){
    if(ui->graytorgb_radio->isChecked() == true){
        cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
        showOutput(image_op);
    }
    else if(ui->graytociexyz_radio->isChecked() == true){
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2XYZ);
       showOutput(image_op);
    }
    else if(ui->graytociexyz_radio->isChecked() == true){
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2YCrCb);
       showOutput(image_op);
    }
    else if(ui->graytociexyz_radio->isChecked() == true){
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2HSV);
       showOutput(image_op);
    }
    else if(ui->graytociexyz_radio->isChecked() == true){
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2HLS);
       showOutput(image_op);
    }
    else if(ui->graytociexyz_radio->isChecked() == true){
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2Lab);
       showOutput(image_op);
    }
    else{
       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
       cv::cvtColor(image_op, image_op, CV_RGB2Luv);
       showOutput(image_op);
    }
}

// function to plot histograms
void MainWindow::plotHistograms(){
    if(ui->plothistogram_radio->isChecked() == true){
        image_op = histogram.getHistogramImage(image_inp_gray);
        showOutput(image_op);
    }
    else if(ui->equalisehistogram_radio->isChecked() == true){
        image_op = histogram.equalize(image_inp_gray);
        showOutput(image_op);
        cv::namedWindow("Equalised Histogram");
        cv::imshow("Equalised Histogram", histogram.getHistogramImage(image_op));
    }
    else{

        image_op = histogram.stretch(image_inp_gray,minvalstretch_value );
        showOutput(image_op);
        cv::namedWindow("Stretched Histogram");
        cv::imshow("Stretched Histogram", histogram.getHistogramImage(image_op));
    }
}

//function to get morphological operations
void MainWindow::getMorphology(){

    // to get structuring element RECT, ELLIPSE, CROSS
    if(ui->strelerect_radio->isChecked() == true){
        strele = morphology.getstrele(1, ui->elesize_spin->value());
    }
    else if(ui->streleellip_radio->isChecked() == true){
        strele = morphology.getstrele(2, ui->elesize_spin->value());
    }
    else{
        strele = morphology.getstrele(3, ui->elesize_spin->value());
    }

    // to do morph operations DILATE, ERODE, OPEN, CLOSE ...
    if(ui->dilate_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 1, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->erode_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 2, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->open_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 3, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->close_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 4, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->gradient_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 5, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->tophat_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 6, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else if(ui->blackhat_radio->isChecked() == true){
        image_op = morphology.getMorph(image_inp_gray, 7, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
    else{
        image_op = morphology.getMorph(image_inp_gray, 8, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
        showOutput(image_op);
    }
}

//to get filtered images
void MainWindow::getFilter(){
    // to get lowpass filter images
    if(current_index_filter_tab == 0){

        if(ui->box_radio->isChecked() == true){

            image_op = cfilter.getbox(image_inp_gray, ui->kersize_spin->value());
            showOutput(image_op);
        }
        else if(ui->gaussian_radio->isChecked() == true){
            image_op = cfilter.getGaussianBlurr(image_inp_gray, ui->kersize_spin->value(), ui->alongx_spin->value(), ui->alongy_spin->value());
            showOutput(image_op);
        }
        else if(ui->median_radio->isChecked() == true){
            image_op = cfilter.getMedianBlurr(image_inp_gray, ui->kersize_spin->value());
            showOutput(image_op);
        }
        else if(ui->bilateral_radio->isChecked() == true){

            image_op = cfilter.getBilateralBlurr(image_inp_gray, ui->kersize_spin->value(), ui->sdcolor_spin->value(), ui->sdspatial_spin->value());
            showOutput(image_op);
        }
    }
    else{
        //to do highpass filter images (sobel,scharr,laplacian, detect edges,...)
        if(ui->sobel_radio->isChecked() == true){
            image_op = cfilter.getSobel(image_inp_gray, ui->kerSize_spin->value(), ui->deralongx_spin->value(), ui->deralongy_spin->value());
            showOutput(image_op);
        }
        else if(ui->scharr_radio->isChecked() == true){
            image_op = cfilter.getScharr(image_inp_gray, ui->deralongx_spin->value(), ui->deralongy_spin->value());
            showOutput(image_op);
        }
        else if(ui->laplacian_radio->isChecked() == true){
            image_op = cfilter.getLaplacian(image_inp_gray, ui->kersizelap_spin->value());
            showOutput(image_op);
        }
        else if(ui->edgeSobel_radio->isChecked() == true){
            image_op = cfilter.getEdgesSobel(image_inp_gray, ui->threshEdge_spin->value());
            showOutput(image_op);
        }
        else{
            image_op = laplacianedge.computeLaplacian(image_inp_gray, ui->kersizelap_spin->value());
            double lapmin, lapmax;
            cv::minMaxLoc(image_op,&lapmin,&lapmax);
            image_op = laplacianedge.getLaplacianImage();
            image_op = laplacianedge.getZeroCrossings(lapmax);
            showOutput(image_op);
        }
    }
}

void MainWindow::getEdgesLinesContours(){
    qDebug() << current_index_line_tab << "in function";
    // to compute edges using canny edge detector
    if(current_index_line_tab == 0){
        image_op = linesAndContours.drawEdgeCanny(image_inp_gray, ui->lowThreshold_spin->value(), ui->highThreshold_spin->value(), ui->kernalSize_spin->value());
        showOutput(image_op);
    }
    //to draw lines and circles using hough transform
    else if(current_index_line_tab == 1){
        if(ui->lineAct_radio->isChecked() == true){
            qDebug() << "in linehough";
           image_op = linesAndContours.drawLinesHough(image_inp_gray, ui->lowThresholdHough_spin->value(), ui->highThresholdHough_spin->value(), ui->distStepSize_spin->value(), ui->angleStepSize_spin->value(), ui->votes_spin->value(), ui->minLength_spin->value(), ui->maxGap_spin->value());
           showOutput(image_op);
        }
        else{
            image_op = linesAndContours.drawCirclesHough(image_inp_gray, ui->accumulatorResolution_spin->value(), ui->minDistBetCircles_spin->value(), ui->highThresholdCircles_spin->value(), ui->minVotes_spin->value(), ui->minRadius_spin->value(), ui->maxRadius_spin->value());
            showOutput(image_op);
        }
    }
    // to draw contours
    else if(current_index_line_tab == 2){
        image_op = linesAndContours.myDrawContours(image_inp_gray, ui->minContourLength_spin->value(), ui->maxContourLength_spin->value());
        showOutput(image_op);
    }
    // to draw bounding boxes, min enclosing circles, convexHull, polygon approx, moments
    else{
        if(ui->boundingBox_radio->isChecked() == true){
            image_op = linesAndContours.drawBoundingBox(image_inp_gray, ui->minContourLength_spin_2->value(), ui->maxContourLength_spin_2->value());
            showOutput(image_op);
        }
        else if(ui->circle_radio->isChecked() == true){
            image_op = linesAndContours.drawMinEnclosingCircle(image_inp_gray, ui->minContourLength_spin_2->value(), ui->maxContourLength_spin_2->value());
            showOutput(image_op);
        }
        else if(ui->polygon_radio->isChecked() == true){
            image_op = linesAndContours.drawPolygonApprox(image_inp_gray, ui->minContourLength_spin_2->value(), ui->maxContourLength_spin_2->value());
            showOutput(image_op);
        }
        else if(ui->convexHull_radio->isChecked() == true){
            image_op = linesAndContours.drawConvexHull(image_inp_gray, ui->minContourLength_spin_2->value(), ui->maxContourLength_spin_2->value());
            showOutput(image_op);
        }
        else {
            image_op = linesAndContours.drawMoments(image_inp_gray, ui->minContourLength_spin_2->value(), ui->maxContourLength_spin_2->value());
            showOutput(image_op);
        }
    }
}

// function to get corners, features, surf, sift, fast
void MainWindow::getFeaturesAndMatches(){

    // to get harris corners
    if(current_index_feature_tab == 0){
        if(ui->harrisCornerAct_radio->isChecked() == true){
            image_op = harrisCorner.getHarrisCorners(image_inp_gray, ui->neighbourhoodSize_spin->value(), ui->kernalSizeHarris_spin->value(), ui->parameterK_spin->value(), ui->qualityLevel_spin->value(), ui->neighbourhoodSizeNonMax_spin->value());
            showOutput(image_op);
        }
        else{
            image_op = harrisCorner.getUniformCorners(image_inp_gray, ui->maxCorners_spin->value(), ui->qualityLevelFeature_spin->value(), ui->minDistance_spin->value());
            showOutput(image_op);
        }
    }
    else{
        if(ui->fast_radio->isChecked() == true){
            image_op = features.getFastFeatures(image_inp_gray, ui->thresholdFast_spin->value());
            showOutput(image_op);
        }
        else if(ui->surf_radio->isChecked() == true){
            image_op = features.getSurfFeatures(image_inp_gray, ui->thresholdSurf_spin->value());
            showOutput(image_op);
        }
        else if(ui->sift_radio->isChecked() == true) {
            image_op = features.getSiftFeatures(image_inp_gray, ui->contrastThresholdSift_spin->value(), ui->edgeThresholdSift_spin->value());
            showOutput(image_op);
        }
        else{
            image_op = features.getMatchFeatures(image_inp_gray, image_inp_gray_second, ui->thresholdSurfMatch_spin->value(), ui->displyFirstNMatches_spin->value());
            cv::namedWindow("Matches Found");
            cv::imshow("Matches Found", image_op);
        }
    }
}

// for calibration, Fundamental matrix, epipolar lines, hiomography
void MainWindow::getCalibrateAndParameters(){

    // for calibration and undistorted image
    if(ui->calibrate_radio->isChecked() == true ){


        // generate list of chessboard image filename
        for (int i=1; i<=20; i++) {

            std::stringstream str;
            str << "D:/mscv_sem2/VP/project_c/project/build-sample-Desktop_Qt_5_9_0_MinGW_32bit-Debug/images/chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";

            filelist.push_back(str.str());
            image= cv::imread(str.str(),0);
        }

        // add the corners from the chessboard
        cv::Size boardSize(6,4);
        cameraCalibration.addChessboardPoints(
            filelist,	// filenames of chessboard image
            boardSize);	// size of chessboard

        // calibrate the camera
        cameraCalibration.calibrate(image.size());

        // Image Undistortion
        image = cv::imread(filelist[9]);
        uImage = cameraCalibration.remap(image);

        // get camera matrix
        cameraMatrix= cameraCalibration.getCameraMatrix();


        // show the undistorted image on onput image panel
        imageView = QImage((const unsigned char*)(image.data),  // Qt image structure
                           image.cols,image.rows,image.step,QImage::Format_RGB888);
        // to know the label width and height to scale the image
        int width = ui->input_lab->width();
        int height = ui->input_lab->height();
        // convert QImage to QPixmap and show on QLabel
        ui->input_lab->setPixmap(QPixmap::fromImage(imageView).scaled(width,height,Qt::KeepAspectRatio));

        //show the undistorted image on output panel
        showOutput(uImage);

        std::string temp = " Camera intrinsic: ";
        QString qstr = QString::fromStdString(temp);
        ui->textBrowser->setText(qstr);

        std::stringstream str;
        str << std::endl << std::setw(12) << cameraMatrix.at<double>(0,0) << " " << std::setw(12) << cameraMatrix.at<double>(0,1) << " " << std::setw(12) << cameraMatrix.at<double>(0,2) << std::endl <<
        std::setw(12) << cameraMatrix.at<double>(1,0) << " " << std::setw(12) << cameraMatrix.at<double>(1,1) << " " << std::setw(12) << cameraMatrix.at<double>(1,2) << std::endl <<
        std::setw(12) << cameraMatrix.at<double>(2,0) << " " << std::setw(12) << cameraMatrix.at<double>(2,1) << " " << std::setw(12) << cameraMatrix.at<double>(2,2) << std::endl;

        QString qstrt = QString::fromStdString(str.str());
        ui->textBrowser->append(qstrt);
    }

    // for Fundamental matrix and epipolar lines
    else if (ui->calculateF_radio->isChecked() == true ){

        // by 7 POint method
        if (ui->point7_radio->isChecked() == true){
            cv::destroyAllWindows();
            fundamental = getFAndDrawEpi.getFundamental7PointAndDrawEpilines(image_inp_gray, image_inp_gray_second );

            //show fundamental matrix
            std::string temp = " Fundamental Matrix: ";
            QString qstr = QString::fromStdString(temp);
            ui->textBrowser->setText(qstr);

            std::stringstream str;
            str << std::endl << std::setw(16) << fundamental.at<double>(0,0) << " " << std::setw(12) << fundamental.at<double>(0,1) << " " << std::setw(16) << fundamental.at<double>(0,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(1,0) << " " << std::setw(16) << fundamental.at<double>(1,1) << " " << std::setw(16) << fundamental.at<double>(1,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(2,0) << " " << std::setw(16) << fundamental.at<double>(2,1) << " " << std::setw(16) << fundamental.at<double>(2,2) << std::endl;

            QString qstrt = QString::fromStdString(str.str());
            ui->textBrowser->append(qstrt);
        }

        //by 8 point method
        else if (ui->point8_radio->isChecked() == true){
            cv::destroyAllWindows();
            fundamental = getFAndDrawEpi.getFundamental8PointAndDrawEpilines(image_inp_gray, image_inp_gray_second );

            //show fundamental matrix
            std::string temp = " Fundamental Matrix: ";
            QString qstr = QString::fromStdString(temp);
            ui->textBrowser->setText(qstr);

            std::stringstream str;
            str << std::endl << std::setw(16) << fundamental.at<double>(0,0) << " " << std::setw(16) << fundamental.at<double>(0,1) << " " << std::setw(16) << fundamental.at<double>(0,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(1,0) << " " << std::setw(16) << fundamental.at<double>(1,1) << " " << std::setw(16) << fundamental.at<double>(1,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(2,0) << " " << std::setw(16) << fundamental.at<double>(2,1) << " " << std::setw(16) << fundamental.at<double>(2,2) << std::endl;

            QString qstrt = QString::fromStdString(str.str());
            ui->textBrowser->append(qstrt);
        }

        //by RANSAC
        else if (ui->ransac_radio->isChecked() == true){
            cv::destroyAllWindows();
            fundamental = getFAndDrawEpi.getFundamentalRANSACAndDrawEpilines(image_inp_gray, image_inp_gray_second );

            //show fundamental matrix
            std::string temp = " Fundamental Matrix: ";
            QString qstr = QString::fromStdString(temp);
            ui->textBrowser->setText(qstr);

            std::stringstream str;
            str << std::endl << std::setw(16) << fundamental.at<double>(0,0) << " " << std::setw(16) << fundamental.at<double>(0,1) << " " << std::setw(16) << fundamental.at<double>(0,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(1,0) << " " << std::setw(16) << fundamental.at<double>(1,1) << " " << std::setw(16) << fundamental.at<double>(1,2) << std::endl <<
            std::setw(16) << fundamental.at<double>(2,0) << " " << std::setw(16) << fundamental.at<double>(2,1) << " " << std::setw(16) << fundamental.at<double>(2,2) << std::endl;

            QString qstrt = QString::fromStdString(str.str());
            ui->textBrowser->append(qstrt);
        }
    }

    //for homography
    else{
        cv::destroyAllWindows();
        homography = getFAndDrawEpi.getHomography(image_inp_gray, image_inp_gray_second );

        //show homography matrix
        std::string temp = " Homography Matrix: ";
        QString qstr = QString::fromStdString(temp);
        ui->textBrowser->setText(qstr);

        std::stringstream str;
        str << std::endl << std::setw(16) << homography.at<double>(0,0) << " " << std::setw(16) << homography.at<double>(0,1) << " " << std::setw(16) << homography.at<double>(0,2) << std::endl <<
        std::setw(16) << homography.at<double>(1,0) << " " << std::setw(16) << homography.at<double>(1,1) << " " << std::setw(16) << homography.at<double>(1,2) << std::endl <<
        std::setw(16) << homography.at<double>(2,0) << " " << std::setw(16) << homography.at<double>(2,1) << " " << std::setw(16) << homography.at<double>(2,2) << std::endl;

        QString qstrt = QString::fromStdString(str.str());
        ui->textBrowser->append(qstrt);
    }


}

void MainWindow::showOutput(cv::Mat image){

    ui->output_lab->clear();

    if(image.channels() == 1){
        imageView = QImage((const unsigned char*)(image.data),  // Qt image structure
                           image.cols,image.rows,image.step,QImage::Format_Indexed8);
    }
    else{
        imageView = QImage((const unsigned char*)(image.data),  // Qt image structure
                           image.cols,image.rows,image.step,QImage::Format_RGB888);
    }


    // to know the label width and height to scale the image
    int width = ui->input_lab->width();
    int height = ui->input_lab->height();


    if(imageView.isNull()){
        qDebug() << "null";
    }


    // convert QImage to QPixmap and show on QLabel
    ui->output_lab->setPixmap(QPixmap::fromImage(imageView).scaled(width,height,Qt::KeepAspectRatio));
}


//TRIED MOUSE EVENT TO PLACE LOGO BUT FAILED IN RUN TIME (GETTING CRASHED)
void MainWindow::onMouse( int event, int x, int y,int flags, void *param ){
    MainWindow *mainwindow;
    cv::Point *p = (cv::Point*)param;
    mainwindow->onMouse(event, x, y, &p);

}

void MainWindow::onMouse(int event, int x, int y, void *param){
    if (event == cv::EVENT_LBUTTONDOWN){
//        int x_cord = x;
//        int y_cord = y;
//        qDebug() << x ;
//        cv::destroyWindow("logo position select");
        cv::Point* p = (cv::Point*)param;
        p->x = x;
        p->y = y;
        cv::destroyWindow("logo position select");


     }
}

void MainWindow::on_saltAndPepper_slider_valueChanged(int value)
{
    ui->saltAndPepper_lab->setText(QString::number(value));
    saltAndPepper_value = value;
}

void MainWindow::on_minvalstretch_slider_valueChanged(int value){
    ui->minValStretch_lab->setText(QString::number(value));
    minvalstretch_value = value;
}

void MainWindow::on_loadlogo_push_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.bmp)"));

    image_logo= cv::imread(fileName.toStdString(), 0);

}

// function to get current index of main tab
void MainWindow::on_main_tab_currentChanged(int index)
{
    current_index_main_tab = index;

}


// slot to check main tab index
void MainWindow::on_tabWidget_currentChanged(int index)
{
    current_index_line_tab = index;
    qDebug() << current_index_line_tab << "in slot";
}

// slot to check filter tab index
void MainWindow::on_filter_tab_currentChanged(int index)
{
    current_index_filter_tab = index;
}

// slot to check features tab index
void MainWindow::on_tabWidget_2_currentChanged(int index)
{
    current_index_feature_tab = index;
}

// slot to check match features radio button and activate load other image
void MainWindow::on_matchFeatures_radio_clicked()
{
    ui->loadOtherImage_push->setEnabled(true);
}

// slot to loa other image and display in new window
void MainWindow::on_loadOtherImage_push_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.bmp)"));

    image_inp_second = cv::imread(fileName.toStdString());
    cv::cvtColor(image_inp_second,image_inp_second,CV_BGR2RGB);  // change color channel ordering
    cv::cvtColor(image_inp_second,image_inp_gray_second,CV_RGB2GRAY); // to change input image to grayscale

    // to show second image in output image
    showOutput(image_inp_gray_second);


}

void MainWindow::on_calibrate_radio_clicked()
{
     ui->process_push->setEnabled(true);
}

void MainWindow::on_loadOtherImage_push_2_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.bmp)"));

    image_inp_second = cv::imread(fileName.toStdString());
    cv::cvtColor(image_inp_second,image_inp_second,CV_BGR2RGB);  // change color channel ordering
    cv::cvtColor(image_inp_second,image_inp_gray_second,CV_RGB2GRAY); // to change input image to grayscale

    // to show second image in output image
    showOutput(image_inp_gray_second);
}

void MainWindow::on_saveImage_push_clicked()
{
    cv::imwrite("output.jpg", image_op);
}


void MainWindow::on_startLive_push_clicked()
{
    int width = ui->input_lab->width();
    int height = ui->input_lab->height();
    cv::Size size(width,height);


    ui->stopLive_push->setEnabled(true); //enable stop capture buttton


    // setup video capture
        cv::VideoCapture capture(0);
        if (!capture.isOpened()){

          qDebug() << "Could not open VideoCapture";
        }

        cv::Mat imgRead;

        for (;;)
        {


            // capture a frame
            capture.read(imgRead);

            // convert image to gray
            cv::cvtColor(imgRead, imgRead, CV_BGR2GRAY );

            // to know the label width and height to scale the image

            cv::resize(imgRead, image_inp_gray, size ); // resize the frames

            //showInput(imgRead);
            imshow("Input video", image_inp_gray);
            cv::waitKey(1);

            // General Tab
            if (current_index_main_tab == 0){

                // to add salt and pepper noise
                if (ui->salt_radio->isChecked() == true){
                image_op = salt(image_inp_gray, saltAndPepper_value);
                imshow("Output video", image_op);
                cv::waitKey(1);
                }

                // to add logo
                else if (ui->logo_radio->isChecked() == true){
                    cv::Mat image_input = image_inp_gray.clone();
                    // define ROI
                    cv::Mat imageROI= image_input(cv::Rect(image_inp_gray.cols-image_logo.cols,image_inp_gray.rows-image_logo.rows,image_logo.cols,image_logo.rows));
                    image_logo.copyTo(imageROI,image_logo);
                    imshow("Output video", image_op);
                    cv::waitKey(1);
                }

                //to change color spaces
                else {
                    if(ui->graytorgb_radio->isChecked() == true){
                        cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->graytociexyz_radio->isChecked() == true){
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2XYZ);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                    else if(ui->graytociexyz_radio->isChecked() == true){
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2YCrCb);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                    else if(ui->graytociexyz_radio->isChecked() == true){
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2HSV);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                    else if(ui->graytociexyz_radio->isChecked() == true){
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2HLS);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                    else if(ui->graytociexyz_radio->isChecked() == true){
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2Lab);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                    else{
                       cv::cvtColor(image_inp_gray, image_op, CV_GRAY2RGB);
                       cv::cvtColor(image_op, image_op, CV_RGB2Luv);
                       imshow("Output video", image_op);
                       cv::waitKey(1);                    }
                }

            }

            //to do histograms
            else if(current_index_main_tab == 1){
                if(ui->plothistogram_radio->isChecked() == true){
                    image_op = histogram.getHistogramImage(image_inp_gray);
                    imshow("Output video", image_op);
                    cv::waitKey(1);                 }

            }

            //to do morphology
            else if(current_index_main_tab == 2){
                // to get structuring element RECT, ELLIPSE, CROSS
                if(ui->strelerect_radio->isChecked() == true){
                    strele = morphology.getstrele(1, ui->elesize_spin->value());
                }
                else if(ui->streleellip_radio->isChecked() == true){
                    strele = morphology.getstrele(2, ui->elesize_spin->value());
                }
                else{
                    strele = morphology.getstrele(3, ui->elesize_spin->value());
                }

                // to do morph operations DILATE, ERODE, OPEN, CLOSE ...
                if(ui->dilate_radio->isChecked() == true){
                    image_op = morphology.getMorph(image_inp_gray, 1, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
                    imshow("Output video", image_op);
                    cv::waitKey(1);                }
                else if(ui->erode_radio->isChecked() == true){
                    image_op = morphology.getMorph(image_inp_gray, 2, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
                    imshow("Output video", image_op);
                    cv::waitKey(1);                }
                else if(ui->open_radio->isChecked() == true){
                    image_op = morphology.getMorph(image_inp_gray, 3, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
                    imshow("Output video", image_op);
                    cv::waitKey(1);                }
                else if(ui->close_radio->isChecked() == true){
                    image_op = morphology.getMorph(image_inp_gray, 4, strele, ui->elesize_spin->value(), ui->morphiter_spin->value());
                    imshow("Output video", image_op);
                    cv::waitKey(1);                }

            }

            //to do filtering
            else if(current_index_main_tab == 3){

                if(current_index_filter_tab == 0){

                    if(ui->box_radio->isChecked() == true){

                        image_op = cfilter.getbox(image_inp_gray, ui->kersize_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->gaussian_radio->isChecked() == true){
                        image_op = cfilter.getGaussianBlurr(image_inp_gray, ui->kersize_spin->value(), ui->alongx_spin->value(), ui->alongy_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->median_radio->isChecked() == true){
                        image_op = cfilter.getMedianBlurr(image_inp_gray, ui->kersize_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->bilateral_radio->isChecked() == true){

                        image_op = cfilter.getBilateralBlurr(image_inp_gray, ui->kersize_spin->value(), ui->sdcolor_spin->value(), ui->sdspatial_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                }
                else{
                    //to do highpass filter images (sobel,scharr,laplacian, detect edges,...)
                    if(ui->sobel_radio->isChecked() == true){
                        image_op = cfilter.getSobel(image_inp_gray, ui->kerSize_spin->value(), ui->deralongx_spin->value(), ui->deralongy_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->scharr_radio->isChecked() == true){
                        image_op = cfilter.getScharr(image_inp_gray, ui->deralongx_spin->value(), ui->deralongy_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->laplacian_radio->isChecked() == true){
                        image_op = cfilter.getLaplacian(image_inp_gray, ui->kersizelap_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else if(ui->edgeSobel_radio->isChecked() == true){
                        image_op = cfilter.getEdgesSobel(image_inp_gray, ui->threshEdge_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                    else{
                        image_op = laplacianedge.computeLaplacian(image_inp_gray, ui->kersizelap_spin->value());
                        double lapmin, lapmax;
                        cv::minMaxLoc(image_op,&lapmin,&lapmax);
                        image_op = laplacianedge.getLaplacianImage();
                        image_op = laplacianedge.getZeroCrossings(lapmax);
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                }

            }

            // to detect lines, circles, contours
            else if(current_index_main_tab == 4) {
                if(current_index_line_tab == 0){
                    image_op = linesAndContours.drawEdgeCanny(image_inp_gray, ui->lowThreshold_spin->value(), ui->highThreshold_spin->value(), ui->kernalSize_spin->value());
                    imshow("Output video", image_op);
                    cv::waitKey(1);
                }
                //to draw lines and circles using hough transform
                else if(current_index_line_tab == 1){
                    if(ui->lineAct_radio->isChecked() == true){
                        qDebug() << "in linehough";
                       image_op = linesAndContours.drawLinesHough(image_inp_gray, ui->lowThresholdHough_spin->value(), ui->highThresholdHough_spin->value(), ui->distStepSize_spin->value(), ui->angleStepSize_spin->value(), ui->votes_spin->value(), ui->minLength_spin->value(), ui->maxGap_spin->value());
                       imshow("Output video", image_op);
                       cv::waitKey(1);
                    }
                    else{
                        image_op = linesAndContours.drawCirclesHough(image_inp_gray, ui->accumulatorResolution_spin->value(), ui->minDistBetCircles_spin->value(), ui->highThresholdCircles_spin->value(), ui->minVotes_spin->value(), ui->minRadius_spin->value(), ui->maxRadius_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);
                    }
                }
            }

            //to get and match features
            else if(current_index_main_tab == 5){
                // to get harris corners
                if(current_index_feature_tab == 0){
                    if(ui->harrisCornerAct_radio->isChecked() == true){
                        image_op = harrisCorner.getHarrisCorners(image_inp_gray, ui->neighbourhoodSize_spin->value(), ui->kernalSizeHarris_spin->value(), ui->parameterK_spin->value(), ui->qualityLevel_spin->value(), ui->neighbourhoodSizeNonMax_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);                    }
                    else{
                        image_op = harrisCorner.getUniformCorners(image_inp_gray, ui->maxCorners_spin->value(), ui->qualityLevelFeature_spin->value(), ui->minDistance_spin->value());
                        imshow("Output video", image_op);
                        cv::waitKey(1);                    }
                }
            }

            if (ui->stopLive_push->isDown() == true){
                cv::destroyAllWindows();
                ui->stopLive_push->setEnabled(false);
                ui->video_radio->setEnabled(false);
                break;

            }


        }
}


void MainWindow::on_stopLive_push_clicked()
{

}

//to disable non video options
void MainWindow::on_video_radio_clicked()
{
    ui->equalisehistogram_radio->setEnabled(false);
    ui->stretchhistogram_radio->setEnabled(false);
    ui->gradient_radio->setEnabled(false);
    ui->tophat_radio->setEnabled(false);
    ui->blackhat_radio->setEnabled(false);
    ui->hitmiss_radio->setEnabled(false);

    ui->boundingBox_radio->setEnabled(false);
    ui->circle_radio->setEnabled(false);
    ui->polygon_radio->setEnabled(false);
    ui->convexHull_radio->setEnabled(false);
    ui->moments_radio->setEnabled(false);

    ui->surf_radio->setEnabled(false);
    ui->sift_radio->setEnabled(false);
    ui->fast_radio->setEnabled(false);
    ui->matchFeatures_radio->setEnabled(false);

    ui->calibrate_radio->setEnabled(false);
    ui->calculateF_radio->setEnabled(false);
    ui->homography_radio->setEnabled(false);

    ui->images_radio->setEnabled(true);

}

void MainWindow::on_images_radio_clicked()
{
    ui->equalisehistogram_radio->setEnabled(true);
    ui->stretchhistogram_radio->setEnabled(true);
    ui->gradient_radio->setEnabled(true);
    ui->tophat_radio->setEnabled(true);
    ui->blackhat_radio->setEnabled(true);
    ui->hitmiss_radio->setEnabled(true);

    ui->boundingBox_radio->setEnabled(true);
    ui->circle_radio->setEnabled(true);
    ui->polygon_radio->setEnabled(true);
    ui->convexHull_radio->setEnabled(true);
    ui->moments_radio->setEnabled(true);

    ui->surf_radio->setEnabled(true);
    ui->sift_radio->setEnabled(true);
    ui->fast_radio->setEnabled(true);
    ui->matchFeatures_radio->setEnabled(true);

    ui->calibrate_radio->setEnabled(true);
    ui->calculateF_radio->setEnabled(true);
    ui->homography_radio->setEnabled(true);

    ui->video_radio->setEnabled(true);
}

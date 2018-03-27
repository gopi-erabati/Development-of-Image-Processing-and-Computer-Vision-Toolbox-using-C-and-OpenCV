#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QRadioButton>
#include <QSlider>
#include <QLabel>
#include <iostream>
#include <QDir>
#include <QString>
#include <QDebug>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <thread>

//#include "inoutsave.h"
#include "histogram.h"
#include "morphology.h"
#include "Cfilter.h"
#include "laplacianedge.h"
#include "linesandcontours.h"
#include "harriscorner.h"
#include "features.h"
#include "cameracalibration.h"
#include "getfanddrawepi.h"


namespace Ui {
class MainWindow;
}



class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::MainWindow *ui;
    cv::Mat image_inp; // input image varaiable
    cv::Mat image_inp_gray;
    cv::Mat image_inp_second; //second image to match
    cv::Mat image_inp_gray_second; // second image in gray
    cv::Mat image_op;
    cv::Mat image_logo;
    cv::Mat strele;
    QImage imageView;
    int saltAndPepper_value;
    int minvalstretch_value;
    int current_index_main_tab; // for current index of main tab
    int current_index_line_tab; // for lines tab
    int current_index_filter_tab;
    int current_index_feature_tab;
    cv::Point point;
    std::vector<std::string> filelist; //to store filenames of chessboard pattern
    cv::Mat image; //to store chessdboard images
    cv::Mat uImage; // to store undistorted image
    cv::Mat cameraMatrix; //to store camera matrix
    cv::Mat fundamental; //to store fundamental matrix
    cv::Mat homography; //to store homography matrix

    //objects for classes
    Histogram histogram;
    Morphology morphology;
    Cfilter cfilter;
    LaplacianEdge laplacianedge;
    LinesAndContours linesAndContours;
    HarrisCorner harrisCorner;
    Features features;
    CameraCalibration cameraCalibration;
    GetFAndDrawEpi getFAndDrawEpi;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void showInput(cv::Mat image_inp_gray); // to show input image
    void showOutput(cv::Mat); // to show output image
    cv::Mat salt(const cv::Mat , int); // to add salt and pepper
    void convert_color(); // to convert colors
    void plotHistograms(); //to plost histograms
    void getMorphology(); // to get morphology
    void getFilter(); // to get filtered images
    void getEdgesLinesContours(); // to get edges, lines, contours
    void getFeaturesAndMatches(); // to get harris corners, surf, sift, fast
    void getCalibrateAndParameters(); // to get camera calibration, fundamental matrix, Epipolarlines, Homography

    // to getui pointer from other class
    Ui::MainWindow* getui();

    void onMouse(int event, int x, int y, void *param);
    static void onMouse(int event, int x, int y,int flags,void *param);

private slots:
    void on_loadImage_push_clicked();


    void on_process_push_clicked();
    void on_saltAndPepper_slider_valueChanged(int value);
    void on_minvalstretch_slider_valueChanged(int value);
    void on_loadlogo_push_clicked();
    void on_main_tab_currentChanged(int index);
    void on_tabWidget_currentChanged(int index);
    void on_filter_tab_currentChanged(int index);
    void on_tabWidget_2_currentChanged(int index);
    void on_matchFeatures_radio_clicked();
    void on_loadOtherImage_push_clicked();
    void on_calibrate_radio_clicked();
    void on_loadOtherImage_push_2_clicked();
    void on_saveImage_push_clicked();
    void on_stopLive_push_clicked();
    void on_startLive_push_clicked();
    void on_video_radio_clicked();
    void on_images_radio_clicked();
};

#endif // MAINWINDOW_H

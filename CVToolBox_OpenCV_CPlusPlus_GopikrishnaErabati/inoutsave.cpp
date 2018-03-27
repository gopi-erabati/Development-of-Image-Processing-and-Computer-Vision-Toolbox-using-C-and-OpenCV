//#include "inoutsave.h"
//#include "interface.h"



//Inoutsave::Inoutsave(){ }

//cv::Mat Inoutsave::openImage(){
//    //mainwindow = new MainWindow();

//    uinew = interface->getUi();

//    QString fileName = QFileDialog::getOpenFileName(this,
//                                                    tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.bmp)"));

//    image_inp= cv::imread(fileName.toStdString());
//    cv::cvtColor(image_inp,image_inp,CV_BGR2RGB);  // change color channel ordering
//    cv::cvtColor(image_inp,image_inp_gray,CV_RGB2GRAY); // to change input image to grayscale


//    if (image_inp.data) {

//        uinew->process_push->setEnabled(true);
//    }

//    imageView = QImage((const unsigned char*)(image_inp_gray.data),  // Qt image structure
//                       image_inp_gray.cols,image_inp_gray.rows,image_inp_gray.step,QImage::Format_Indexed8);

//    // to know the label width and height to scale the image
//    int width = uinew->input_lab->width();
//    int height = uinew->input_lab->height();


//    // convert QImage to QPixmap and show on QLabel
//    uinew->input_lab->setPixmap(QPixmap::fromImage(imageView).scaled(width,height,Qt::KeepAspectRatio));

//    return image_inp_gray;
//}



//void Inoutsave::saveOutputImage(cv::Mat image){
//    cv::imwrite("output.jpg", image);
//}

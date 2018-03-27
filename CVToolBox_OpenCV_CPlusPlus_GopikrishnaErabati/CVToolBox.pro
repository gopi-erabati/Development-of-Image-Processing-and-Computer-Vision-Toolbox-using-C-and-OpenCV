#-------------------------------------------------
#
# Project created by QtCreator 2017-04-22T23:16:10
#
#-------------------------------------------------
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = untitled
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    inoutsave.cpp \
    histogram.cpp \
    morphology.cpp \
    cfilter.cpp \
    laplacianedge.cpp \
    linesandcontours.cpp \
    harriscorner.cpp \
    features.cpp \
    interface.cpp \
    cameracalibration.cpp \
    getfanddrawepi.cpp \
    robustmatcher.cpp

HEADERS  += mainwindow.h \
    inoutsave.h \
    histogram.h \
    morphology.h \
    cfilter.h \
    laplacianedge.h \
    linesandcontours.h \
    harriscorner.h \
    features.h \
    interface.h \
    cameracalibration.h \
    getfanddrawepi.h \
    robustmatcher.h

FORMS    += mainwindow.ui

INCLUDEPATH += C:\opencv\opencv-build\install\include
LIBS += -LC:\opencv\opencv-build\install\x86\mingw\lib \
    -lopencv_core320.dll \
    -lopencv_highgui320.dll \
    -lopencv_imgcodecs320.dll \
    -lopencv_imgproc320.dll \
    -lopencv_features2d320.dll \
    -lopencv_calib3d320.dll \
    -lopencv_xfeatures2d320.dll \
    -lopencv_flann320.dll \
    -lopencv_videoio320.dll \
    -lopencv_ffmpeg320.dll \

RESOURCES += \
    images.qrc

DISTFILES += \
    ../build-sample-Desktop_Qt_5_9_0_MinGW_32bit-Debug/images/logo.bmp

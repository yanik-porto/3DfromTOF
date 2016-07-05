#-------------------------------------------------
#
# Project created by QtCreator 2016-06-13T11:21:29
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11

TARGET = tofUcblApp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    CameraManager.cpp \
    pclManager.cpp

HEADERS  += mainwindow.h \
    CameraManager.h \
    pclManager.h

FORMS    += mainwindow.ui

INCLUDEPATH +=  /usr/include/pcl-1.7 \
                /usr/include/eigen3 \
                /usr/include/voxel-0.6.1 \
                /usr/include/boost \
                /usr/include/vtk-5.8


LIBS += -L/usr/lib \
        -lpcl_common -lpcl_io -lpcl_visualization -lpcl_features

LIBS += -L/usr/lib \
        -lvoxel -lti3dtof -lvoxelpcl

LIBS += -L/usr/lib/x86_64-linux-gnu \
        -lboost_system \
        -lboost_thread \
        -lboost_filesystem \
        -lboost_date_time

LIBS += -L/usr/lib/vtk-5.8 \
        -lvtkCommon \
        -lvtkFiltering \
        -lvtkRendering \
        -lQVTK \
        -lvtkIO \
        -lvtkWidgets \
        -lvtkGraphics

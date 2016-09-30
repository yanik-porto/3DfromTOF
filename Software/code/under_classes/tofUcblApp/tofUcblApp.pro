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

CONFIG += serialport

SOURCES += main.cpp\
        mainwindow.cpp \
    CameraManager.cpp \
    pclManager.cpp \
    icp_registration.cpp

HEADERS  += mainwindow.h \
    CameraManager.h \
    pclManager.h \
    icp_registration.h

FORMS    += mainwindow.ui

INCLUDEPATH +=  /usr/local/include/pcl-1.8 \ #/usr/include/pcl-1.7 \
                /usr/include/eigen3 \
                /usr/include/voxel \ #/usr/include/voxel-0.6.1 \
                /usr/include/boost \
                /usr/include/vtk-5.8


LIBS +=  -L/usr/local/lib \ #-L/usr/lib \
        -lpcl_common \
        -lpcl_filters \
        -lpcl_io \
        -lpcl_visualization \
        -lpcl_surface \
         -lpcl_features \
#        -lpcl_stereo \
#        -lpcl_tracking \
#        -lpcl_segmentation \
        -lpcl_search \
        -lpcl_registration \
#        -lpcl_sample_consensus

LIBS += -L/usr/lib \
        -lvoxel -lti3dtof -lvoxelpcl

LIBS += -L/usr/lib/x86_64-linux-gnu \
        -lboost_system \
        -lboost_thread \
#        -lboost_atomic \
#        -lboost_chrono \
#        -lboost_context \
#        -lboost_filesystem \
#        -lboost_date_time \
#        -lboost_graph \
#        -lboost_graph_parallel \
#        -lboost_iostreams \
#        -lboost_locale \
#        -lboost_log \
#        -lboost_math_c99 \
#        -lboost_timer \
#        -lboost_mpi \


LIBS += -L/usr/lib/vtk-5.8 \
        -lvtkCommon \
        -lvtkFiltering \
        -lvtkRendering \
        -lQVTK \
#        -lvtkIO \
        -lvtkWidgets \
#        -lvtkGraphics

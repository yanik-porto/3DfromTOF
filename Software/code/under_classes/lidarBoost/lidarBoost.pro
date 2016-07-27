#-------------------------------------------------
#
# Project created by QtCreator 2016-07-11T14:57:02
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = lidarBoost
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

HEADERS += \
    pclManager.h \
    lidarboostengine.h

SOURCES += main.cpp \
    pclManager.cpp \
    lidarboostengine.cpp

#for PCL
INCLUDEPATH +=  /usr/include/pcl-1.7 \
                /usr/include/eigen3 \
                /usr/include/boost \
                /usr/include/vtk-5.8

LIBS += -L/usr/lib \
        -lpcl_common \
        -lpcl_filters \
        -lpcl_io \
        -lpcl_visualization \
        -lpcl_surface \
#        -lpcl_stereo \
#        -lpcl_tracking \
#        -lpcl_segmentation \
#        -lpcl_search \
        -lpcl_registration \
#        -lpcl_sample_consensus


LIBS += -L/usr/lib/x86_64-linux-gnu \
        -lboost_system \
        -lboost_thread \

LIBS += -L/usr/lib/vtk-5.8 \
        -lvtkCommon \
        -lvtkFiltering \
        -lvtkRendering \
        -lQVTK \
        -lvtkWidgets \

#For OpenCV
INCLUDEPATH += /usr/local/include
LIBS += -L"/usr/local/lib"
LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs


#For Ceres
INCLUDEPATH += /usr/local/include/ceres \
               /usr/include/eigen3 \
               /usr/include/glog \
               /usr/include/suitesparse \
#                /usr/include/atlas \
                /usr/lib/openmpi/include

LIBS += -L/usr/local/lib \
        -lceres

LIBS += -L/usr/lib/x86_64-linux-gnu \
        -lglog \
        -lsuitesparseconfig -lcholmod

LIBS += -L/usr/lib \
        -lblas -llapack
#        -latlas

LIBS += -L/usr/lib/openmpi/lib \
        -lmpi -fopenmp

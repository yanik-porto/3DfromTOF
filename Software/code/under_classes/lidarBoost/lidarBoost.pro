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


SOURCES += main.cpp \
    pclManager.cpp \
    lidarboostengine.cpp

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
        -lvtkWidgets \

HEADERS += \
    pclManager.h \
    lidarboostengine.h

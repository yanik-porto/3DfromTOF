#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
//#include <CameraSystem.h>
#include "CameraManager.h"
#include "pclManager.h"

//using namespace Voxel;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

//    Voxel::String strVox;
//    CameraSystem sys;

private slots:
    void on_pushButton_capture_clicked();

    void on_pushButton_visu_clicked();

private:
    Ui::MainWindow *ui;
    CameraManager mngCam;
    pclManager mngPcl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

#endif // MAINWINDOW_H

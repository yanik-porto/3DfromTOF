#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <map>
//#include <CameraSystem.h>
#include "CameraManager.h"
#include "pclManager.h"
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

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


private slots:

    /**
     * Activated when an event on the widgets occures
     */
    void on_pushButton_capture_clicked();

    void on_pushButton_visu_clicked();

private:
    Ui::MainWindow *ui;

    /**
     * Initialize the PCLVisualizer through the QVTKWidget
     */
    void init_viewer();

    /**
     * Instance of the classes
     */
    CameraManager mngCam;
    pclManager mngPcl;

    /**
     * Current PointCloud and PCLVisualizer
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    /**
     * List of calibration profiles
     */
    std::map<std::string, int> profilmap;


};

#endif // MAINWINDOW_H

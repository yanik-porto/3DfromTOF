#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QWidget>
#include <QListWidget>
#include <QFileDialog>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <map>
//#include <CameraSystem.h>
#include "CameraManager.h"
#include "pclManager.h"
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QIODevice>

#include "icp_registration.h"

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

    void display_descriptions();

    /**
     * Activated when an event on the widgets occures
     */
    void on_pushButton_capture_clicked();

    void on_pushButton_visu_clicked();

    void on_pushButton_import_clicked();

    void on_pushButton_save_clicked();

    void on_horizontalSlider_filterz_valueChanged(int value);

    void on_pushButton_filter_clicked();


    void on_horizontalSlider_filteri_valueChanged(int value);

    void on_pushButton_arduino_clicked();


    void on_horizontalSlider_filtery_valueChanged(int value);

    void on_horizontalSlider_filterx_valueChanged(int value);


    void on_pushButton_merge_clicked();

private:
    Ui::MainWindow *ui;

    /**
     * Initialize the PCLVisualizer through the QVTKWidget
     */
    void init_viewer();

    /**
     * Create property bar
     */
    void create_actions();

    /**
     * Instance of the classes
     */
    CameraManager mngCam;
    pclManager mngPcl;

    /**
     * Current PointCloud and PCLVisualizer
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_clouds;
    float valx, valy, valz, vali;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    /**
     * List of calibration profiles
     */
    std::map<std::string, int> profilmap;

    /**
     * List of Actions
     */
    QAction *descr;

    /**
     * Port of the Arduino board
     */
    QSerialPort *serial;

    /**
     * Port name of the Arduino board
     */
    QString arduino_port_name = "";

//    ICP_Registration icpreg;
};

#endif // MAINWINDOW_H

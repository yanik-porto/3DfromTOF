#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    std::cout<<__cplusplus<<std::endl;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//    if(pcl::io::loadPCDFile<pcl::PointXYZ>("speakerRight.pcd",*cloud) == -1)
//    {
////        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    }
//    else
//    {
//        std::cout << "Loaded "
//                  << cloud->width * cloud->height
//                  << " data points from test_pcd.pcd with the following fields: "
//                  << std::endl;
////        for (size_t i = 0; i < cloud->points.size (); ++i)
////          std::cout << "    " << cloud->points[i].x
////                    << " "    << cloud->points[i].y
////                    << " "    << cloud->points[i].z << std::endl;
//    }

    ui->pushButton_visu->setEnabled(false);

    std::vector<std::string> list = mngCam.get_devices_name();
    profilmap = mngCam.get_profiles_name();

    for(int i=0; i<list.size(); i++)
        ui->comboBox_devices->addItem(QString::fromStdString(list[i]));

    typedef std::map<std::string, int>::iterator itmap_type;

    for(itmap_type iter = profilmap.begin(); iter != profilmap.end(); iter++)
    {
        ui->comboBox_calib->addItem(QString::fromStdString( iter->first ));
    }



    init_viewer();

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);

    // The number of points in the cloud
    cloud->points.resize (200);

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    mngPcl.init_viewer(viewer);
    mngPcl.simpleVis(cloud);
    ui->widget_visu->update();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_viewer()
{
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget_visu->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget_visu->GetInteractor(), ui->widget_visu->GetRenderWindow());
    ui->widget_visu->update();
}

void MainWindow::on_pushButton_capture_clicked()
{

    //Get index of selected device
    mngCam.set_numOfShots(ui->spinBox_nShots->value());
    int sel_device = ui->comboBox_devices->currentIndex();

    //Get index of camera calibration mode
    std::string sel_calib = ui->comboBox_calib->currentText().toStdString();
    int idCalib = profilmap[sel_calib];

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = mngCam.capture(sel_device, idCalib);
    mngPcl.set_cloud(cloud);
    ui->pushButton_visu->setEnabled(true);
    ui->label_infos->setText("Captured");
    //cloud = mngPcl.filter_cloud(cloud);

}

void MainWindow::on_pushButton_visu_clicked()
{

    if(ui->radioButton_viewer->isChecked())
        mngPcl.visualizePcl();

    else if(ui->radioButton_vizualizer->isChecked())
    {
        //viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
        //init_viewer();
        //mngPcl.init_viewer(viewer);
        viewer->removeAllPointClouds();
        viewer = mngPcl.simpleVis(cloud);
        // Set up the QVTK window
        //viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

//        while (!viewer->wasStopped ())
//        {
//          viewer->spinOnce (100);
//          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//        }

        //ui->widget_visu->SetRenderWindow(viewer->getRenderWindow());
        //viewer->setupInteractor(ui->widget_visu->GetInteractor(), ui->widget_visu->GetRenderWindow());
        ui->widget_visu->update();
    }
}


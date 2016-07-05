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

    //std::cout << list[0] << std::endl;
    //QString test =  QString::fromStdString(list[0]);
    //std::cout << test.toStdString() << std::endl;

    for(int i=0; i<list.size(); i++)
        ui->comboBox_devices->addItem(QString::fromStdString(list[i]));

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget_visu->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget_visu->GetInteractor(), ui->widget_visu->GetRenderWindow());
    ui->widget_visu->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_capture_clicked()
{
    mngCam.set_numOfShots(ui->spinBox_nShots->value());
    int sel_device = ui->comboBox_devices->currentIndex();
    cloud = mngCam.capture(sel_device);
    mngPcl.set_cloud(cloud);
    ui->pushButton_visu->setEnabled(true);
    ui->label_infos->setText("Captured");

}

void MainWindow::on_pushButton_visu_clicked()
{

    if(ui->radioButton_viewer->isChecked())
        mngPcl.visualizePcl();

    else if(ui->radioButton_vizualizer->isChecked())
    {
        viewer = mngPcl.simpleVis(cloud, viewer);
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


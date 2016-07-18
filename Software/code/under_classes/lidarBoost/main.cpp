#include <QCoreApplication>
#include "pclManager.h"
#include "lidarboostengine.h"
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    pclManager mngPcl;
    lidarBoostEngine engLidBoost;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    string filename = "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/bottle.pcd";

    mngPcl.set_cloud_from_pcd(filename);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = mngPcl.get_cloud();
    engLidBoost.set_cloud(cloud);


    engLidBoost.test_apply_lk();


    mngPcl.init_viewer(viewer);
    viewer = mngPcl.simpleVis(cloud);



    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return a.exec();
}

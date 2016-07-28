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

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    string filename = "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/test.pcd";
//    string filename = "/home/yanik/Documents/Interniship_UCBL/Acquisition/bottleLR.pcd";
    vector<string> list_files(5);
    list_files[0] =  "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/sugar1.pcd";
    list_files[1] =  "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/sugar2.pcd";
    list_files[2] =  "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/sugar3.pcd";
    list_files[3] =  "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/sugar4.pcd";
    list_files[4] =  "/home/yanik/qtProjects/build-tofUcblApp-Desktop-Debug/sugar5.pcd";

//    mngPcl.set_cloud_from_pcd(filename);

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = mngPcl.get_cloud();
//    engLidBoost.set_cloud(cloud);

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_clouds = mngPcl.set_multiple_clouds_from_pcd(list_files);

    std::cout << list_clouds.size() << std::endl;

    engLidBoost.set_from_list_clouds(list_clouds);

//    vector<int> listClouds(2);
//    listClouds[0]=3;
//    listClouds[1]=14;
//    engLidBoost.set_selected_cloud(cloud, listClouds);
//    std::cout << cloud->width << std::endl;


    engLidBoost.build_superresolution(4);


//    mngPcl.init_viewer(viewer);
//    viewer = mngPcl.simpleVis(cloud);



//    while (!viewer->wasStopped ())
//    {
//      viewer->spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

    return a.exec();
}

#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <iomanip>
//#include <pcl/visualization/cloud_viewer.h>
//#include <CameraSystem.h>
#include "CameraManager.h"
#include "pclManager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main()
{

	CameraManager cam;
	pclManager ptcloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

	bool isCaptured = false;
	isCaptured = cam.capture(2);

	if (isCaptured)
		cloud = cam.convert2pcl();

	system("pause");

	bool isInitCloud = false;
	isInitCloud = ptcloud.set_cloud(cloud);
	if (isInitCloud)
		ptcloud.visualizePcl();


	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	//if (pcl::io::loadPCDFile<pcl::PointXYZI>("test_pcd2.pcd", *cloud) == -1) //* load the file
	//{
	//	PCL_ERROR("Couldn't read file test_pcd.pcd \n");
	//	return (-1);
	//}
	//std::cout << "Loaded "
	//	<< cloud->width * cloud->height
	//	<< " data points from test_pcd.pcd with the following fields: "
	//	<< std::endl;
	////for (size_t i = 0; i < cloud->points.size(); ++i)
	////	std::cout << "    " << cloud->points[i].x
	////	<< " " << cloud->points[i].y
	////	<< " " << cloud->points[i].z << std::endl;

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped())
	//{
	//}
	//std::cout << sizeof(Voxel::IntensityPoint) << std::endl;

	system("pause");

	return (0);
}
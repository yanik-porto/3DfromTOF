#include <iostream>
#include "CameraManager.h"
#include "pclManager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main()
{

	CameraManager cam;
	pclManager ptcloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

	cloud = cam.capture(2);

	system("pause");

	bool isInitCloud = false;
	isInitCloud = ptcloud.set_cloud(cloud);
	if (isInitCloud)
		ptcloud.visualizePcl();

	system("pause");

	return (0);
}
#include "pclManager.h"


pclManager::pclManager()
{
}


pclManager::~pclManager()
{
}

bool pclManager::set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud)
{
	cloud = ptcloud;

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << std::endl;
	return true;
}

bool pclManager::set_cloud_from_pcd(const std::string &filename)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPcd(new pcl::PointCloud<pcl::PointXYZI>);
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloudPcd) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}
	std::cout << "Loaded "
		<< cloudPcd->width * cloudPcd->height
		<< " data points from .pcd with the following fields: "
		<< std::endl;

	cloud = cloudPcd;
	return true;
}

bool pclManager::save2pcd(const std::string &filename)
{
	pcl::io::savePCDFileASCII(filename, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;
	return true;
}

void pclManager::visualizePcl()
{
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(&cloud);
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
}
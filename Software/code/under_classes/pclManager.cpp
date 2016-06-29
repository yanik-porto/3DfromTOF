#include "pclManager.h"


pclManager::pclManager()
{
}


pclManager::~pclManager()
{
}

bool pclManager::set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud)
{
	//Set the cloud with an already existing pointCloud
	cloud = ptcloud;

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from point cloud"
		<< std::endl;
	return true;
}

bool pclManager::set_cloud_from_pcd(const std::string &filename)
{
	//Set the cloud from a .pcd file
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPcd(new pcl::PointCloud<pcl::PointXYZI>);
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloudPcd) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}
	std::cout << "Loaded "
		<< cloudPcd->width * cloudPcd->height
		<< " data points from .pcd "
		<< std::endl;

	cloud = cloudPcd;
	return true;
}

bool pclManager::save2pcd(const std::string &filename)
{
	//Save the cloud in a .pcd file
	pcl::io::savePCDFileASCII(filename, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;
	return true;
}

void pclManager::visualizePcl()
{
	//Display the cloud
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
}
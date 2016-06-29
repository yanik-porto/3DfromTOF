#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

class pclManager
{
public:
	pclManager();
	~pclManager();

	bool set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);
	bool set_cloud_from_pcd(const std::string &);

	bool save2pcd(const std::string &);
	void visualizePcl();


private:
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};


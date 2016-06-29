#pragma once

#include <CameraSystem.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>

class CameraManager
{
public:
	CameraManager();
	~CameraManager();

	//Accessors
	void get_pts(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);

	pcl::PointCloud<pcl::PointXYZI>::Ptr capture(short);
	pcl::PointCloud<pcl::PointXYZI>::Ptr convert2pcl(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);


private:
	std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer > intPts;
	int sz_cloud;
	short numOfShots;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};


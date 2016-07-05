#pragma once

#include <CameraSystem.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>

class CameraManager
{
public:
	CameraManager();
    CameraManager(const short &);
	~CameraManager();

	//Accessors
	void get_pts(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);
    void set_numOfShots(const short &);

    pcl::PointCloud<pcl::PointXYZI>::Ptr capture(const short &);
	pcl::PointCloud<pcl::PointXYZI>::Ptr convert2pcl(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);
    std::vector<std::string> get_devices_name();


private:
	std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer > intPts;
	int sz_cloud;
	short numOfShots;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};


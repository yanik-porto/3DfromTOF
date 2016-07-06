#pragma once

#include <CameraSystem.h>
#include <DepthCamera.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <map>
#include <string>

class CameraManager
{
public:
    /**
     * Constructors and Destructors
     */
	CameraManager();
    CameraManager(const short &);
	~CameraManager();

    /**
     * Accessors and Mutators
     */
	void get_pts(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);
    void set_numOfShots(const short &);

    /**
     * return list of connected devices and calibration modes as string
     */
    std::vector<std::string> get_devices_name();
    std::map<std::string, int> get_profiles_name();

    /**
     * Main function for capturing frames from the specified device and calibration mode
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr capture(const short &, const int &);

    /**
     * Convert from vector of voxel intensity point to pcl::PointCloud
     */
	pcl::PointCloud<pcl::PointXYZI>::Ptr convert2pcl(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);




private:
    /**
     * vector of intensity points where are stored the coordinates of the points
     * coming directly from the the camera
     */
	std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer > intPts;

    /**
     * Size of a point cloud built from one shot only
     */
	int sz_cloud;

    /**
     * Number of desired shots to store in the point cloud
     */
	short numOfShots;

    /**
     * Final output point cloud
     */
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};


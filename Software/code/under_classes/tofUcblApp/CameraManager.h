#pragma once

#include <CameraSystem.h>
#include <DepthCamera.h>
#include <VideoMode.h>
//#include <Configuration.h>
//#include <Parameter.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <map>
//#include <string>
#include <QObject>
#include <unistd.h>
#include <memory>

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
    void set_freq(float);

    /**
     * return list of connected devices and calibration modes as string
     */
    const short &get_numb_connected_devices();
    std::vector<std::string> get_devices_name();
    std::map<std::string, int> get_profiles_name();
    std::map<std::string, std::string> get_param_descr(const short &);
    std::vector<float> get_supported_frameRate();

    /**
     * Main function for capturing frames from the specified device and calibration mode
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr capture(const short &, const int &);

    /**
     * Convert from vector of voxel intensity point to pcl::PointCloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr convert2pcl(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >);

private:

    bool captureOn;

    /**
     * vector of intensity points where are stored the coordinates of the points
     * coming directly from the camera
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

    /**
     * Camera frequency
     */
    float freq;

};


#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
#include <QVTKWidget.h>
#include <iostream>
//#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/filters/voxel_grid.h>

class pclManager
{
public:

    /**
     * Constructors and Destructors
     */
	pclManager();
	~pclManager();

    /**
     * Accessors and Mutators
     */
	bool set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);
	bool set_cloud_from_pcd(const std::string &);

    /**
     * Initialize the PCLVisualizer by pointing to an already existing PCLVisualizer
     */
    void init_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>);

    /**
     * Save the current stored pointcloud into the specified pcd file
     */
	bool save2pcd(const std::string &);

    /**
     * Convert PointCloud<PointXYZI> to PointCloud<PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_from_xyzi(pcl::PointCloud<pcl::PointXYZI>::ConstPtr);

    /**
     * Visualize the current stored pointcloud with the PCLViewer tool
     */
	void visualizePcl();

    /**
     * Display a given point cloud inside the PCVisualizer
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr);

    //pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);


private:
    /**
     * Stored cloud
     */
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    /**
     * PCLVisualizer
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    /**
     * number of clouds passed through the class
     */
    int nCloud;
};


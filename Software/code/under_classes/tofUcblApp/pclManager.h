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
	pclManager();
	~pclManager();

	bool set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);
	bool set_cloud_from_pcd(const std::string &);

	bool save2pcd(const std::string &);
	void visualizePcl();

    void init_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_from_xyzi(pcl::PointCloud<pcl::PointXYZI>::ConstPtr);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);


private:
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    int nCloud;
};


#ifndef LIDARBOOSTENGINE_H
#define LIDARBOOSTENGINE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <iostream>

using namespace Eigen;

class lidarBoostEngine
{
public:
    lidarBoostEngine();
    ~lidarBoostEngine();

    void set_cloud( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );

    std::vector< MatrixXd > convert_pcl_to_eigen( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );
private:

    //pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;

    int m, n, sz, n_cloud, one_pcl_sz;
    std::vector< MatrixXd > Y, W, D;
    SparseMatrix<double> T;

};

#endif // LIDARBOOSTENGINE_H

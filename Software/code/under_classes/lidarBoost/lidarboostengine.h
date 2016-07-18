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

    template <typename Derived, typename Derived2 >
    Derived conv2d(const MatrixBase<Derived>& I, const MatrixBase<Derived2> &kernel );

    template <typename Derived>
    std::vector< Derived > lk_optical_flow(const MatrixBase<Derived>& I1, const MatrixBase<Derived> &I2, int );


    void test_apply_lk();

private:

    //pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;

    int m, n, sz, n_cloud, one_pcl_sz;
    std::vector< MatrixXd > Y, W, D;
    SparseMatrix<double> T;

};

#endif // LIDARBOOSTENGINE_H

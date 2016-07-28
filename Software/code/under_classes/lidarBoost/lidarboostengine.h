#ifndef LIDARBOOSTENGINE_H
#define LIDARBOOSTENGINE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <iostream>
#include <math.h>

using namespace Eigen;
using namespace ceres;

class lidarBoostEngine
{
public:
    lidarBoostEngine();
    ~lidarBoostEngine();

    /**
     * Accessors and Mutators
     */
    void set_cloud( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );

    void set_selected_cloud( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int> numOfClouds);

    void set_from_list_clouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_clouds);
    /**
     * Convert a cloud made from different point clouds of "one_pcl_sz" points to
     * a vector of Eigen Matrices, representing a Depth image, with a width of m and a heigh of n
     */
    std::vector< MatrixXd > convert_pcl_to_eigen( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );

    /**
     * Build intensity images, intensity maps from the intensity layer of all the pointclouds
     * composing the cloud
     */
    std::vector< MatrixXd > get_intensity_images( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud );

    /**
     * Apply a convolution to an image I with a given kernel
     */
    template <typename Derived, typename Derived2 >
    Derived conv2d(const MatrixBase<Derived>& I, const MatrixBase<Derived2> &kernel );

    /**
     * Get the two matrices u and v of the optical flow obtained with the method of
     * lukas and kanade
     */
    template <typename Derived>
    std::vector< Derived > lk_optical_flow(const MatrixBase<Derived>& I1, const MatrixBase<Derived> &I2, int );

    /**
     * Correct a the position of an image with the optical flow computed with a reference frame
     */
    template <typename Derived>
    Derived apply_optical_flow(const MatrixBase<Derived>& I, std::vector< Derived > uv);

    /**
     * Get a mask discarding the too weak samples
     */
    MatrixXd check_unreliable_samples(MatrixXd map, double thresh);

    /**
     * Fill holes in a matrix with the nearest neighbors
     */
    template <typename Derived>
    Derived nearest_neigh_upsampling(Derived M);

    /**
     *  Compute Regularization term
     */
    MatrixXd build_regularization_term(MatrixXd M);

    /**
     * Build a superresolution cloud upsanmpled beta times from the saved depth maps
     */
    void build_superresolution(short beta);

    // Generic functor
    template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
    struct Functor
    {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }

    };

//    struct my_functor : lidarBoostEngine::Functor<double>
//    {
//    my_functor(void): Functor<double>(2,2) {}
//    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
//    {
//        // Implement y = 10*(x0+3)^2 + (x1-5)^2
//        fvec(0) = 10.0*pow(x(0)+3.0,2.0) +  pow(x(1)-5.0,2.0);
//        fvec(1) = 0;

//        return 0;
//    }
//    };

//    struct my_functor : lidarBoostEngine::Functor<double>
//    {
//    my_functor(void): Functor<double>(Eigen::Dynamic, Eigen::Dynamic) {}
//    int operator()(const MatrixXd &X, const MatrixXd &W, MatrixXd &T, MatrixXd &D, MatrixXd &E) const
//    {
////         Implement y = 10*(x0+3)^2 + (x1-5)^2
////        E(0) = X;
////        E(1) = 0;
//        E = W.cwiseProduct(T).cwiseProduct((D - X));

//        return 0;
//    }
//    };

//    struct my_functor : lidarBoostEngine::Functor<double>
//    {
//    my_functor(void): Functor<double>(Eigen::Dynamic, Eigen::Dynamic) {}
//    int operator()(const double &X, const double &W, double &T, double &D, double &E) const
//    {
////         Implement y = 10*(x0+3)^2 + (x1-5)^2
////        E(0) = X;
////        E(1) = 0;
//        E = W*T*(D - X);

//        return 0;
//    }
//    };

//    struct CostFunctor {
//       template <typename T>
//       bool operator()(const T* const x, T* residual) const {
//         residual[0] = T(10.0) - x[0];
//         return true;
//       }
//    };

    struct CostFunctor {
        CostFunctor(double w, double t, double d)
            : w_(w), t_(t), d_(d){}

       template <typename T>
       bool operator()(const T* const x,  T* residual) const {
//         residual[0] = T(10.0) - x[0];
//         residual[0] = w * t * (d - x[0]);

           residual[0] = w_*w_ * t_*t_ * ((d_ - x[0])*(d_ - x[0]));
         return true;
       }

    private:
        const double w_;
        const double t_;
        const double d_;
    };



private:

    //pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud;

    int m, n, sz, n_cloud, one_pcl_sz;
    std::vector< MatrixXd > Y, intensityMap;
//    SparseMatrix<double> W, T;
    MatrixXd W, T;

    short beta;






};

#endif // LIDARBOOSTENGINE_H

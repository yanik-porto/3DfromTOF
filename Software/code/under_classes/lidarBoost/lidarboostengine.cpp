#include "lidarboostengine.h"


lidarBoostEngine::lidarBoostEngine():
    one_pcl_sz(76800),
    m(320),
    n(240)
{

}

lidarBoostEngine::~lidarBoostEngine()
{

}

void lidarBoostEngine::set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    sz = cloud->size();
    n_cloud = sz/one_pcl_sz;
    std::cout << n_cloud << std::endl;

    Y = std::vector< MatrixXd >( n_cloud );
    Y = convert_pcl_to_eigen( cloud );
    intensityMap = get_intensity_images( cloud );
}


std::vector< MatrixXd > lidarBoostEngine::convert_pcl_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::vector< MatrixXd >eigMat( n_cloud );

//    typedef std::vector< MatrixXd >::iterator it_type;
    for( int i = 0; i < n_cloud; i++ )
    {
        eigMat[i] = MatrixXd( n, m );

        for (int j = 0; j < one_pcl_sz; j++ )
        {
            eigMat[i]( j/m, j%m ) = cloud->points[ j+i*one_pcl_sz ].z;
        }
    }

    return eigMat;
}

void lidarBoostEngine::set_selected_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int> numOfClouds)
{
    sz = cloud->size();
    n_cloud = sz/one_pcl_sz;
    std::cout << n_cloud << std::endl;

    int list_sz = numOfClouds.size();

    Y = std::vector< MatrixXd >( list_sz );
    std::vector< MatrixXd >eigMat( list_sz );

    int num = 0;
//    typedef std::vector< MatrixXd >::iterator it_type;
    for( int i = 0; i < list_sz; i++ )
    {
        eigMat[i] = MatrixXd( n, m );
        num = numOfClouds[i];

        for (int j = 0; j < one_pcl_sz; j++ )
        {
            eigMat[i]( j/m, j%m ) = cloud->points[ j+num*one_pcl_sz ].z;
        }
    }

    Y = eigMat;
}

void lidarBoostEngine::set_from_list_clouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_clouds)
{
    int sz_vec = list_clouds.size();
    sz = list_clouds[0]->size();
    n_cloud = sz/one_pcl_sz;
    std::vector< MatrixXd > vecClouds(n_cloud), vecIntensity(n_cloud);
    Y = std::vector< MatrixXd >( n_cloud*sz_vec );
    intensityMap = std::vector< MatrixXd >( n_cloud*sz_vec );
    for(int i = 0; i < sz_vec; i++)
    {
        vecClouds = convert_pcl_to_eigen(list_clouds[i]);
        vecIntensity = get_intensity_images(list_clouds[i]);
        for(int j = 0; j < n_cloud; j++)
        {
            Y[i*n_cloud + j] = vecClouds[j];
            intensityMap[i*n_cloud + j] = vecIntensity[j];
            std::cout << i*n_cloud + j << std::endl;
        }
    }
}

std::vector< MatrixXd > lidarBoostEngine::get_intensity_images(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::vector< MatrixXd >intensityEigMat( n_cloud );

//    typedef std::vector< MatrixXd >::iterator it_type;
    for( int i = 0; i < n_cloud; i++ )
    {
        intensityEigMat[i] = MatrixXd( n, m );

        for (int j = 0; j < one_pcl_sz; j++ )
        {
            intensityEigMat[i]( j/m, j%m ) = cloud->points[ j+i*one_pcl_sz ].intensity;
        }

    }

    return intensityEigMat;
}

template <typename Derived, typename Derived2 >
Derived lidarBoostEngine::conv2d(const MatrixBase<Derived>& I, const MatrixBase<Derived2> &kernel )
{
    Derived O = Derived::Zero(I.rows(),I.cols());


    typedef typename Derived::Scalar Scalar;
    typedef typename Derived2::Scalar Scalar2;

    int col=0,row=0;
    int KSizeX = kernel.rows();
    int KSizeY = kernel.cols();

    int limitRow = I.rows()-KSizeX;
    int limitCol = I.cols()-KSizeY;

    Derived2 block ;
    Scalar normalization = kernel.sum();
    if ( normalization < 1E-6 )
    {
        normalization=1;
    }
    for ( row = KSizeX; row < limitRow; row++ )
    {

      for ( col = KSizeY; col < limitCol; col++ )
      {
      Scalar b=(static_cast<Derived2>( I.block(row,col,KSizeX,KSizeY ) ).cwiseProduct(kernel)).sum();
      O.coeffRef(row,col) = b;
      }
    }

    return O/normalization;
}

template <typename Derived >
std::vector < Derived > lidarBoostEngine::lk_optical_flow( const MatrixBase<Derived>& I1, const MatrixBase<Derived> &I2, int win_sz)
{
    //Instantiate optical flow matrices
    std::vector < Derived > uv(2);

    //Create masks
    Matrix2d robX, robY, robT;
    robX << -1, 1,
            -1, 1;
    robY << -1, -1,
            1, 1;
    robT << -1, -1,
            -1, -1;

    Derived Ix, Iy, It, A, solutions, x_block, y_block, t_block;

    //Apply masks to images and average the result
    Ix = 0.5 * ( conv2d( I1, robX ) + conv2d( I2, robX ) );
    Iy = 0.5 * ( conv2d( I1, robY ) + conv2d( I2, robY ) );
    It = 0.5 * ( conv2d( I1, robT ) + conv2d( I2, robT ) );

    uv[0] = Derived::Zero( I1.rows(), I1.cols() );
    uv[1] = Derived::Zero( I1.rows(), I1.cols() );

    int hw = win_sz/2;

    for( int i = hw+1; i < I1.rows()-hw; i++ )
    {
        for ( int j = hw+1; j < I1.cols()-hw; j++ )
        {
            //Take a small block of window size in the filtered images
            x_block = Ix.block( i-hw, j-hw, win_sz, win_sz);
            y_block = Iy.block( i-hw, j-hw, win_sz, win_sz);
            t_block = It.block( i-hw, j-hw, win_sz, win_sz);

            //Convert these blocks in vectors
            Map<Derived> A1( x_block.data(), win_sz*win_sz, 1);
            Map<Derived> A2( y_block.data(), win_sz*win_sz, 1);
            Map<Derived> B( t_block.data(), win_sz*win_sz, 1);

            //Organize the vectors in a matrix
            A = Derived( win_sz*win_sz, 2 );
            A.block(0, 0, win_sz*win_sz, 1) = A1;
            A.block(0, 1, win_sz*win_sz, 1) = A2;

            //Solve the linear least square system
            solutions = (A.transpose() * A).ldlt().solve(A.transpose() * B);

            //Insert the solutions in the optical flow matrices
            uv[0](i, j) = solutions(0);
            uv[1](i, j) = solutions(1);

        }
    }

    return uv;

}

template <typename Derived>
Derived lidarBoostEngine::apply_optical_flow(const MatrixBase<Derived>& I, std::vector< Derived > uv)
{
    Derived moved_I(beta*n, beta*m);
//    W = SparseMatrix<double>( beta*n, beta*m );
//    W.reserve(VectorXi::Constant(n, m));
//    T = SparseMatrix<double>( beta*n, beta*m );

//    MatrixXi W( beta*n, beta*m ), T( beta*n, beta*m );
    W = MatrixXd( beta*n, beta*m );

    int new_i, new_j;

    for( int i = 0; i < I.rows(); i++ )
    {
        for( int j = 0; j < I.cols(); j++ )
        {
            new_i = round( beta * (i + uv[0](i, j)) );
            new_j = round( beta * (j + uv[1](i, j)) );
            if(new_i > 0 && new_i < beta*n && new_j > 0 && new_j < beta*m)
            {
                moved_I(new_i, new_j) = I(i ,j);
                W(new_i, new_j) = 1;
            }

        }
    }

    return moved_I;
}

MatrixXd lidarBoostEngine::check_unreliable_samples(MatrixXd map, double thresh)
{
    MatrixXd Mask( beta*n, beta*m );

    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < m; j++)
        {
            if(map(i, j) > thresh)
                Mask(beta*i, beta*j) = 1;
        }
    }

    return Mask;
}

template <typename Derived>
Derived lidarBoostEngine::nearest_neigh_upsampling(Derived M)
{
    Derived M_up(beta * n, beta * m);
    int min_dist(0), k(0);
    int dir = 0;

//    std::cout << M.rows() << " " << M.cols() << std::endl;

    for(int i = 0; i < M.rows(); i++)
    {
        for(int j = 0; j < M.cols(); j++)
        {
            if(M(i, j) == 0)
            {
                k = 0;
                min_dist = 0;

                //down
                while(M(i + k, j) == 0 && (i + k) > 0)
                    k--;

                min_dist = abs(k);
                dir = 0;

                //up
                k = 0;
                while(M(i + k, j) == 0 && (i + k) < beta*n-1)
                    k++;

                if(abs(k) < min_dist)
                {
                    min_dist = abs(k);
                    dir = 1;
                }

                //left
                k = 0;
                while(M(i, j + k) == 0 && (j + k) > 0)
                    k--;

                if(abs(k) < min_dist)
                {
                    min_dist = abs(k);
                    dir = 2;
                }

                //right
                k = 0;
                while(M(i, j + k) == 0 && (j + k) < beta*m-1)
                    k++;

                if(abs(k) < min_dist)
                {
                    min_dist = abs(k);
                    dir = 3;
                }

//                std::cout << "i = " << i << std::endl;
//                std::cout << "j = " << j << std::endl;
//                std::cout << "k = " << k << std::endl;
//                std::cout << "dir = " << dir << std::endl;

                //Insert closest value into the case
                switch(dir)
                {
                case 0: M_up(i, j) = M(i - min_dist, j);
                    break;

                case 1: M_up(i, j) = M(i + min_dist, j);
                    break;

                case 2: M_up(i, j) = M(i, j - min_dist);
                    break;

                case 3: M_up(i, j) = M(i, j + min_dist);
                    break;

                default:
                    break;

                }

            }
//            else
//                M_up(i, j) = M(i, j);
        }
    }

    return M_up;
}

//struct my_functor : lidarBoostEngine::Functor<double>
//{
//my_functor(void): Functor<double>(2,2) {}
//int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
//{
//    // Implement y = 10*(x0+3)^2 + (x1-5)^210
//    fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
//    fvec(1) = 0;

//    return 0;
//}
//};

void lidarBoostEngine::build_regularization_term(MatrixXd M)
{

}

void lidarBoostEngine::build_superresolution(short coeff)
{
    std::cout<< "Num of clouds : " << Y.size() << std::endl;

//    std::cout << Y[0] << std::endl;
    beta = coeff;
    std::vector < MatrixXd > optflow = lk_optical_flow( Y[2], Y[4], 10 );
    MatrixXd D( beta*n, beta*m ); //, X( beta*n, beta*m );
//    SparseMatrix<double> W( beta*n, beta*m ), T( beta*n, beta*m );

    D = apply_optical_flow(Y[2], optflow);
    T = check_unreliable_samples(intensityMap[2], 0.0001);

    MatrixXd up_D = nearest_neigh_upsampling(D);

////    Display and Debug
    cv::Mat M(n, m, CV_32FC1);
//    MatrixXd diff1(n, m);
//    diff1 = MatrixXd::Ones(n, m) - Y[0];
    cv::eigen2cv(Y[2], M);

    cv::Mat M1(n, m, CV_32FC1);
    cv::eigen2cv(Y[4], M1);

//    MatrixXd diff(beta*n, beta*m);
//    diff = MatrixXd::Ones(beta*n, beta*m) - up_D;
    cv::Mat M2(beta*n, beta*m, CV_32FC1);
    cv::eigen2cv(up_D, M2);

    cv::namedWindow("check", cv::WINDOW_AUTOSIZE );
    cv::imshow("check", M);

    cv::namedWindow("check1", cv::WINDOW_AUTOSIZE );
    cv::imshow("check1", M1);

    cv::namedWindow("check2", cv::WINDOW_AUTOSIZE );
    cv::imshow("check2", M2);

////  Solve example equation with eigen
//    Eigen::VectorXd x(2);
//    x(0) = 10.0;
//    x(1) = 25.0;
//    std::cout << "x: " << x << std::endl;

//    my_functor functor;
//    Eigen::NumericalDiff<my_functor> numDiff(functor);
//    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,double> lm(numDiff);
//    lm.parameters.maxfev = 2000;
//    lm.parameters.xtol = 1.0e-10;
//    std::cout << lm.parameters.maxfev << std::endl;

//    int ret = lm.minimize(x);
//    std::cout << lm.iter << std::endl;
//    std::cout << ret << std::endl;

//    std::cout << "x that minimizes the function: " << x << std::endl;

//////    Try to solve lidarboost with Eigen
//      my_functor functor;
//      Eigen::NumericalDiff<my_functor> numDiff(functor);
//      Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,double> lm(numDiff);
//      lm.parameters.maxfev = 2000;
//      lm.parameters.xtol = 1.0e-10;
//      std::cout << lm.parameters.maxfev << std::endl;

//    VectorXd val(2);
//    for(int i = 0; i < X.rows(); i++)
//    {
//        for(int j = 0; j < X.cols(); j++)
//        {
//            val = X(i, j);
//            int ret = lm.minimize(val);
//        }
//    }

//    std::cout << lm.iter << std::endl;
//    std::cout << ret << std::endl;

//    std::cout << "x that minimizes the function: " << X << std::endl;

////  Solve example using ceres

//         The variable to solve for with its initial value.
//        double initial_x = 5.0;
//        double x = initial_x;

        MatrixXd X(beta*n, beta*m);// init_X(beta*n, beta*m);
//        X = MatrixXd::Zero(beta*n,beta*m);
        X = up_D;
//        MatrixXd init_X( beta*n, beta*m );
//        init_X = X;
//        int M[2][2], M2[2][2];
//        M[0][0] = 5;
//        M[1][0] = 10;
//        M[0][1] = 20;
//        M[1][1] = 30;

//        M2 = *M;

        // Build the problem.
        Problem problem;

        // Set up the only cost function (also known as residual). This uses
        // auto-differentiation to obtain the derivative (jacobian).

        double val, w, t, d;

        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        Solver::Summary summary;

        for(int i = 0; i < X.rows(); i++)
        {
            for(int j = 0; j < X.cols(); j++)
            {

                val = X(i, j);
                w = W(i, j);
                t = T(i, j);
                d = up_D(i, j);

                std::cout << "i = " << i << "; j = " << j << std::endl;
                std::cout << "w = " << w << "; t = " << t << "; d = " << d << std::endl;
                CostFunction* cost_function =
                    new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor(w, t, d));

                problem.AddResidualBlock(cost_function, NULL, &val);
                // Run the solver
                Solve(options, &problem, &summary);
                X(i, j) = val;
            }
        }




        std::cout << summary.BriefReport() << "\n";
//        std::cout << "x : " << init_X
//                  << " -> " << X << "\n";

        cv::Mat M3(beta*n, beta*m, CV_32FC1);
        cv::eigen2cv(X, M3);
        cv::namedWindow("check3", cv::WINDOW_AUTOSIZE );
        cv::imshow("check3", M3);
}



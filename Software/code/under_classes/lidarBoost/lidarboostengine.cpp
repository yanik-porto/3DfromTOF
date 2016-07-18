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
    //ptcloud = cloud;
    sz = cloud->size();
    n_cloud = sz/one_pcl_sz;
    std::cout << n_cloud << std::endl;
    //Y = MatrixXd( n, m );
    Y = std::vector< MatrixXd >( n_cloud );
    Y = convert_pcl_to_eigen( cloud );
    //std::cout<<Y[0].rows() << " " << Y[0].cols() << std::endl;
}

std::vector< MatrixXd > lidarBoostEngine::convert_pcl_to_eigen(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::vector< MatrixXd >eigMat( n_cloud );

    typedef std::vector< MatrixXd >::iterator it_type;
    for( int i = 0; i < n_cloud; i++ )
    {
        eigMat[i] = MatrixXd( n, m );

        for (int j = 0; j < one_pcl_sz; j++ )
        {
//            std::cout << i << std::endl;
            eigMat[i]( j/m, j%m ) = cloud->points[ j+i*one_pcl_sz ].z;
//            std::cout << cloud->points[ j+i*one_pcl_sz ].z << std::endl;
//            std::cout << j/320 << " " << j%320 << std::endl;
        }
    }

    return eigMat;
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
    std::vector < Derived > uv(2);
    //Create masks
    Matrix2d robX, robY, robT;
    robX << -1, 1,
            -1, 1;
    robY << -1, -1,
            1, 1;
    robT << -1, -1,
            -1, -1;

    //Apply masks to images and average the result
    Derived Ix, Iy, It;

    Ix = 0.5 * ( conv2d( I1, robX ) + conv2d( I2, robX ) );
    Iy = 0.5 * ( conv2d( I1, robY ) + conv2d( I2, robY ) );
    It = 0.5 * ( conv2d( I1, robT ) + conv2d( I2, robT ) );

    uv[0] = Derived::Zero( I1.rows(), I1.cols() );
    uv[1] = Derived::Zero( I1.rows(), I1.cols() );

    int hw = win_sz/2;

//    for( int i = hw+1; i <= I1.rows()-hw; i++ )
//    {
//        for ( int j = hw+1; j <= I1.cols()-hw; j++ )
//        {

//        }
//    }

    Map<RowVectorXd> Vx( Ix.data(), Ix.size() );
//    std::cout << "Vx : " << Vx[76700] << std::endl;


    return uv;

}

void lidarBoostEngine::test_apply_lk()
{
    std::vector < MatrixXd > optflow = lk_optical_flow( Y[0], Y[1], 5 );
}

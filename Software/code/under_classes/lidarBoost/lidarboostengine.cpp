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


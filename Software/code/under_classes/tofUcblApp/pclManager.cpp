#include "pclManager.h"


pclManager::pclManager():
    nCloud(0)
{
}


pclManager::~pclManager()
{
}

bool pclManager::set_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud)
{
	//Set the cloud with an already existing pointCloud
	cloud = ptcloud;

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from point cloud"
		<< std::endl;
	return true;
}

bool pclManager::set_cloud_from_pcd(const std::string &filename)
{
	//Set the cloud from a .pcd file
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPcd(new pcl::PointCloud<pcl::PointXYZI>);
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloudPcd) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}
	std::cout << "Loaded "
		<< cloudPcd->width * cloudPcd->height
		<< " data points from .pcd "
		<< std::endl;

	cloud = cloudPcd;
	return true;
}

bool pclManager::save2pcd(const std::string &filename)
{
	//Save the cloud in a .pcd file
	pcl::io::savePCDFileASCII(filename, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;
	return true;
}

void pclManager::visualizePcl()
{
	//Display the cloud
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);

    while (!viewer.wasStopped())
    {
    }
}

void pclManager::init_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> pclvisu)
{
    //viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer = pclvisu;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pclManager::xyz_from_xyzi(pcl::PointCloud<pcl::PointXYZI>::ConstPtr intensityCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
    xyzCloud->width = intensityCloud->width;
    xyzCloud->height = intensityCloud->height;
    xyzCloud->is_dense = false;
    xyzCloud->points.resize(xyzCloud->width * xyzCloud->height);

    pcl::PointXYZ temp;
    for(int i=0; i<intensityCloud->size(); i++)
    {
        temp.x = intensityCloud->at(i).x;
        temp.y = intensityCloud->at(i).y;
        temp.z = intensityCloud->at(i).z;
        xyzCloud->push_back(temp);
    }

    return xyzCloud;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclManager::simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr intensityCloud)
{
    nCloud++;
    QString cloudName = "cloud" + QString::number(nCloud);

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->resetCamera();
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZI> (intensityCloud, cloudName.toStdString());
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName.toStdString());
  viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
  viewer->resetCamera();
  return (viewer);
}

//pcl::PointCloud<pcl::PointXYZI>::Ptr pclManager::filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud)
//{
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredx(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredy(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredz(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredi(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PassThrough<pcl::PointXYZI> pass;
//    //pcl::PassThrough<pcl::PointXYZ> pass;
//    /*pcl::VoxelGrid<pcl::PointXYZI> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.02f, 0.02f, 0.02f);
//    sor.filter(*cloud_filtered);
//    cout << "down done\n";*/
//    pass.setInputCloud(ptcloud);
//    pass.setFilterFieldName("intensity");
//    pass.setFilterLimits(0.007, 1);
//    pass.filter(*cloud_filteredi);
//    cout << "i done\n";
//    //filtered = cloud;
//    pass.setInputCloud(cloud_filteredi);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0, 2);
//    cout << "load done\n";
//    //pass.setFilterLimitsNegative (true);
//    pass.filter(*cloud_filteredz);
//    cout << "z done\n";
//    //pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud(cloud_filteredz);
//    pass.setFilterFieldName("y");
//    pass.setFilterLimits(0, 1);
//    //pass.setFilterLimitsNegative (true);
//    pass.filter(*cloud_filteredy);
//    cout << "y done\n";
//    //pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud(cloud_filteredy);
//    pass.setFilterFieldName("x");
//    pass.setFilterLimits(-2, 2);
//    //pass.setFilterLimitsNegative (true);
//    pass.filter(*cloud_filteredx);
//    cout << "x done\n";

//    return cloud_filteredx;
//}

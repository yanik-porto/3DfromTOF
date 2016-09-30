#include "pclManager.h"

//***********************************************************************************************************************************************
// * Destructors and Constructors
//***********************************************************************************************************************************************


pclManager::pclManager():
    nCloud(0)
{
}

pclManager::~pclManager()
{
}


//***********************************************************************************************************************************************
// * Accessors and Mutators
//***********************************************************************************************************************************************

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

pcl::PointCloud<pcl::PointXYZI>::Ptr pclManager::get_cloud()
{
    return cloud;
}

//***********************************************************************************************************************************************
// * Functions
//***********************************************************************************************************************************************

bool pclManager::save2pcd(const std::string &filename)
{
	//Save the cloud in a .pcd file
	pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " << filename << std::endl;
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
    viewer = pclvisu;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pclManager::xyz_from_xyzi(pcl::PointCloud<pcl::PointXYZI>::ConstPtr intensityCloud)
{
    //Transform a cloud with intensity information to a cloud without intensity
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
    xyzCloud->width = intensityCloud->width;
    xyzCloud->height = intensityCloud->height;
    xyzCloud->is_dense = false;
    xyzCloud->points.resize(xyzCloud->width * xyzCloud->height);

//    pcl::PointXYZ temp;
    for(int i=0; i<intensityCloud->size(); i++)
    {
        xyzCloud->points[i].x = intensityCloud->at(i).x;
        xyzCloud->points[i].y = intensityCloud->at(i).y;
        xyzCloud->points[i].z = intensityCloud->at(i).z;
//        temp.x = intensityCloud->at(i).x;
//        temp.y = intensityCloud->at(i).y;
//        temp.z = intensityCloud->at(i).z;
//        xyzCloud->push_back(temp);
    }

    return xyzCloud;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclManager::simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr intensityCloud)
{
    //Set the name of the new cloud
    nCloud++;
    QString cloudName = "cloud" + QString::number(nCloud);

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZI> (intensityCloud, cloudName.toStdString());
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName.toStdString());
  viewer->addCoordinateSystem (0.1);
  //viewer->initCameraParameters ();
  viewer->resetCamera();
  return (viewer);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr pclManager::filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud, const float &threshX, const float &threshY, const float &threshZ, const float &threshI)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

    //Filter the input cloud with the given values for the 3 coordinates and the intensity
    for (size_t i = 0; i < ptcloud->points.size(); ++i)
    {
         if ((ptcloud->points[i].intensity>threshI)&&(ptcloud->points[i].z<(threshZ))&&(ptcloud->points[i].y<threshY) && (ptcloud->points[i].y>-threshY) && (ptcloud->points[i].x<threshX) && (ptcloud->points[i].x>-threshX))
            filtered->push_back(ptcloud->at(i));
    }

    std::cout << "Filtering " << ptcloud->size() <<" points results in " << filtered->size() << "points" << std::endl;

    return filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pclManager::set_multiple_clouds_from_pcd(std::vector<std::string> list_filenames)
{
    int sz = list_filenames.size();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> list_Clouds(sz);

    for(int i = 0; i < sz; i++)
    {
        //Set the cloud from a .pcd file
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPcd(new pcl::PointCloud<pcl::PointXYZI>);

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(list_filenames[i], *cloudPcd) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        std::cout << "Loaded "
            << cloudPcd->width * cloudPcd->height
            << " data points from .pcd "
            << std::endl;

        list_Clouds[i] = cloudPcd;
    }

    return list_Clouds;
}

#define _CRT_SECURE_NO_WARNINGS
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>

int
main(int argc, char** argv)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D filter"));
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("10frames.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredx(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredy(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredz(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredi(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> pass;
	//pcl::PassThrough<pcl::PointXYZ> pass;
	/*pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.02f, 0.02f, 0.02f);
	sor.filter(*cloud_filtered);
	cout << "down done\n";*/
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("intensity");
	pass.setFilterLimits(0.007, 1);
	pass.filter(*cloud_filteredi);
	cout << "i done\n";
	//filtered = cloud;
	pass.setInputCloud(cloud_filteredi);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 2);
	cout << "load done\n";
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filteredz);
	cout << "z done\n";
    //pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filteredz);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, 1);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filteredy);
	cout << "y done\n";
    //pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filteredy);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-2, 2);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filteredx);
	cout << "x done\n";
	
	
	pcl::PointXYZ temp;
	for (int i = 0; i < cloud->width;i++) {
		temp.x = cloud_filteredx->at(i).x;
		temp.y = cloud_filteredx->at(i).y;
		temp.z = cloud_filteredx->at(i).z;
		filtered->push_back(temp);
		//cout << i<<"\n";
	}
	cout << "f done\n";

	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*filtered, min, max);

	cout << min <<"\n";
	cout << max <<"\n";

	////cloud = cloud_filtered;
	//cout << "fliter done\n";
	////Normal estimation*
	////pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	////pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(filtered);
	//
	////n.setInputCloud(filtered);
	////n.setSearchMethod(tree);
	////n.setKSearch(20);
	////n.compute(*normals);
	//////* normals should not contain the point normals + surface curvatures

	////// concatenate the xyz and normal fields*
	////pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	////pcl::concatenateFields(*filtered, *normals, *cloud_with_normals);
	//////* cloud_with_normals = cloud + normals
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//pcl::PointCloud<pcl :: PointNormal >::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal>());
	//pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
	//
 //   mls.setInputCloud(filtered);
	//mls.setComputeNormals(true);
	//mls.setPolynomialFit(true);
	//mls.setSearchMethod(tree1);
	//mls.setSearchRadius(0.05);
	//mls.process(*mls_normals);
	////pcl::io::savePCDFile("bun0-mls.pcd", *mls_normals);
	//// Create search tree*
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(mls_normals);

	//// Initialize objects
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//boost::shared_ptr<pcl::PolygonMesh> triangles(new pcl::PolygonMesh);


	////// Set the maximum distance between connected points (maximum edge length)
	//gp3.setSearchRadius(0.05);

	//// Set typical values for the parameters
	//gp3.setMu(2.5);
	//gp3.setMaximumNearestNeighbors(3000);
	//gp3.setMaximumSurfaceAngle(M_PI / 9); // 45 degrees
	//gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	//gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	//gp3.setNormalConsistency(true);

	//// Get result
	//gp3.setInputCloud(mls_normals);
	//gp3.setSearchMethod(tree2);
	//gp3.reconstruct(*triangles);
	//std::vector<int> parts = gp3.getPartIDs();
	//std::vector<int> states = gp3.getPointStates();
	//
	//pcl::Poisson<pcl::PointNormal> poisson;
	//poisson.setDepth(7);
	//poisson.setInputCloud(mls_normals);
	//poisson.reconstruct(*triangles);*/
	//// Additional vertex information


	//pcl::PolygonMesh output;
 //   pcl::MeshSmoothingLaplacianVTK vtk;
	//vtk.setInputMesh(triangles);
	//vtk.setNumIter(2000);
	//vtk.setConvergence(0.0001);
	//vtk.setRelaxationFactor(0.001);
	//vtk.setFeatureEdgeSmoothing(false);
 //   vtk.setFeatureAngle(M_PI / 5);
	//vtk.setBoundarySmoothing(false);
	//vtk.process(output);

	viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPolygonMesh(*triangles);
	viewer->addPointCloud(filtered);
	viewer->addCube(min[0], max[0], min[1], max[1], min[2], max[2]);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->addCoordinateSystem(0.2);
	viewer->initCameraParameters();

	//viewer1->setBackgroundColor(0, 0, 0);
	//viewer1->addPolygonMesh(output);
	//////viewer1->addPointCloud(cloud);
	//////viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	//viewer1->addCoordinateSystem(0.2);
	//viewer1->initCameraParameters();

	/*while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	std::cout << "filtered\n";
	//pcl::io::saveVTKFile("mesh.vtk", triangles);
	// Finish
	return (0);
}
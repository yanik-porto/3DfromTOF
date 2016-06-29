#include "CameraManager.h"

CameraManager::CameraManager()
{
}


CameraManager::~CameraManager()
{
}

void CameraManager::get_pts(std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer ptr)
{
	ptr = intPts;
}

bool CameraManager::capture(short nShots)
{
	//Scan all connected devices
	Voxel::CameraSystem sys;
	Voxel::Vector<Voxel::DevicePtr> listDev = sys.scan();

	//Go through all the devices and display their ID
	int i = 0;
	for (auto &d : listDev)
	{
		i++;
		std::cout << i << std::endl;
		std::cout << d->id() << std::endl;
	}

	//Load and initialize the first detected camera
	Voxel::DepthCameraPtr currentCam = sys.connect(listDev[0]);

	//std::cout << listDev[0]->id() << std::endl;

	if (!currentCam)
	{
		std::cerr << "Could not load depth camera for device " << listDev[0]->id() << std::endl;
		return false;
	}

	if (!currentCam->isInitialized())
	{
		std::cerr << "Depth camera not initialized for device " << listDev[0]->id() << std::endl;
		return false;
	}

	//Initialize variables
	int count = 0;
	Voxel::TimeStampType lastTimeStamp = 0;
	int32_t frameCount = nShots - 1;
	
	//Capture point cloud, called whenever the cam is started
	currentCam->registerCallback(Voxel::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c)
	{
		//Store the data in a frame
		const Voxel::XYZIPointCloudFrame *d = dynamic_cast<const Voxel::XYZIPointCloudFrame *>(&frame);

		//Check if well captured
		if (!d)
		{
			std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
			return;
		}

		//Display that a frame has been captured
		std::cout << "Capture frame " << d->id << "@" << d->timestamp;

		//If not the first shot, display the frequency
		if (lastTimeStamp != 0)
			std::cout << " (" << 1E6 / (d->timestamp - lastTimeStamp) << " fps)" << std::endl;

		//record when it has been recorded
		lastTimeStamp = d->timestamp;

		sz_cloud = d->size();
		intPts = d->points.data();

		//Stop when you have recorded enough frame
		if (count >= frameCount)
			dc.stop();

		count++;
	});

	if (currentCam->start())
	{
		Voxel::FrameRate r;
		if (currentCam->getFrameRate(r))
			Voxel::logger(Voxel::LOG_INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
		currentCam->wait();
	}
	else
		std::cerr << "Could not start the depth camera " << currentCam->id() << std::endl;


	return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CameraManager::convert2pcl()
{
	for (int i = 0; i < 10; i++)
	{
		std::cout << "    " << intPts[i].x
			<< " " << intPts[i].y
			<< " " << intPts[i].z << std::endl;
	}

	cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	cloud->width = 320;
	cloud->height = 240;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = intPts[i].x;
		cloud->points[i].y = intPts[i].y;
		cloud->points[i].z = intPts[i].z;
		cloud->points[i].intensity = intPts[i].i;
	}

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;

	for (size_t i = 0; i < 10; ++i)
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << std::endl;

	return cloud;
}
#include "CameraManager.h"

//***********************************************************************************************************************************************
// * Destructors and Constructors
//***********************************************************************************************************************************************

CameraManager::CameraManager():
    numOfShots(1)
{

}

CameraManager::CameraManager(const short &n)
{
    numOfShots = n;
}

CameraManager::~CameraManager()
{
}

//***********************************************************************************************************************************************
// * Accessors and Mutators
//***********************************************************************************************************************************************

void CameraManager::get_pts(std::vector<std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer> ptr)
{
    //Will not work (const_pointer)
	ptr = intPts;
}

void CameraManager::set_numOfShots(const short &n)
{
    numOfShots = n;
}


//***********************************************************************************************************************************************
// * Get infos
//***********************************************************************************************************************************************

std::vector<std::string> CameraManager::get_devices_name()
{
    //Scan all connected devices
    Voxel::CameraSystem sys;
    Voxel::Vector<Voxel::DevicePtr> listDev = sys.scan();

    //Initialize the list of devices
    std::vector<std::string> listNames(listDev.size());

    //Go through all the devices and get their ID
    int i = 0;
    for (auto &d : listDev)
    {
        listNames[i] = static_cast<std::string>(d->id());
        i++;
    }

    return listNames;
}

std::map<std::string, int> CameraManager::get_profiles_name()
{
    std::map<std::string, int> outMap;

    //Scan all connected devices
    Voxel::CameraSystem sys;
    Voxel::Vector<Voxel::DevicePtr> listDev = sys.scan();

    //Load and initialize the first detected camera
   Voxel::DepthCameraPtr Cam = sys.connect(listDev[0]);

   Voxel::Map<int,Voxel::String> profilMap = Cam->getCameraProfileNames();

   std::cout << profilMap.size() << std::endl;
   typedef Voxel::Map<int,Voxel::String>::iterator it_type;

   for(it_type iter = profilMap.begin(); iter != profilMap.end(); iter++)
   {
       std::cout << iter->first << ": " << iter->second << std::endl;
       outMap[iter->second] = iter->first;
   }



   return outMap;
}

std::map<std::string, std::string> CameraManager::get_param_descr(const short &num_device)
{
    //Initialize the vector of string that will contain all the parameters
//    std::vector<std::string> paramVec;

    std::map<std::string, std::string> descrMap;

    //Scan all connected devices
    Voxel::CameraSystem sys;
    Voxel::Vector<Voxel::DevicePtr> listDev = sys.scan();

    //Load and initialize the first detected camera
    Voxel::DepthCameraPtr currentCam = sys.connect(listDev[num_device]);

//    Voxel::FrameRate frate;
//    currentCam->getFrameRate(frate);

    Voxel::Map< Voxel::String, Voxel::ParameterPtr > paramMap = currentCam->getParameters();

    typedef Voxel::Map< Voxel::String, Voxel::ParameterPtr >::iterator it_type;
    for( it_type iter = paramMap.begin(); iter != paramMap.end(); iter++ )
    {
        //std::cout << iter->first << ": " << iter->second->name() << std::endl;
        descrMap[iter->first] = iter->second->description();
    }

    return descrMap;
//    int coeff_illum;
//    currentCam->get("coeff_illum",coeff_illum);
//    std::cout << "coeff_illum value = " << coeff_illum << std::endl;




}

//***********************************************************************************************************************************************
// * Functions
//***********************************************************************************************************************************************


pcl::PointCloud<pcl::PointXYZI>::Ptr CameraManager::capture(const short &num_device, const int &id_calib)
{

	//Initialize the vector of points
    intPts = std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer >(numOfShots);

	//Scan all connected devices
	Voxel::CameraSystem sys;
	Voxel::Vector<Voxel::DevicePtr> listDev = sys.scan();

	//Load and initialize the first detected camera
   Voxel::DepthCameraPtr currentCam = sys.connect(listDev[num_device]);

   //Set the calibration mode
    currentCam->setCameraProfile(id_calib);

	if (!currentCam)
	{
        std::cerr << "Could not load depth camera for device " << currentCam->id() << std::endl;
		return false;
	}

	if (!currentCam->isInitialized())
	{
        std::cerr << "Depth camera not initialized for device " << currentCam->id() << std::endl;
		return false;
	}

	//Initialize variables
	int count = 0;
	Voxel::TimeStampType lastTimeStamp = 0;
    int32_t frameCount = numOfShots - 1;
	
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
		intPts[count] = d->points.data();

		//Stop when you have recorded enough frame
		if (count >= frameCount)
			dc.stop();

		count++;
	});

	if (currentCam->start())
	{
		//Display framerate
		Voxel::FrameRate r;
		if (currentCam->getFrameRate(r))
			Voxel::logger(Voxel::LOG_INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
		currentCam->wait();
	}
	else
		std::cerr << "Could not start the depth camera " << currentCam->id() << std::endl;

	//Convert from vector to pointCloud
	cloud = convert2pcl(intPts);

	std::cout << "Saved "
		<< cloud->width * cloud->height
        << " data points from the camera " << currentCam->id()
		<< std::endl;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CameraManager::convert2pcl(std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer > vecPts)
{
	//Convert from vector to point cloud

	//Initialize the cloud with the size information
	cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	cloud->width = (sz_cloud)*(numOfShots);
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	//Copy every coordinates
	for (int j = 0; j <= numOfShots-1; j++)
	{
		for (int i = 0; i < sz_cloud; i++)
		{
			cloud->points[i + j*sz_cloud].x = intPts[j][i].x;
			cloud->points[i + j*sz_cloud].y = intPts[j][i].y;
			cloud->points[i + j*sz_cloud].z = intPts[j][i].z;
			cloud->points[i + j*sz_cloud].intensity = intPts[j][i].i;
		}
	}

	return cloud;
}

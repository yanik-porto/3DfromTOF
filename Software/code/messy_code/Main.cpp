#include <CameraSystem.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main()
{
	int32_t frameCount = 2;

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
		return -1;
	}

	if (!currentCam->isInitialized())
	{
		std::cerr << "Depth camera not initialized for device " << listDev[0]->id() << std::endl;
		return -1;
	}

	//Initialize variables
	int count = 0;
	Voxel::TimeStampType lastTimeStamp = 0;

	//pcl::PointXYZI o;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	std::vector< std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer> intPts(frameCount+1);
	int sz_cloud;

		//Capture point cloud, called whenever the cam is started
		currentCam->registerCallback(Voxel::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType c)
		{
			//for (int i = 0; i < 1; i++)
			//{

				std::cout << count << std::endl;

				const Voxel::XYZIPointCloudFrame *d = dynamic_cast<const Voxel::XYZIPointCloudFrame *>(&frame);
				
				if (!d)
				{
					std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
					return;
				}

				std::cout << "Capture frame " << d->id << "@" << d->timestamp;

				if (lastTimeStamp != 0)
					std::cout << " (" << 1E6 / (d->timestamp - lastTimeStamp) << " fps)";

				std::cout << std::endl;

				//f.write((char *)&d->id, sizeof(d->id));
				//f.write((char *)&d->timestamp, sizeof(d->timestamp));

				lastTimeStamp = d->timestamp;

				/*f.write((char *)d->points.data(), sizeof(Voxel::IntensityPoint)*d->points.size());*/
				//f.write((char *)d->points.data(), d->points.size());
				//f << std::dec << d->points.data();

				//std::vector<Voxel::IntensityPoint, std::allocator<Voxel::IntensityPoint>>::const_pointer intPts = d->points.data();
				intPts[count] = d->points.data();

				//std::cout << intPts->x << intPts->y << intPts->z << intPts->i << std::endl;
				//std::cout << intPts[1].x << intPts[1].y << intPts[1].z << intPts[1].i << std::endl;
				//sz_cloud = d->size();

				//o.x = intPts->x;
				//o.y = intPts->y;
				//o.z = intPts->z;
				//o.intensity = intPts->i;
				//std::cout << d->points.data() << std::endl;

				if (count >= frameCount)
					dc.stop();

				count++;
			//}
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

	cloud.width = (320*240)*(frameCount+1);
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	
	sz_cloud = 320 * 240;

	for (int j = 0; j <= frameCount; j++)
	{
		for (int i = 0; i < sz_cloud; i++)
		{
			cloud.points[i + j*sz_cloud].x = intPts[j][i].x;
			cloud.points[i + j*sz_cloud].y = intPts[j][i].y;
			cloud.points[i + j*sz_cloud].z = intPts[j][i].z;
			cloud.points[i + j*sz_cloud].intensity = intPts[j][i].i;
		}
	}

	pcl::io::savePCDFileASCII("3frames.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
	
	
	system("pause");

}

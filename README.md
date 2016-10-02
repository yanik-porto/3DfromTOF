# 3DfromTOF
3D Reconstruction from a merging of clouds captured by a TOF camera mounted on a turntable. 

##Software
Tutorials for :
- the installation of the voxelsdk, provided for the camera, as well as Qt and pcl for Linux, and Visual Studio and pcl for Windows. 
- the usage of our application, accessing the camera and processing data

###Code
####under_classes (C++)
* tofUcblApp = Application embedding capture of the data from a ToF camera, Visualization of the point clouds, Filtering option, Control of an Arduino board connected to a motor, Merging of the clouds with ICP, Saving and Importing .pcd files. 
* Lidarboost = Implementation of the first steps of the LidarBoost paper from Sebastian Schuon et al. 

####messy_code (C++)
* draft of the code

####matlab_code
* matlab_lidarboost = Implementation of the first steps of the LidarBoost paper from Sebastian Schuon et al. 
* matlab_laserscanner = 3D scanning with a laser, a rgb camera and a turntable

## Hardware
Documentation relative to the Tintin ToF camera from T.I. (Texas Instrument)

##Dataset
Some acquisitions with the ToF camera and a turntable. Object rotated of small angles between each cloud

##Documentation
Papers for improving the clouds taken by the ToF camera


clear all;
close all;
clc;

%% Read data
cloud1 = loadpcd('sugar1.pcd');
cloud2 = loadpcd('sugar2.pcd');
cloud3 = loadpcd('sugar3.pcd');
cloud4 = loadpcd('sugar4.pcd');
cloud5 = loadpcd('sugar5.pcd');
% pcshow(cloud2(1:3,:)','VerticalAxis','Y');

%% Map clouds to depth images

depth_img{1} = depth_map_from_cloud(cloud1, [240,320]);
depth_img{2} = depth_map_from_cloud(cloud2, [240,320]);
depth_img{3} = depth_map_from_cloud(cloud3, [240,320]);
depth_img{4} = depth_map_from_cloud(cloud4, [240,320]);
depth_img{5} = depth_map_from_cloud(cloud5, [240,320]);


%% Display
imshow(depth_img{1},[]);


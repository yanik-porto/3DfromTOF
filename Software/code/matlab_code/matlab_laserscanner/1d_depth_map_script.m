clear all
close all
vidobj = videoinput('winvideo', 1, 'RGB24_800x600'); %Setup Camera
aMega = arduino('COM4');
writeDigitalPin(aMega,'D22',1);
%% Model Variables
f = 919; % Focal length
cx = 495; % Optical center x
cy = 335; % Optical center y
b0 = 140; % Laser distance from camera
C0 =65.9534; % Angle of laser
Y_mltply = 0.234;%Y axis scaling factor
pic1 = getsnapshot(vidobj);
imgSize = size(pic1);
%% Variables for the code
LaserPts = []; % Camera Coordinates of Laser line 
CamCoordPts = []; % List of X; Y; Z; points
BW = NewFilter(vidobj,imgSize,aMega,'D22');%Filter laser line
imshow(BW) %display the image
%% Convert Single Line to camera coordinates 
counter = 1;
for i = 1:imgSize(1)
    for i1 = 1:imgSize(2)
        testpt = BW(i,i1);
        if testpt == 1 % if point on Y
        LaserPts(1,counter) = i1-cx; % add to LaserPts list in Camera Coordinates
        LaserPts(2,counter) = cy-i;
        counter = counter+1;
        end
    end
end
%% Determine World Coordinates for each Y
sizeDisx = size(LaserPts);
for i = 1:sizeDisx(2)
    delta_px_x0 = LaserPts(1,i);
    A0 = 90+atand(delta_px_x0/f);
    B0 = 180 - (A0+C0);
    a0 = (sind(A0)*b0)/sind(B0);
    h1 = a0*cosd(atand(delta_px_x0/f));
    CamCoordPts(3,i)=h1;
    CamCoordPts(2,i) = Y_mltply*LaserPts(2,i);
end
CamCoordPts(1,:) = 0;
%% Set X = 0
transCW = transpose(CamCoordPts); % Transpose the list
pointCloud = pointCloud(transCW);
figure;pcshow(pcdenoise(pointCloud)) % View the pointcloud
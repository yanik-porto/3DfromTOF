instrreset
clear all
close all
imaqreset
vidobj = videoinput('winvideo', 1, 'RGB24_800x600'); %Setup Camera
aMega  = serial('COM3');
fopen(aMega);
pause_time = 0.005;
LaserPin = 'D22';
Ang_of_onerot = 1.6822;
onerot_loops = 2;

pic1 = getsnapshot(vidobj);
pic1 = pic1(:,:,1)';
imgSize = size(pic1);
%% Model Variables
f = 860.2846; % Focal length
cx = 422.240197188849; % Optical center x
cy = 294.510595146250; % Optical center y
b0 = 165; % Laser distance from camera
C0 = 53; % Angle of laser
Y_mltply = 0.431;%Y axis scaling factor
y_trans = -20; % Transform value for Y
z_trans = -315; % Transform value for Z
Ang_per_loop = Ang_of_onerot*onerot_loops;
Main_loops = round(360/Ang_per_loop);
%% Set Constants
MaxPoints = imgSize(1);
cordW = zeros(MaxPoints,3,Main_loops);
%% Start Loop
for Tm = 1:Main_loops    
LaserPts = []; % Camera Coordinates of Laser line 
CamCoordPts = []; % List of X; Y; Z; points
BW = newfilter(vidobj,imgSize,aMega);
BW =flipud(BW');
imshow(BW);
%% Convert Single Line to camera coordinates 
counter = 1;
for i = 1:imgSize(1)
    for i1 = 1:imgSize(2)
        testpt = BW(i,i1);
        if testpt == 1 % if point on Y
        LaserPts(1,counter) = i1-cy; % add to LaserPts list in Camera Coordinates
        LaserPts(2,counter) = cx-i;
        counter = counter+1;
break;
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


    %% Transform CamCoords to World Coordinate
    Mext = [ 1 0 0 0;
        0 1 0 y_trans;
        0 0 1 z_trans;
        0 0 0 1;];
    
    thetaW=(Tm-1)*3.3645; % New angle
    Trans = [cosd(thetaW),0,sind(thetaW),0;0,1,0,0;-sind(thetaW),0,cosd(thetaW),0;0,0,0,1]; %Translation Matrix

    CordCT = [];
    counterct = 1;
    
    for i = 1:sizeDisx(2)
       % if CamCoordPtsR(3,i)<350
        CordCT(:,i) = Mext*[CamCoordPts(1,i);CamCoordPts(2,i);CamCoordPts(3,i);1];
         counterct = counterct+1;
       % end
    end
    
    for i = 1:sizeDisx(2)
        CordW(:,i) = Trans*[CordCT(1,i);CordCT(2,i);CordCT(3,i);1];
    end
    
        
     transCW = transpose(CordW(1:3,:));
     sRc = size(CordW);
     if sRc(2)==MaxPoints
         cordW(:,:,Tm) =transCW;
     else
    transCW(sRc(2):MaxPoints,:)=0;
    cordW(:,:,Tm) =transCW;
     end
    

    %% Rotate platform
     fprintf(aMega,'r1r');
end

xyzPoints = [];
scW = size(cordW);
counter = 1;
for i0 = 1:scW(3)
    for i1 = 1:scW(1)
       %if cordWR(i1,2,i0)>1
        xyzPoints(counter,:)=cordW(i1,:,i0);
        counter=counter+1;
       %end
    end
end


ptCloud =pointCloud( xyzPoints);
ptCloud = pcdenoise(ptCloud);
pcwrite(ptCloud,'PointCloudOutput','PLYFormat','ascii');
figure; pcshow(ptCloud);
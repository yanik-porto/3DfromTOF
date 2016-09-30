% function [ srImage ] = LidarBoost(C, deltaEstimation, phiEstimation, beta)
%Function according to Sebastian Schuon paper 2009, "LidarBoost: Depth
%Superresolution for ToF 3D Shape Scanning"
% @C : cell of 5 LR images
% @deltaEstimation: shift estimation between images (all referencede to
% first image coordinate frame)
% @phiEstimation: rotation estimation between images (all referencede to
% first image coordinate frame)
% @beta: srFactor
% @srImage: output Super Resolution Image
% Inspired by Luis Parra - 07/01/2014 - Universidad de Extremadura, Spain.
 
C = depth_img;
deltaEstimation = 0;
phiEstimation = 18;
beta = 4;

%% SET START CONDITIONS
% Number of depth Images of current chunk
N = size(C,2);
 
% Size of Low Resolution image
[n, m] = size(C{1});
 
% Preallocated Matrices
srImage = zeros(beta*n,beta*m); % Output high Resolution image
Tk = zeros(beta*n,beta*m); % Diagonal matrix that contain 0 entries for unreliables Dk samples
Dk = cell(1,N); % Upsampled with nearest neighborg and aligned N - Low resolution frames
for i=1:N
 Dk{i} = zeros(beta*n,beta*m);
 Wk{i} = zeros(beta*n,beta*m); % Banded matrix that encodes the position of Dk
end

%% ALIGN AND NEAREST NEIGHBORG - WE ASSUME NO ROTATION, ONLY TRANSLATION
% Compute Lukas Kanade for Current Chunk of images and build delta - All referenced to
% first frame
windowSize = 5; % 5 x 5 Window
for i = 2:N
 [u{i-1}, v{i-1}] = LucasKanade(double(C{1}), double(C{i}), windowSize);
 currentU = u{i-1};
 currentV = v{i-1};
 meanU = mean(currentU(currentU~=0));
 meanV = mean(currentV(currentV~=0));
 
delta(i-1,1) = meanU; % Delta represent X-Y (u,v) translation of each frame to respect to frame 1
 delta(i-1,2) = meanV;
end
 
% Warp each image to reference frame 1,with affine transformation and delta
% and then apply nearest neighbor interpolation
Dk{1} = double(imresize(C{1},beta,'nearest'));
for i =2:N
 tx = delta(i-1,1);
 ty = delta(i-1,2);
 tform = maketform('affine',[1 0 ; 0 1; tx ty]);
 tempImage = imtransform(C{i},tform,'XData',[1 size(C{1},2)], 'YData',[1 size(C{1},1)]);
 Dk{i} = double(imresize(tempImage,beta,'nearest'));
end

%% 
% uMax = size(Dk{1},1);
% vMax = size(Dk{1},2);
% for k=1:N
%   for u=1:(uMax-1)
%     for v=1:(vMax-1)
%      i = round(((u-1)*(uMax-1))/(beta*uMax-1)+1);
%      j = round(((v-1)*(vMax-1))/(beta*vMax-1)+1);
% %      Dk{k}(u,v) = UDk{k}(i,j);
%      Wk{k}(i,j) = 1;
%     end
%   end
% end

for i=1:N
    Wk{i} = ones(beta*n, beta*m);
    Wk{i}(1,1) = 0;
    Wk{i}(beta*n, beta*m) = 0;
end


%% Minization
% Build 20 x 20 images patches with a 18 pixel step
step = 18;
patch = 20;
tic;
for i=1:step:(n*beta)-patch
 for j=1:step:(m*beta)-patch
disp('i value');
disp(i);
disp('j value');
disp(j);
 
for k = 1:N
 Wkmin(:,:,k) = Wk{k}(i:(i+patch-1),j:(j+patch-1));
 if (isempty(Wkmin))
 Wkmin = zeros(patch);
 end
 Dkmin(:,:,k) = Dk{k}(i:(i+patch-1),j:(j+patch-1));
 if (isempty(Dkmin))
 Dkmin = zeros(patch);
 end
 end
 
 cvx_begin
 variable X(patch,patch);
 minimize(sum(norm(Wkmin(:,:,k).*(Dkmin(:,:,k)-X))));
 cvx_end
 X = full(X); % sparse to full matrix
 Xhr((i:(i+patch-1)),(j:(j+patch-1))) = X(:,:); % This begin to build our HR IMAGE!!
 end
end
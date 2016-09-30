clear all;
close all;
clc;

%%
pts = loadpcd('10frames.pcd');
pts1 = pts(:,pts(4,:)>0.0080);
pts2 = pts1(:,pts1(3,:)<2.5);
pcshow(pts(1:3,:)','VerticalAxis','Y');
%savepcd('handFilter.pcd',pts2(1:3,:),'ascii');
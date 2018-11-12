clear all
close all 

%Prepare parameters and access to functions
load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');

r1 = dir ('.\Cam1\*.png'); %lists the files in directory .png (all the images)
d1 = dir('.\Cam1\*.mat'); %lists the files in directory .mat (all the depth images)
r2 =  dir ('.\Cam2\*.png'); %lists the files in directory .png (all the images)
d2 = dir('.\Cam1\*.mat'); %lists the files in directory .mat (all the depth images)

imgs = [];

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (d1) %for each image in time
    im = imread(d1(i).name);
    load(dd1(i).name);
    imgs = cat(3,imgs,im); 
    imgsd(:,:,i) = double(depth_array)/1000; %store all depth_array
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd(:,:,i),[480*640,1]); %vectorize
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (d2) %for each image in time
    im = imread(d2(i).name);
    load(dd2(i).name);
    imgs = cat(3,imgs,im); 
    imgsd(:,:,i) = double(depth_array)/1000; %store all depth_array
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd(:,:,i),[480*640,1]); %vectorize
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end


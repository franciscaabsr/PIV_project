clear all
close all 

%Prepare parameters and access to functions
run('VLFEATROOT/toolbox/vl_setup');
load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');

r1 = dir ('./rgb_image1_*'); %lists the files in directory .png (all the images)
d1 = dir('./depth1_*'); %lists the files in directory .mat (all the depth images)
r2 =  dir ('./rgb_image2_*'); %lists the files in directory .png (all the images)
d2 = dir('./depth2_*'); %lists the files in directory .mat (all the depth images)

%sort in alphanumeric order
r1_sorted = natsortfiles({r1.name})';
d1_sorted = natsortfiles({d1.name})';
r2_sorted = natsortfiles({r2.name})';
d2_sorted = natsortfiles({d2.name})';

imgs_1 = [];
imgsd_1 = zeros(480,640,length(d1));
xyz_depth_1 = zeros(480*640,3,length(d1));
rgbd_1 = zeros(480,640,3*length(r1));

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (r1) %for each image in time
    im = imread(char(r1_sorted(i)));
    load(char(d1_sorted(i)));
    imgs_1 = cat(3,imgs_1,im); 
    imgsd_1(:,:,i) = double(depth_array)/1000; %store all depth_array
    [r,c] = ind2sub(size(imgsd_1(:,:,i)),find(imgsd_1(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd_1(:,:,i),[480*640,1]); %vectorize
    xyz_depth_1(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    rgbd_1(:,:,i:i+2) = get_rgbd(xyz_depth_1(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

imgs_2 = [];
imgsd_2 = zeros(480,640,length(d2));
xyz_depth_2 = zeros(480*640,3,length(d2));
rgbd_2 = zeros(480,640,3*length(r2));

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (r2) %for each image in time
    im = imread(char(r2_sorted(i)));
    load(char(d2_sorted(i)));
    imgs_2 = cat(3,imgs_2,im); 
    imgsd_2(:,:,i) = double(depth_array)/1000; %store all depth_array
    [r,c] = ind2sub(size(imgsd_2(:,:,i)),find(imgsd_2(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd_2(:,:,i),[480*640,1]); %vectorize
    xyz_depth_2(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    rgbd_2(:,:,i:i+2) = get_rgbd(xyz_depth_2(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

%computation of keypoints and its descriptors for first frame (in time)
im1 = single(rgb2gray(rgbd_1(:,:,1:3))); %images are needed to be in grayscale and in single precision
im2 = single(rgb2gray(rgbd_2(:,:,1:3)));

[f1,d1] = vl_sift(im1) ;
[f2,d2] = vl_sift(im2) ;

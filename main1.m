clear all
close all 

%Prepare parameters and access to functions
load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');

%Put all data in respective directory
r = dir ('./Cam1/*.png'); %lists the files in directory .jpg or .png (all the rgb images)
d = dir('./Cam1/*.mat'); %lists the files in directory .mat (all the depth images)
imgs = [];

%sort in alphanumeric order
r_sorted = natsortfiles({r.name})';
d_sorted = natsortfiles({d.name})';

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (r) %for each image in time
    im = imread(r_sorted(i).name);
    imgs = cat(3,imgs,im); 
    load(d_sorted(i).name);
    imgsd(:,:,i) = double(depth_array)/1000; %store all depth_array
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd(:,:,i),[480*640,1]); %vectorize
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

clear all
close all 

%Prepare parameters and access to functions
load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');

%Put all data in respective directory
r = dir ('./rgb_image1_*'); %lists the files in directory .jpg or .png (all the rgb images)
d = dir('./depth1_*'); %lists the files in directory .mat (all the depth images)
imgs = [];
imgsd = zeros(480,640,length(d));
xyz_depth = zeros(480*640,3,length(d));
rgbd = zeros(480,640,3*length(r));

%sort in alphanumeric order
r_sorted = natsortfiles({r.name})';
d_sorted = natsortfiles({d.name})';

%computation of rgbd (rgb image expressed in depth reference frame)
for i = 1: length (r) %for each image in time
    im = imread(char(r_sorted(i)));
    imgs = cat(3,imgs,im); 
    load(char(d_sorted(i)));
    imgsd(:,:,i) = double(depth_array); %store all depth_array in mm
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd(:,:,i),[480*640,1]); %vectorize
    xyz_depth(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    [rgbd(:,:,i:i+2), u1_temp, v1_temp, xyz_rgb_1_temp] = get_rgbd(xyz_depth(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
    if i == 1 %only the values for the first image are required
        u1 = u1_temp;
        v1 = v1_temp;
        xyz_rgb_1 = xyz_rgb_1_temp;
    end
end


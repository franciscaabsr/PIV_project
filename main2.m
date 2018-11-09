clear all
close all 

d1 = dir ('.\Cam1\*.png'); %lists the files in directory .png (all the images)
dd1 = dir('.\Cam1\*.mat'); %lists the files in directory .mat (all the depth images)
d2 =  dir ('.\Cam2\*.png'); %lists the files in directory .png (all the images)
dd2 = dir('.\Cam1\*.mat'); %lists the files in directory .mat (all the depth images)
load ('.\CalibData\cameraparametersAsus.mat');
imgs = [];


for i = 1: length (d1)
    im = imread(d1(i).name);
    load(dd1(i).name);
    imgs = cat(3,imgs,im); 
    imgsd(:,:,i) = double(depth_array)/1000; 
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i)));
    im_vec = reshape(imgsd(:,:,i),[480*640,1]);
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb);1
end


for i = 1: length (d1)
    im = imread(d1(i).name);
    load(dd1(i).name);
    imgs = cat(3,imgs,im); 
    imgsd(:,:,i) = double(depth_array)/1000; 
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i)));
    im_vec = reshape(imgsd(:,:,i),[480*640,1]);
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb);1
end


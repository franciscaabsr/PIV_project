clear all
close all 

d = dir ('*.jpg'); %lists the files in directory .jpg (all the images)
dd = dir('*.mat'); %lists the files in directory .mat (all the depth images)
load ('.\CalibData\cameraparametersAsus.mat');
imgs = [];

for i = 1: length (d)
    im = imread(d(i).name);
    imgs = cat(3,imgs,im); 
    load(dd(i).name);
    imgsd(:,:,i) = double(depth_array)/1000; 
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i)));
    im_vec = reshape(imgsd(:,:,i),[480*640,1]);
    xyz_depth = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    rgbd = get_rgbd(xyz_depth, im, cam_params.R, cam_params.T, cam_params.Krgb);
end

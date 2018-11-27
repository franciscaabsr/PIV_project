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
    imgsd_1(:,:,i) = double(depth_array); %store all depth_array in milimeters
    [r,c] = ind2sub(size(imgsd_1(:,:,i)),find(imgsd_1(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd_1(:,:,i),[480*640,1]); %vectorize
    xyz_depth_1(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    [rgbd_1(:,:,i:i+2),u1, v1, xyz_rgb_1] = get_rgbd(xyz_depth_1(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
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
    imgsd_2(:,:,i) = double(depth_array); %store all depth_array in mm
    [r,c] = ind2sub(size(imgsd_2(:,:,i)),find(imgsd_2(:,:,i))); %select (i,j) that correspond to entries non zero
    im_vec = reshape(imgsd_2(:,:,i),[480*640,1]); %vectorize
    xyz_depth_2(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); %compute xyz in depth reference frame
    [rgbd_2(:,:,i:i+2),u2,v2,xyz_rgb_2] = get_rgbd(xyz_depth_2(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

%computation of keypoints and its descriptors for first frame (in time)

%fazer sift na img original
im1=imread('rgb_image1_1.png');
im2=imread('rgb_image2_1.png');

%images are needed to be in grayscale and in single precision
[f1,d1] = vl_sift(single(rgb2gray(im1))) ; %images are needed to be in grayscale and in single precision
[f2,d2] = vl_sift(single(rgb2gray(im2))) ;

figure(1);
imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
figure(2);
imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;

[matches, scores] = vl_ubcmatch(d1, d2) ;
%matches sao os indices do keypoints que deram match
%o indice e a coluna na matriz de descriptors d1_t e d2_t que e igual ao
%indice de f1 e f2
%a primeira linha de f1_t corresponde a x e a segunda corresponde a y na
%img original
figure(3); clf;
imagesc(cat(2,im1,im2));

%obtain the coordinates (u,v) for the matches in the original pair of rgb
u1_m = f1(1,matches(1,:));
v1_m = f1(2,matches(1,:));
u2_m = f2(1,matches(2,:));
v2_m = f2(2,matches(2,:));

u2_m_plot = f2(1,matches(2,:))+size(im1,2);

hold on;
h = line([u1_m; u2_m_plot],[v1_m;v2_m]);
set(h, 'linewidth', 1, 'color', 'b');

% we want to find depth_pixel corresponding to these (u,v) from the match
% in the rgb
% (u,v) matched -> find (u,v) corresponding in (u,v) obtained in the
% rgbd(mapping) -> cand determine corresponding pixel in depth image!

%check correspondence u1_m with u1 and correspondence v1_m with v1, find
%best for both (min of sum((u1_m-u1)+(v1_m-v1))) -> this entry will
%correspond to the pixel of the depth image

val_1 = [];
val_2 = [];
depth_index_1 = [];
depth_index_2 = [];

for i=1:size(u1_m,2)
    [val_1(i), depth_index_1(i)] = min(abs(u1-u1_m(i))+abs(v1-v1_m(i)));
    [val_2(i), depth_index_2(i)] = min(abs(u2-u2_m(i))+abs(v2-v2_m(i)));
end

%create vector of z values for 45 depth_index of each camera
%compute xyz values for each camera
%determine centroid of each pointcloud to subtract it

%apply rigid motion xyz = R* xyz + T -> RANSAC 
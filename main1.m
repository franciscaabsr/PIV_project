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

%% Estimate the background

%compute median of every pixel in terms of time (third dimension in list of matrices)
bgdepth = median(imgsd,3);

%% Background subtraction for depth (try with gray too)
hold all

for i=1:length(d);
    
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.50; 
    %subtracting the background to the image in order to obtain moving
    %objects
    %if difference is bigger than .20, then it s true and gives value
    %1(white), as not part of background. if not, false and gives value
    %0(black), as part of background
    
    % with each image obtained, we can see that these images have white contours that are actually background 
    % so we can "clean" these, considering the neighbourhood of each pixel
    imgdiffiltered=imopen(imdiff,strel('disk',5));
    
    %opens imdiff with structuring element strel('disk',5): 2D disk with radius 5 pixels
    %N=4 (default), meaning that the disk is approximated by a sequence of 4 periodic-line structuring elements
    %removes white elements (1) with radius less then 5 pixels (verifies if
    %neighbours are classified as background or not)
    %if in a disk of radius=5 we have one black element (background) then
    %that pixel with that neighbour will also be classified as blackground
    %this means "opening" the image with the disk, working like a filter
    %for classification
    
    %open filter - clean image (erosion) from everything smaller than 
    %disk and then do the dilation to recover the original shape except the small stuff I took away
    
    %we will only have big connected components, that will be our moving
    %objects that we wish to detect
  
    figure(1);
    imagesc([imdiff imgdiffiltered]); 
    title('Difference image and morph filtered');
    colormap(gray); %define colour map to know the scale in terms of black and white
    figure(2);
    imagesc([imgsd(:,:,i) bgdepth]);
    title('Depth image i and background image');
    figure(3);
    imagesc(bwlabel(imgdiffiltered)); %obtain the same image now labeled with connected components 
    %(possible moving objects); value zero is the background
    title('Connected components');
    pause(1);
end
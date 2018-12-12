clear all
close all 

%Prepare parameters and access to functions
load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');
addpath('hungarian_method/');

%Put all data in respective directory
%r = dir ('./rgb_image1_*'); %lists the files in directory .jpg or .png (all the rgb images)
%d = dir('./depth1_*'); %lists the files in directory .mat (all the depth images)
r = dir ('*.jpg'); %lists the files in directory .jpg or .png (all the rgb images)
d = dir('*.mat'); %lists the files in directory .mat (all the depth images)
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
end

% Estimate the background

%compute median of every pixel in terms of time (third dimension in list of matrices)
bgdepth = median(imgsd,3);

% Background subtraction for depth (try with gray too)
hold all

imgdiffiltered = [];
imlabel = [];
num_components = zeros(1,length(d));


for i=1:length(d)
    
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.50; 
    %subtracting the background to the image in order to obtain moving
    %objects
    %1 foreground, 0 background
    
    % with each image obtained, we can see that these images have white contours that are actually background 
    % so we can "clean" these, considering the neighbourhood of each pixel
    imgdiffiltered= cat(3, imgdiffiltered, imopen(imdiff,strel('disk',8)));
    
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
  
    %figure(1);
    %imagesc([imdiff imgdiffiltered(:,:,i)]); 
    %title('Difference image and morph filtered');
    %colormap(gray); %define colour map to know the scale in terms of black and white
    %figure(2);
    %imagesc([imgsd(:,:,i) bgdepth]);
    %title('Depth image i and background image');

    [im_label(:,:,i), num_components(i)] = bwlabel(imgdiffiltered(:,:,i));
    
    figure(1);
    imagesc(im_label(:,:,i));
    title('Connected components');
    pause(1);
end

%remove small components TODO
%update variable num_components after removing small components TODO

centroid = zeros(3, max(num_components), length(d));
%box = zeros(8*3, max(num_components), length(d));
for i =  1 : length(d)
    for j = 1 : num_components(i)
       indx = find(im_label(:,:,i)==j);
       %M = [min(xyz_depth(indx,1,i)),min(xyz_depth(indx,2,i)),min(xyz_depth(indx,3,i)), max(xyz_depth(indx,1,i)), max(xyz_depth(indx,2,i)), max(xyz_depth(indx,3,i))]
       %box(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6), 
                      % M(1), M(5), M(6), M(1), M(5), M(3),
                      % M(4), M(2), M(3), M(4), M(2), M(6), 
                      % M(4), M(5), M(6), M(4), M(5), M(3)]
       centroid(:,j,i)= [mean(xyz_depth(indx,1,i)), mean(xyz_depth(indx,2,i)), mean(xyz_depth(indx,3,i))];
    end
end

%to store sequence of components along the images, with components of first
%image as a starting point
track = zeros(num_components(1), length(d));
track(:,1) = 1:num_components(1);

%compare components of image pair by pair -> total of length(d)-1 pairs
for i = 1 : length(d)-1
    j = i + 1;
    
    % to compare all combinations of components from one image with the
    % ones from the second image
    %change to 3D box !!! TODO
    centroid_j = reshape(nonzeros(centroid(:,:,j)), 3, num_components(j));
    centroid_i = reshape(nonzeros(centroid(:,:,i)), 3, num_components(i));
    centroid_i = reshape(centroid_i, 3*num_components(i),1);
    
    dif = repmat(centroid_j,num_components(i),1) - repmat(centroid_i,1,num_components(j));
    norm = vecnorm(reshape(dif, 3, num_components(i)*num_components(j)));
    %cost matrix, of proximity values between centroids of all components
    %of the two images
    costmat = reshape(norm, num_components(i), num_components(j));
    
    %hungarian method to determine the assignment
    [assign, cost] = munkres(costmat); 
    
    %find(assign==1)

end

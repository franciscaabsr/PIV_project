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
    [rgbd_1(:,:,i:i+2),u1_temp, v1_temp, xyz_rgb_1_temp] = get_rgbd(xyz_depth_1(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
    if i == 1 %only the values for the first image are required to find features
        u1 = u1_temp;
        v1 = v1_temp;
        xyz_rgb_1 = xyz_rgb_1_temp;
    end
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
    [rgbd_2(:,:,i:i+2),u2_temp,v2_temp,xyz_rgb_2_temp] = get_rgbd(xyz_depth_2(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
    if i == 1 %only the values for the first image are required
        u2 = u2_temp;
        v2 = v2_temp;
        xyz_rgb_2 = xyz_rgb_2_temp;
    end
end

%computation of keypoints and its descriptors for first frame (in time)

%fazer sift na img original
im1=imread('rgb_image1_0001.png');
im2=imread('rgb_image2_0001.png');

%images are needed to be in grayscale and in single precision
[f1,d1] = vl_sift(single(rgb2gray(im1))) ; %images are needed to be in grayscale and in single precision
[f2,d2] = vl_sift(single(rgb2gray(im2))) ;

figure(1);
imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
figure(2);
imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;

[matches, scores] = vl_ubcmatch(d1, d2, 2) ;
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
number_of_matches = size(u1_m,2)
val_1 = zeros(1,number_of_matches);
val_2 = zeros(1,number_of_matches);
depth_index_1 = zeros(1,number_of_matches);
depth_index_2 = zeros(1,number_of_matches);
xyz_matches_1 = zeros(3,number_of_matches);
xyz_matches_2 = zeros(3,number_of_matches);

for i=1:size(u1_m,2)
    [val_1(i), depth_index_1(i)] = min(abs(u1-u1_m(i))+abs(v1-v1_m(i)));
    [val_2(i), depth_index_2(i)] = min(abs(u2-u2_m(i))+abs(v2-v2_m(i)));
    
    %compute xyz values for depth camera of each kinect that correspond to
    %the points matched in the rgb images
    xyz_matches_1(:,i) = [xyz_depth_1(depth_index_1(i),1,1) ; xyz_depth_1(depth_index_1(i),2,1) ; xyz_depth_1(depth_index_1(i),3,1)];
    xyz_matches_2(:,i) = [xyz_depth_2(depth_index_2(i),1,1) ; xyz_depth_2(depth_index_2(i),2,1) ; xyz_depth_2(depth_index_2(i),3,1)];
end

%RANSAC TIME!!!
%vary threshold to see if results are improved!! 100 iterations
[R_final_21, T_final_21, max_inliers] = ransac(xyz_matches_1, xyz_matches_2);

verify = R_final_21'*R_final_21; %if it is identity matrix
det = det(R_final_21);

% plot inliers and its matches over the two images

figure(4); clf;
imagesc(cat(2,im1,im2));

%to select only the inliers
u1_m_in = u1_m(:,max_inliers);
v1_m_in = v1_m(:, max_inliers);
u2_m_in = u2_m_plot(:, max_inliers);
v2_m_in = v2_m(:, max_inliers);

hold on;
h = line([u1_m_in; u2_m_in],[v1_m_in;v2_m_in]);
set(h, 'linewidth', 1, 'color', 'b');

% DETECTION OF MOVING OBJECTS BY THE TWO CAMERAS

% Estimate the background and components for camera 1

%compute median of every pixel in terms of time (third dimension in list of matrices)
bgdepth_1 = median(imgsd_1,3);

%background subtraction for depth (try with gray too)
hold all

imgdiffiltered_1 = [];
im_label_1 = zeros(480,640,length(d1));
num_components_1 = [];

for i=1:length(d1_sorted)
    
    imdiff=abs(imgsd_1(:,:,i)-bgdepth_1)>50; 
    %subtracting the background to the image in order to obtain moving
    %objects: 1 foreground, 0 background
    
    % with each image obtained, we can see that these images have white contours that are actually background 
    % so we can "clean" these, considering the neighbourhood of each pixel
    imdiff = imopen(imdiff,strel('disk',8));
    %ignore objects with less than 1000 pixels
    imdiff = bwareaopen(imdiff,1000);
    imgdiffiltered_1= cat(3, imgdiffiltered_1, imdiff);
    
    %store for each image: matrix with labels and number of components
    [im_label_1(:,:,i), num_components_1(i)] = bwlabel(imgdiffiltered_1(:,:,i));
    
    %remove outliers in relation to values of z
    for j = 1 : num_components_1(i)
         comp_indx = find(im_label_1(:,:,i)==j);         
         comp_points = (xyz_depth_1(comp_indx,3,i));
         
         error_indx = find(comp_points == 0); % get indexes from depth cam errors
         if length(error_indx) > length(comp_indx)*.5 %if the object is more than 50% erros, delete obj
             error_indx = 1:length(comp_indx);
         end
         
         avg_point = median(nonzeros(comp_points));
         other_indx = find(abs(comp_points-avg_point) > 1);
         other_indx = union(other_indx, error_indx);
         if other_indx
             num_removed_1(i,j) = length(other_indx);
             others_indx = comp_indx(other_indx);
             filterd_im = reshape(im_label_1(:,:,i), 480*640, 1);
             filterd_im(others_indx) = 0;
             filterd_im = reshape(filterd_im, 480,640);
             im_label_1(:,:,i) = filterd_im;
         end
         
    end
    
    im_label_1(:,:,i) = bwareaopen(im_label_1(:,:,i),1000);
    
    %store for each image: matrix with labels and number of components
    [im_label_1(:,:,i), num_components_1(i)] = bwlabel(im_label_1(:,:,i));
    
    figure(5);
    imagesc(im_label_1(:,:,i));
    title('Connected components for camera 1');
    %pause(1); 
end

% Estimate the background for camera 2

%compute median of every pixel in terms of time (third dimension in list of matrices)
bgdepth_2 = median(imgsd_2,3);

%background subtraction for depth (try with gray too)
hold all

imgdiffiltered_2 = [];
im_label_2 = zeros(480,640,length(d1));
num_components = [];

for i=1:length(d2_sorted)
    
    imdiff=abs(imgsd_2(:,:,i)-bgdepth_2)>50; 
    %subtracting the background to the image in order to obtain moving
    %objects: 1 foreground, 0 background
    
    % with each image obtained, we can see that these images have white contours that are actually background 
    % so we can "clean" these, considering the neighbourhood of each pixel
    imdiff = imopen(imdiff,strel('disk',8));
    %ignore objects with less than 1000 pixels
    imdiff = bwareaopen(imdiff,1000);
    imgdiffiltered_2= cat(3, imgdiffiltered_2, imdiff);
    
    %store for each image: matrix with labels and number of components
    [im_label_2(:,:,i), num_components_2(i)] = bwlabel(imgdiffiltered_2(:,:,i));
    
    %remove outliers in relation to values of z
    for j = 1 : num_components_2(i)
         comp_indx = find(im_label_2(:,:,i)==j);         
         comp_points = (xyz_depth_2(comp_indx,3,i));
         
         error_indx = find(comp_points == 0); % get indexes from depth cam errors
         if length(error_indx) > length(comp_indx)*.5 %if the object is more than 50% erros, delete obj
             error_indx = 1:length(comp_indx);
         end
         
         avg_point = median(nonzeros(comp_points));
         other_indx = find(abs(comp_points-avg_point) > 1);
         other_indx = union(other_indx, error_indx);
         if other_indx
             others_indx = comp_indx(other_indx);
             filterd_im = reshape(im_label_2(:,:,i), 480*640, 1);
             filterd_im(others_indx) = 0;
             filterd_im = reshape(filterd_im, 480,640);
             im_label_2(:,:,i) = filterd_im;
         end
         
    end
    
    im_label_2(:,:,i) = bwareaopen(im_label_2(:,:,i),1000);
    
    %store for each image: matrix with labels and number of components
    [im_label_2(:,:,i), num_components_2(i)] = bwlabel(im_label_2(:,:,i));
    
    figure(1);
    imagesc(im_label_2(:,:,i));
    title('Connected components for camera 2');
    %pause(1);
end

% check components that match for the two cameras

% each box is composed by 8 points, each with (x,y,z)

box_1 = zeros(24,max(num_components_1),length(d1_sorted));
box_2 = zeros(24,max(num_components_2),length(d2_sorted));
% we want the box 2 expressed in camera 1 reference frame

for i =  1 : length(d1_sorted)   
    for j = 1 : num_components_1(i)
       index = find(im_label_1(:,:,i)==j);
       M = [min(xyz_depth_1(index,1,i)),min(xyz_depth_1(index,2,i)),min(xyz_depth_1(index,3,i)), max(xyz_depth_1(index,1,i)), max(xyz_depth_1(index,2,i)), max(xyz_depth_1(index,3,i))];
       box_1(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
    end
end

% express xyz coordinates of camera 2 in camera 1 reference frame

xyz_2_1 = zeros(480*640,3,length(d2_sorted));

for i =  1 : length(d2_sorted)  
    
    xyz_2_1(:,:,i) = (R_final_21 * xyz_depth_2(:,:,i)' + repmat(T_final_21,1,480*640))';
    
    for j = 1 : num_components_2(i)
       index = find(im_label_2(:,:,i)==j);
       M = [min(xyz_2_1(index,1,i)),min(xyz_2_1(index,2,i)),min(xyz_2_1(index,3,i)), max(xyz_2_1(index,1,i)), max(xyz_2_1(index,2,i)), max(xyz_2_1(index,3,i))];
       box_2(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
    end
end

% determine correspondent components of images between two cameras and
% recompute boxes for common components
assigns = [];
box_common = cat(2, box_1, zeros(24, max(num_components_2), length(d2_sorted)));
num_components_t = num_components_1;

for i = 1 : length(d1_sorted)
    
    % to compare all combinations of components from one image with the
    % ones from the second image
    box_i2 = reshape(nonzeros(box_2(:,:,i)), 3*8, num_components_2(i)); %nonzeros eliminate zeros -> no component
    box_i1 = reshape(nonzeros(box_1(:,:,i)), 3*8, num_components_1(i));
    box_i1 = reshape(box_i1, 24*num_components_1(i),1);
    
    % compute norm of difference between the two images, for box (24
    % values) and for colour (30 values)
    difBox = repmat(box_i2,num_components_1(i),1) - repmat(box_i1,1,num_components_2(i));
    normBox = sqrt(sum((reshape(difBox, 24, num_components_1(i)*num_components_2(i))).^2, 1));
    %normBox = vecnorm(reshape(difBox, 24, num_components_1(i)*num_components_2(i)));  
    
    % cost matrix, of proximity values between centroids of all components
    % of the two images
    costmat = reshape(normBox, num_components_1(i), num_components_2(i));
    
    [assign, cost] = munkres(costmat); % hungarian method to determine the assignment
    
    box_i1 = reshape(box_i1, 3*8, num_components_1(i));
    
    for k = 1 : length(assign)
        assigns(k,i) = assign(k);
        if assigns(k,i) ~= 0
            x_min = min(box_i1(1,k), box_i2(1,assign(k)));
            y_min = min(box_i1(2,k), box_i2(2,assign(k)));
            z_min = min(box_i1(3,k), box_i2(3,assign(k)));
            x_max = max(box_i1(19,k), box_i2(19,assign(k)));
            y_max = max(box_i1(20,k), box_i2(20,assign(k)));
            z_max = max(box_i1(21,k), box_i2(21,assign(k)));
            M = [x_min,y_min,z_min,x_max,y_max,z_max];
            box_common(:,k,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
        end
        
    end 
    
    if length(assign) < num_components_2(i)
            % components that did not match with the ones of image from
            % camera 1
            new = setdiff(1:num_components_2(i),assign);
            box_common(:,new + max(num_components_1),i) = box_2(:,new,i);
            num_components_t(i) = num_components_t(i) + length(new);
            % to have the total number of components to do tracking
    end
    
end

% Tracking through time, considering the information of the two cameras
% together

%to store sequence of components along the images, with components of slice of image as a starting point
track(:,1) = 1:num_components_t(1);

%compare components of image pair by pair -> total of length(d)-1 pairs
for i = 1 : length(d1_sorted)-1
    j = i + 1;
    
    % to compare all combinations of components from one image with the
    % ones from the second image
    box_j = reshape(nonzeros(box_common(:,:,j)), 3*8, num_components_t(j)); %nonzeros eliminate zeros -> no component
    box_i = reshape(nonzeros(box_common(:,:,i)), 3*8, num_components_t(i));
    box_i = reshape(box_i, 24*num_components_t(i),1);
    
    % compute norm of difference between the two images, for box (24
    % values) and for colour (30 values)
    difBox = repmat(box_j,num_components_t(i),1) - repmat(box_i,1,num_components_t(j));
    normBox = sqrt(sum((reshape(difBox, 24, num_components_t(i)*num_components_t(j))).^2, 1));
    %normBox = vecnorm(reshape(difBox, 24, num_components_t(i)*num_components_t(j)));  
    
    % cost matrix, of proximity values between centroids of all components
    % of the two images
    costmat = reshape(normBox, num_components_t(i), num_components_t(j));
    
    [assign, cost] = munkres(costmat); % hungarian method to determine the assignment

    
    if ~isempty(assign)
        for index = 1 : length(assign)
            indextrack = find(track(:,i)==index);
            track(indextrack,j) = assign(index);
        end
    else
        track =  cat(2,track,zeros(size(track,1),1));
    end
    
    %verify
    if length(assign) < num_components_t(j)
        new = setdiff(1:num_components_t(j),assign);
        track=cat(1,track,cat(2,zeros(length(new),size(track,2)-1),new'));
    end
    
end       

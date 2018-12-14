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
    [rgbd(:,:,i:i+2), u_temp, v_temp, xyz_rgb_temp] = get_rgbd(xyz_depth(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
end

%estimate the background

%compute median of every pixel in terms of time (third dimension in list of matrices)
bgdepth = median(imgsd,3);

%background subtraction for depth (try with gray too)
hold all

imgdiffiltered = [];
im_label = [];
num_components = [];

%VERIFICAR DAQUI PARA BAIXO!

for i=1:length(d)
    
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.50; 
    %subtracting the background to the image in order to obtain moving
    %objects: 1 foreground, 0 background
    
    % with each image obtained, we can see that these images have white contours that are actually background 
    % so we can "clean" these, considering the neighbourhood of each pixel
    imdiff = imopen(imdiff,strel('disk',8));
    %ignore objects with less than 1000 pixels
    imdiff = bwareaopen(imdiff,1000);
    imgdiffiltered= cat(3, imgdiffiltered, imdiff);
    
    %store for each image: matrix with labels and number of components
    [im_label(:,:,i), num_components(i)] = bwlabel(imgdiffiltered(:,:,i));
    
    %remove pixels with z = 0 from every object
    mask = reshape((abs(xyz_depth(:,3,i)) ~= 0),480,640); %1 the oneswith z~=0 and 0 the ones with z=0
    im_label(:,:,i) = im_label(:,:,i).*mask;  
    %0 in the ones who are not objects and the labels in the objects
    
    figure(1);
    imagesc(im_label(:,:,i));
    title('Connected components');
    %pause(1);
end

colour = zeros(15*2,max(num_components),length(d));
%each box is composed by 8 points, each with (x,y,z)
box = zeros(3*8,max(num_components),length(d));
for i =  1 : length(d)
    [hue sat int] = rgb2hsv(rgbd(:,:,i:i+2));
    hue = reshape(hue, 480*640,1);
    sat = reshape(sat, 480*640,1);
    
    for j = 1 : num_components(i)
       indx = find(im_label(:,:,i)==j);
       M = [min(xyz_depth(indx,1,i)),min(xyz_depth(indx,2,i)),min(xyz_depth(indx,3,i)), max(xyz_depth(indx,1,i)), max(xyz_depth(indx,2,i)), max(xyz_depth(indx,3,i))];
       box(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
       
       %signiture for pattern in terms of colour
       histHue = histogram(hue(indx),15); histSat = histogram(sat(indx),15);
       colour(:,j,i) = cat(2,histHue.Values,histSat.Values)';
       
       if isempty(find(colour(:,j,i)==0)) == 0
        colour(find(colour(:,j,i)==0),j,i)=-1;
       end
    end
    
    %CREATE BOX AND SHOW TODO!!
    %box(:,j,i)
    %for j = 1 : num_components(i)
     %   box = reshape(box(:,j,i), 3, 8);
      %  X = cat(2, box(1,1), box(1,5)); % xmin xmax
       % Y = cat(2, box(2,:), box(2,1)); % ymin ymax
        %Z = cat(2, box(3,:), box(3,1)); % zmin zmax
        %to plot 3D line in each dimension, to create the box in the image
        %figure(2);
        %im = imread(char(r_sorted(i)));
        %imagesc(im);
        %hold on;
        %plot(plot::Box(box(:,1), box(:,8),Filled = FALSE));
        %plot3(X,Y,Z);
        %hold off;
%     X = box(1:3:end-2,1,i);
%     Y = box(2:3:end-1,1,i);
%     Z = box(3:3:end,1,i);
%     C = [0 0 0 0 0 0 0 0];
%     figure(2)
%     pcshow(xyz_depth(:,:,i)); hold on
%     fill3(X,Y,Z,C); hold off
   % end
    %hold off;
end

%to store sequence of components along the images, with components of first
%image as a starting point
track(:,1) = 1:num_components(1);

%compare components of image pair by pair -> total of length(d)-1 pairs
for i = 1 : length(d)-1
    j = i + 1;
    
    % to compare all combinations of components from one image with the
    % ones from the second image
    box_j = reshape(nonzeros(box(:,:,j)), 3*8, num_components(j)); %nonzeros eliminate zeros -> no component
    box_i = reshape(nonzeros(box(:,:,i)), 3*8, num_components(i));
    box_i = reshape(box_i, 24*num_components(i),1);
    
    colour_j = reshape(nonzeros(colour(:,:,j)), 30, num_components(j));
    colour_i = reshape(nonzeros(colour(:,:,i)), 30, num_components(i));
    colour_i = reshape(colour_i, 30*num_components(i),1);
    
    % compute norm of difference between the two images, for box (24
    % values) and for colour (30 values)
    difBox = repmat(box_j,num_components(i),1) - repmat(box_i,1,num_components(j));
    normBox = sqrt(sum((reshape(difBox, 24, num_components(i)*num_components(j))).^2, 1));
    %normBox = vecnorm(reshape(difBox, 24, num_components(i)*num_components(j)));
    
    difColour = repmat(colour_j,num_components(i),1) - repmat(colour_i,1,num_components(j));
    normColour = sqrt(sum((reshape(difColour, 30, num_components(i)*num_components(j))).^2,1));
    %normColour = vecnorm(reshape(difColour, 30, num_components(i)*num_components(j)));   
    
    % cost matrix, of proximity values between centroids of all components
    % of the two images
    costmat = reshape(normBox, num_components(i), num_components(j));
    costmat = costmat + reshape(normColour,num_components(i), num_components(j));
    
    [assign, cost] = munkres(costmat); % hungarian method to determine the assignment

    for index = 1 : length(assign)
        indextrack = find(track(:,i)==index);
        track(indextrack,j) = assign(index);
    end 
 
    %verify
    if length(assign) < num_components(j)
        new = setdiff(1:num_components(j),assign);
        track=cat(1,track,cat(2,zeros(length(new),size(track,2)-1),new'));
    end
end

for j = 1 : size(track,1)
    objects(j).framestracked=[];
    for i = 1 : length(d)
        if track (j,i) ~= 0
            objects(j).X(:,i)=box(1:3:end-2,track(j,i),i);
            objects(j).Y(:,i)=box(2:3:end-1,track(j,i),i);
            objects(j).Z(:,i)=box(3:3:end,track(j,i),i);
            objects(j).framestracked = cat(2, objects(j).framestracked,i);
        else
            objects(j).X(:,i)=zeros(8,1);
            objects(j).Y(:,i)=zeros(8,1);
            objects(j).Z(:,i)=zeros(8,1);
        end
    end
end

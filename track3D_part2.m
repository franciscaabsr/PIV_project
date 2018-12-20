function [objects, cam2toW] = track3D_part2(imgseq1, imgseq2, cam_params)
%clear all
%close all 

% Prepare parameters and access to functions
run('VLFEATROOT/toolbox/vl_setup');
addpath('hungarian_method/');

% REGISTRATION ???

%imgs_1 = [];
imgsd_1 = zeros(480,640,length(imgseq1));
xyz_depth_1 = zeros(480*640,3,length(imgseq1));
rgbd_1 = zeros(480,640,3*length(imgseq1));

% Computation of rgbd and xyz (rgbd: rgb image expressed in depth reference frame)

for i = 1: length(imgseq1)
    im = imread(imgseq1(i).rgb);
    load(imgseq1(i).depth);
    %imgs_1 = cat(3,imgs_1,im); 
    imgsd_1(:,:,i) = double(depth_array); % depth_array in mm
    % select (i,j) that correspond to entries non zero
    [r,c] = ind2sub(size(imgsd_1(:,:,i)),find(imgsd_1(:,:,i))); 
    im_vec = reshape(imgsd_1(:,:,i),[480*640,1]); 
    % compute xyz in depth reference frame
    xyz_depth_1(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); 
    % compute rgb corresponding to xyz_depth
    [rgbd_1(:,:,i:i+2),u1_temp, v1_temp, xyz_rgb_1_temp] = get_rgbd(xyz_depth_1(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
    % only the values for the first image are required to find features
    if i == 1
        u1 = u1_temp;
        v1 = v1_temp;
        %xyz_rgb_1 = xyz_rgb_1_temp;
    end
end

%imgs_2 = [];
imgsd_2 = zeros(480,640,length(imgseq2));
xyz_depth_2 = zeros(480*640,3,length(imgseq2));
rgbd_2 = zeros(480,640,3*length(imgseq2));

% Computation of rgbd and xyz (rgbd: rgb image expressed in depth reference frame)

for i = 1: length(imgseq2)
    im = imread(imgseq2(i).rgb);
    load(imgseq2(i).depth);
    %imgs_2 = cat(3,imgs_2,im); 
    imgsd_2(:,:,i) = double(depth_array); % depth_array in mm
    % select (i,j) that correspond to entries non zero
    [r,c] = ind2sub(size(imgsd_2(:,:,i)),find(imgsd_2(:,:,i))); 
    im_vec = reshape(imgsd_2(:,:,i),[480*640,1]); 
    % compute xyz in depth reference frame
    xyz_depth_2(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    [rgbd_2(:,:,i:i+2),u2_temp,v2_temp,xyz_rgb_2_temp] = get_rgbd(xyz_depth_2(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); %compute rgb corresponding to xyz_depth
    % only the values for the first image are required
    if i == 1 
        u2 = u2_temp;
        v2 = v2_temp;
        %xyz_rgb_2 = xyz_rgb_2_temp;
    end
end

% Computation of keypoints and its descriptors for first frame (in time)

im1=imread(imgseq1(1).rgb);
im2=imread(imgseq2(1).rgb);

% images need to be in grayscale and in single precision- SIFT
[f1,d1] = vl_sift(single(rgb2gray(im1))) ; 
[f2,d2] = vl_sift(single(rgb2gray(im2))) ;

% Visualization of keypoints
% figure(1);
% imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
% figure(2);
% imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;

[matches, scores] = vl_ubcmatch(d1, d2, 2);

% matches - index of keypoints (according to f1 and f2) that matched

% Visualization of matches
% figure(3); clf;
% imagesc(cat(2,im1,im2));

% obtain the coordinates (u,v) for the matches in the original pair of rgb
u1_m = f1(1,matches(1,:));
v1_m = f1(2,matches(1,:));
u2_m = f2(1,matches(2,:));
v2_m = f2(2,matches(2,:));

% u2_m_plot = f2(1,matches(2,:))+size(im1,2);
% 
% hold on;
% h = line([u1_m; u2_m_plot],[v1_m;v2_m]);
% set(h, 'linewidth', 1, 'color', 'b');

% Find depth_pixel corresponding to these (u,v) from the match in the rgb
% (u,v) matched -> find (u,v) in rgbd(mapping) -> determine pixel in depth image !

% check correspondence u1_m with u1 and correspondence v1_m with v1, find
% best for both (min of sum((u1_m-u1)+(v1_m-v1))) -> this entry will
% correspond to the pixel of the depth image
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
    
    % compute xyz values for depth camera of each kinect that correspond to
    % the points matched in the rgb images
    xyz_matches_1(:,i) = [xyz_depth_1(depth_index_1(i),1,1) ; xyz_depth_1(depth_index_1(i),2,1) ; xyz_depth_1(depth_index_1(i),3,1)];
    xyz_matches_2(:,i) = [xyz_depth_2(depth_index_2(i),1,1) ; xyz_depth_2(depth_index_2(i),2,1) ; xyz_depth_2(depth_index_2(i),3,1)];
end

% to make sure, remove matches with z = 0
index_nonzero = find(xyz_matches_1(3,:) ~= 0);
xyz_matches_1 = xyz_matches_1(:,index_nonzero);
index_nonzero = find(xyz_matches_2(3,:) ~= 0);
xyz_matches_2 = xyz_matches_2(:,index_nonzero);

% Removal of outliers and computation of R and T from 2 to World 

% RANSAC and Procrustes

[R_final_21, T_final_21, max_inliers] = ransac(xyz_matches_1, xyz_matches_2);

% Check rotation matrix restrictions 
%verify = R_final_21'*R_final_21;
%det = det(R_final_21);

% Plot inliers and its matches over the two images

% figure(4); clf;
% imagesc(cat(2,im1,im2));
%
% u1_m_in = u1_m(:,max_inliers);
% v1_m_in = v1_m(:, max_inliers);
% u2_m_in = u2_m_plot(:, max_inliers);
% v2_m_in = v2_m(:, max_inliers);
% 
% hold on;
% h = line([u1_m_in; u2_m_in],[v1_m_in;v2_m_in]);
% set(h, 'linewidth', 1, 'color', 'b');

% DETECTION OF MOVING OBJECTS BY THE TWO CAMERAS

% Estimate the background and components for camera 1
[im_label_1, num_components_1] = bg_subtraction(length(imgseq1), imgsd_1);

% Estimate the background for camera 2
[im_label_2, num_components_2] = bg_subtraction(length(imgseq2), imgsd_2);

% check components that match for the two cameras

% each box is composed by 8 points, each with (x,y,z)
box_1 = zeros(24,max(num_components_1),length(imgseq1));
box_2 = zeros(24,max(num_components_2),length(imgseq2));

% we want the box 2 expressed in camera 1 reference frame
for i =  1 : length(imgseq1)   
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

xyz_2_1 = zeros(480*640,3,length(imgseq2));

for i =  1 : length(imgseq2)  
    
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
box_common = cat(2, box_1, zeros(24, max(num_components_2), length(imgseq2)));
num_components_t = num_components_1;

for i = 1 : length(imgseq1)
    assign = [];
    % to compare all combinations of components from one image with the
    % ones from the second image
    box_i2 = reshape(nonzeros(box_2(:,:,i)), 3*8, num_components_2(i)); %nonzeros eliminate zeros -> no component
    box_i1 = reshape(nonzeros(box_1(:,:,i)), 3*8, num_components_1(i));
    box_i1 = reshape(box_i1, 24*num_components_1(i),1);
    
    if ~isempty(box_i2) && ~isempty(box_i1) % same as assign won't be empty
        % compute norm of difference between the two images, for box (24
        % values) and for colour (30 values)
        difBox = repmat(box_i2,num_components_1(i),1) - repmat(box_i1,1,num_components_2(i));
        normBox = sqrt(sum((reshape(difBox, 24, num_components_1(i)*num_components_2(i))).^2, 1));
        %normBox = vecnorm(reshape(difBox, 24, num_components_1(i)*num_components_2(i)));  

        % cost matrix (using proximity)
        costmat = reshape(normBox, num_components_1(i), num_components_2(i));

        % if cost too high -> not to be chosen by Hungarian Method
        ind = find(costmat > 100);
        costmat(ind) = 1e+10; 

        % check if entire row or column have absurd values - if so, must be
        % removed because the assign won't be valid
        index_c = find(all(costmat > 1e+9, 1));
        index_r = find(all(costmat > 1e+9, 2));
        costmat(:, index_c) = [];
        costmat(index_r, :) = [];

        % Hungarian Method to determine the assignment
        [assign, cost] = munkres(costmat);

        % correct assign array, if not empty, considering removed rows and columns
        if ~isempty(assign)
            % correct assign index considering rows removed
            if ~isempty(index_r)
                aux = zeros(1, num_components_t(i));
                aux(1, index_r) = 1;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            % correct assign values considering columns removed
            aux = zeros(1, num_components_t(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
        end
        
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

% to store sequence of components along the images, with components of slice of image as a starting point

track(:,1) = 1:num_components_t(1);

% compare components of image pair by pair

for i = 1 : length(imgseq1)-1
    j = i + 1;
    assign = [];
    % to compare all combinations of components from one image with the
    % ones from the second image
    box_j = reshape(nonzeros(box_common(:,:,j)), 3*8, num_components_t(j)); %nonzeros eliminate zeros -> no component
    box_i = reshape(nonzeros(box_common(:,:,i)), 3*8, num_components_t(i));
    box_i = reshape(box_i, 24*num_components_t(i),1);
    
    % compute norm of difference between the two images, for box (24
    % values) and for colour (30 values)
    if ~isempty(box_j) && ~isempty(box_i) % same as assign won't be empty
        
        difBox = repmat(box_j,num_components_t(i),1) - repmat(box_i,1,num_components_t(j));
        normBox = sqrt(sum((reshape(difBox, 24, num_components_t(i)*num_components_t(j))).^2, 1));
        %normBox = vecnorm(reshape(difBox, 24, num_components_t(i)*num_components_t(j)));  
    
        % cost matrix (using proximity)
        costmat = reshape(normBox, num_components_t(i), num_components_t(j));
    
        % if cost too high -> not to be chosen by Hungarian Method
        ind = find(costmat > 100);
        costmat(ind) = 1e+10; 
        
        % check if entire row or column have absurd values - if so, must be
        % removed because the assign won't be valid
        index_c = find(all(costmat > 1e+9, 1));
        index_r = find(all(costmat > 1e+9, 2));
        costmat(:, index_c) = [];
        costmat(index_r, :) = [];
        
        % Hungarian Method to determine the assignment
        [assign, cost] = munkres(costmat);
        
        % correct assign array, if not empty, considering removed rows and columns
        if ~isempty(assign)
            % correct assign index considering rows removed
            if ~isempty(index_r)
                aux = zeros(1, num_components_t(i));
                aux(1, index_r) = 1;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            % correct assign values considering columns removed
            aux = zeros(1, num_components_t(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
        end
        
        for index = 1:length(assign)
            %track only for assign non zero
            if assign(index) ~= 0
                indextrack = find(track(:,i)==index);
                track(indextrack,j) = assign(index);
            end
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

% Prepare output
for j = 1 : size(track,1)
    objects(j).framestracked=[];
    objects(j).X=[];
    objects(j).Y=[];
    objects(j).Z=[];
    for i = 1 : length(imgseq1)
        if track (j,i) ~= 0
            non_zeros = find(box_common(1,:,i) ~=0);
            box_col = non_zeros(track(j,i));
            objects(j).X = cat(1, objects(j).X, box_common(1:3:end-2,box_col,i)');
            objects(j).Y = cat(1, objects(j).Y, box_common(2:3:end-1,box_col,i)');
            objects(j).Z = cat(1, objects(j).Z, box_common(3:3:end,box_col,i)');
            objects(j).framestracked = cat(2, objects(j).framestracked,i);
        end
    end
end

cam2toW.R = R_final_21;
cam2toW.T = T_final_21;

end

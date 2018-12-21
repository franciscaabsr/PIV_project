function [objects, cam2toW] = track3D_part2(imgseq1, imgseq2, cam_params)
%clear all
%close all 

%prepare parameters and access to functions
run('VLFEATROOT/toolbox/vl_setup');
addpath('hungarian_method/');

imgsd_1 = zeros(480,640,length(imgseq1));
xyz_depth_1 = zeros(480*640,3,length(imgseq1));
rgbd_1 = zeros(480,640,3*length(imgseq1));

%for each image frame of kinect_1 in the directory
%express rgb image in the depth camera reference frame 
for i = 1: length(imgseq1)
    
    %load rgb image and corresponding depth image 
    im = imread(imgseq1(i).rgb);
    load(imgseq1(i).depth);
    
    %compute matrix containing the depth 
    %(in millimiters) of each pixel
    imgsd_1(:,:,i) = double(depth_array);
    
    %find coordinates of with non-zero values
    [r,c] = ind2sub(size(imgsd_1(:,:,i)),find(imgsd_1(:,:,i))); 
    im_vec = reshape(imgsd_1(:,:,i),[480*640,1]); 
    
    %compute the xyz pointCloud for the depth camera 
    xyz_depth_1(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0); 
    
    %express rgb values in the depth camera reference frame 
    [rgbd_1(:,:,i:i+2),u1_temp, v1_temp, xyz_rgb_1_temp] = get_rgbd(xyz_depth_1(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb);
    
    %save the mapping of rgb image to the depth reference frame
    %only for the first image frame (used to perform the matching)
    if i == 1
        
        u1 = u1_temp;
        v1 = v1_temp;
        %xyz_rgb_1 = xyz_rgb_1_temp;
        
    end
    
end

imgsd_2 = zeros(480,640,length(imgseq2));
xyz_depth_2 = zeros(480*640,3,length(imgseq2));
rgbd_2 = zeros(480,640,3*length(imgseq2));

%for each image frame of kinect_2 in the directory
%express rgb image in the depth camera reference frame 
for i = 1: length(imgseq2)
    
    %load rgb image and corresponding depth image
    im = imread(imgseq2(i).rgb);
    load(imgseq2(i).depth);
    
    %compute matrix containing the depth 
    %(in millimiters) of each pixel
    imgsd_2(:,:,i) = double(depth_array);
    
    %find coordinates of pixels with non-zero values
    [r,c] = ind2sub(size(imgsd_2(:,:,i)),find(imgsd_2(:,:,i))); 
    im_vec = reshape(imgsd_2(:,:,i),[480*640,1]); 
    
    %compute the xyz pointCloud for the depth camera 
    xyz_depth_2(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    
    %express rgb values in the depth camera reference frame
    [rgbd_2(:,:,i:i+2),u2_temp,v2_temp,xyz_rgb_2_temp] = get_rgbd(xyz_depth_2(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb); 
    
    %save the mapping of rgb image to the depth reference frame
    %only for the first image frame (used to perform the matching)
    if i == 1 
        
        u2 = u2_temp;
        v2 = v2_temp;
        %xyz_rgb_2 = xyz_rgb_2_temp;
        
    end
    
end

%use only the first image frame to compute
%the transformation between the two kinects
im1=imread(imgseq1(1).rgb);
im2=imread(imgseq2(1).rgb);

%compute features and corresponding descriptors 
[f1,d1] = vl_sift(single(rgb2gray(im1))); 
[f2,d2] = vl_sift(single(rgb2gray(im2)));

% Visualization of keypoints
% figure(1);
% imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
% figure(2);
% imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;

%compute matches between keypoints of both kinects
[matches, scores] = vl_ubcmatch(d1, d2, 2);

% Visualization of matches
% figure(3); clf;
% imagesc(cat(2,im1,im2));

%obtain the coordinates for the matches
%in the rgb camera reference frame 
u1_m = f1(1,matches(1,:));
v1_m = f1(2,matches(1,:));
u2_m = f2(1,matches(2,:));
v2_m = f2(2,matches(2,:));

% u2_m_plot = f2(1,matches(2,:))+size(im1,2);
% 
% hold on;
% h = line([u1_m; u2_m_plot],[v1_m;v2_m]);
% set(h, 'linewidth', 1, 'color', 'b');

%total number of matches 
number_of_matches = size(u1_m,2);

val_1 = zeros(1,number_of_matches);
val_2 = zeros(1,number_of_matches);

depth_index_1 = zeros(1,number_of_matches);
depth_index_2 = zeros(1,number_of_matches);

xyz_matches_1 = zeros(3,number_of_matches);
xyz_matches_2 = zeros(3,number_of_matches);


for i=1:size(u1_m,2)
    
    %find the coordinates of the points matched in the depth reference frame
    [val_1(i), depth_index_1(i)] = min(abs(u1-u1_m(i))+abs(v1-v1_m(i)));
    [val_2(i), depth_index_2(i)] = min(abs(u2-u2_m(i))+abs(v2-v2_m(i)));
    
    %compute the xyz pointclouds of the matched points, for both kinects
    xyz_matches_1(:,i) = [xyz_depth_1(depth_index_1(i),1,1) ; xyz_depth_1(depth_index_1(i),2,1) ; xyz_depth_1(depth_index_1(i),3,1)];
    xyz_matches_2(:,i) = [xyz_depth_2(depth_index_2(i),1,1) ; xyz_depth_2(depth_index_2(i),2,1) ; xyz_depth_2(depth_index_2(i),3,1)];
end

%remove matches of points where a depth camera error occurred (z=0)
index_nonzero = find(xyz_matches_1(3,:) ~= 0);
xyz_matches_1 = xyz_matches_1(:,index_nonzero);
index_nonzero = find(xyz_matches_2(3,:) ~= 0);
xyz_matches_2 = xyz_matches_2(:,index_nonzero);

%use ransac algorithm to compute rotation and translation from kinect_2 to
%world; consider kinect_1 as world
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


%perform background subtraction and identify image components for kinect_1
[im_label_1, num_components_1] = bg_subtraction(length(imgseq1), imgsd_1);

%perform background subtraction and identify image components for kinect_2
[im_label_2, num_components_2] = bg_subtraction(length(imgseq2), imgsd_2);

box_1 = zeros(24,max(num_components_1),length(imgseq1));
box_2 = zeros(24,max(num_components_2),length(imgseq2));

%compute a 3D box for each component of each frame for kinect_1
for i =  1 : length(imgseq1) 
    
    for j = 1 : num_components_1(i)
        
       index = find(im_label_1(:,:,i)==j);
       
       %for the current component, find maximum and minimum x, y and z values 
       M = [min(xyz_depth_1(index,1,i)),min(xyz_depth_1(index,2,i)),min(xyz_depth_1(index,3,i)), max(xyz_depth_1(index,1,i)), max(xyz_depth_1(index,2,i)), max(xyz_depth_1(index,3,i))];
       
       %compute box containing the current component 
       box_1(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
    end
    
end

xyz_2_1 = zeros(480*640,3,length(imgseq2));

%compute a 3D box for each component of each frame for kinect_2
for i =  1 : length(imgseq2)  
    
    %obtain points of the pointcloud of kinect_2 in the world reference frame 
    xyz_2_1(:,:,i) = (R_final_21 * xyz_depth_2(:,:,i)' + repmat(T_final_21,1,480*640))';
    
    for j = 1 : num_components_2(i)
        
       index = find(im_label_2(:,:,i)==j);
       
       %for the current component, find maximum and minimum x, y and z values
       M = [min(xyz_2_1(index,1,i)),min(xyz_2_1(index,2,i)),min(xyz_2_1(index,3,i)), max(xyz_2_1(index,1,i)), max(xyz_2_1(index,2,i)), max(xyz_2_1(index,3,i))];
      
       %compute box containing the current component
       box_2(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
                  
    end
    
end

assigns = [];

%match components from the two kinects belonging to the same frame
%from the matching results compute a box of common components 
%initialize the common box matrix as the box matrix from kinect_1
box_common = cat(2, box_1, zeros(24, max(num_components_2), length(imgseq2)));

%initialize total number of components
%as the number of components of kinect_1
num_components_t = num_components_1;

%for each frame, compare components of the two kinects
for i = 1 : length(imgseq1)
    
    assign = [];
    
    %remove zero values from the box matrices, which correspond
    %to components that don't exist in the current frame
    box_i2 = reshape(nonzeros(box_2(:,:,i)), 3*8, num_components_2(i)); 
    box_i1 = reshape(nonzeros(box_1(:,:,i)), 3*8, num_components_1(i));
    box_i1 = reshape(box_i1, 24*num_components_1(i),1);
    
    if ~isempty(box_i2) && ~isempty(box_i1)
        
        %compute euclidean distance between the normalized box descriptors 
        %of all pairs of components from frame i of both kinects
        difBox = repmat(box_i2,num_components_1(i),1) - repmat(box_i1,1,num_components_2(i));
        normBox = sqrt(sum((reshape(difBox, 24, num_components_1(i)*num_components_2(i))).^2, 1)); 

        %compute cost matrix (using only box proximity)
        costmat = reshape(normBox, num_components_1(i), num_components_2(i));

        %prevent that the Hungarian method chooses
        %matches with very high cost 
        costmat(find(costmat > 100))=1e+10; 

        %remove rows in which all the costs are too high
        index_r = find(all(costmat > 1e+9, 2));
        costmat(index_r, :) = [];
        
        %remove columns in which all the costs are too high
        index_c = find(all(costmat > 1e+9, 1));
        costmat(:, index_c) = [];

        %implement Hungarian Method to determine the assignment
        [assign, cost] = munkres(costmat);

        %correct non-empty assign arrays for the removed rows and columns
        if ~isempty(assign)
            
            %correct assign index for the removed rows
            if ~isempty(index_r)
                aux = zeros(1, num_components_t(i));
                aux(1, index_r) = 1;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            %correct assign index for the removed columns
            aux = zeros(1, num_components_t(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
            
        end
        
        box_i1 = reshape(box_i1, 3*8, num_components_1(i));

        for k = 1 : length(assign)
            
            %update matrix containing assigns for all frames 
            assigns(k,i) = assign(k);
            
            if assigns(k,i) ~= 0
                
                %for each pair of components assign between the kinects,
                %compute minimum values (in x, y, z) of the pointcloud 
                %that contains the supperposition of both components 
                x_min = min(box_i1(1,k), box_i2(1,assign(k)));
                y_min = min(box_i1(2,k), box_i2(2,assign(k)));
                z_min = min(box_i1(3,k), box_i2(3,assign(k)));
                
                %for each pair of components assign between the kinects,
                %compute maximum values (in x, y, z) of the pointcloud 
                %that contains the merging of both components 
                x_max = max(box_i1(19,k), box_i2(19,assign(k)));
                y_max = max(box_i1(20,k), box_i2(20,assign(k)));
                z_max = max(box_i1(21,k), box_i2(21,assign(k)));
                
                
                M = [x_min,y_min,z_min,x_max,y_max,z_max];
                
                %compute box containing the merging of both components
                box_common(:,k,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                          M(1), M(5), M(6), M(1), M(5), M(3)...
                          M(4), M(2), M(3), M(4), M(2), M(6)...
                          M(4), M(5), M(6), M(4), M(5), M(3)];
            end

        end 
        
    end
    
    if length(assign) < num_components_2(i)
            
            %add to the matrix containing the boxes from both kinects
            %the components in kinect_2 that were not assign to any 
            %component of kinect_1, in the current frame 
            new = setdiff(1:num_components_2(i),assign);
            box_common(:,new + max(num_components_1),i) = box_2(:,new,i);
            
            %update the total number of components from both boxes 
            num_components_t(i) = num_components_t(i) + length(new);
    end
    
end

%place an array with the total number of components of 
%the first frame as the first column of the tracking matrix
track(:,1) = 1:num_components_t(1);

%compare components of consecutive frames using the 
%box matrix derived from the merging of the two kinects
for i = 1 : length(imgseq1)-1
    
    j = i + 1;
    assign = [];
    
    %remove zero values from the box matrices, which correspond
    %to components that don't exist in the current frame
    box_j = reshape(nonzeros(box_common(:,:,j)), 3*8, num_components_t(j)); %nonzeros eliminate zeros -> no component
    box_i = reshape(nonzeros(box_common(:,:,i)), 3*8, num_components_t(i));
    box_i = reshape(box_i, 24*num_components_t(i),1);
    
    if ~isempty(box_j) && ~isempty(box_i) 
        
        %compute euclidean distance between the normalized box descriptors 
        %of all pairs of components from frame i and frame i+1
        difBox = repmat(box_j,num_components_t(i),1) - repmat(box_i,1,num_components_t(j));
        normBox = sqrt(sum((reshape(difBox, 24, num_components_t(i)*num_components_t(j))).^2, 1));
    
        %compute cost matrix (using only box proximity)
        costmat = reshape(normBox, num_components_t(i), num_components_t(j));
    
        %prevent that the Hungarian method chooses
        %matches with very high cost 
        costmat(find(costmat > 100))=1e+10;
        
        %remove rows in which all the costs are too high
        index_r = find(all(costmat > 1e+9, 2));
        costmat(index_r, :) = [];
        
        %remove columns in which all the costs are too high 
        index_c = find(all(costmat > 1e+9, 1));
        costmat(:, index_c) = [];
        
        %implement Hungarian Method to determine the assignment
        [assign, cost] = munkres(costmat);
        
        %correct non-empty assign arrays for the removed rows and columns
        if ~isempty(assign)
            
            %correct assign index for the removed rows
            if ~isempty(index_r)
                aux = zeros(1, num_components_t(i));
                aux(1, index_r) = 1;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            %correct assign values for the removed columns
            aux = zeros(1, num_components_t(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
        end
        
        %put the assignments into the tracking matrix 
        for index = 1:length(assign)
            
            %track only for assign non zero
            if assign(index) ~= 0
                indextrack = find(track(:,i)==index);
                track(indextrack,j) = assign(index);
            end
            
        end
        
    else
        
        %add a column of zeros to the track matrix if
        %there are no assigns for the current frame
        track =  cat(2,track,zeros(size(track,1),1));
        
    end
    
    if length(assign) < num_components_t(j)
        
        %add a new object to the track matrix if number
        %of components of the current frame is greater
        %than number of components of the previous frame        
        new = setdiff(1:num_components_t(j),assign);
        track=cat(1,track,cat(2,zeros(length(new),size(track,2)-1),new'));
        
    end
      
end  

%prepare output
for j = 1 : size(track,1)
    objects(j).frames_tracked=[];
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
            objects(j).frames_tracked = cat(2, objects(j).frames_tracked,i);
        end
    end
end

cam2toW.R = R_final_21;
cam2toW.T = T_final_21;

% j= 1;
% for i = 1:3:3*length(imgseq1)
%     pc1=pointCloud(xyz_depth_1(:,:,j),'Color',reshape(rgbd_1(:,:,i:i+2),[480*640 3]));
%     pc2=pointCloud(xyz_2_1(:,:,j),'Color',reshape(rgbd_2(:,:,i:i+2),[480*640 3]));
%     figure(1);hold off;
%     %showPointCloud(pc1)
%     pcshow(pcmerge(pc1,pc2,0.001));
%     drawnow;
%     j = j+1;
% end

% for i=1:length(imgseq1)
%     im1=imread(imgseq1(i).rgb);
%     im2=imread(imgseq2(i).rgb);
%     load(imgseq1(i).depth)
%     dep1=depth_array;
%     load(imgseq2(i).depth)
%     dep2=depth_array;
%     xyz1=get_xyz_asus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
%     xyz2=get_xyz_asus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
%     [rgbd1,u,v,xyz_rgb1] = get_rgbd(xyz1, im1, cam_params.R, cam_params.T, cam_params.Krgb);
%     [rgbd2,u,v,xyz_rgb2] = get_rgbd(xyz2, im2, cam_params.R, cam_params.T, cam_params.Krgb);
%     pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
%     pc2=pointCloud(xyz2*cam2toW.R+cam2toW.T,'Color',reshape(rgbd2,[480*640 3]));
%     figure(1);hold off;
%     %showPointCloud(pc1)
%     pcshow(pcmerge(pc1,pc2,0.001));
%     drawnow;
% end

end

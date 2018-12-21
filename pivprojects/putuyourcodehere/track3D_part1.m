function objects = track3D_part1(imgseq1, cam_params)
%clear all
%close all 

%prepare parameters and access to functions
addpath('hungarian_method/');

imgsd = zeros(480,640,length(imgseq1));
xyz_depth = zeros(480*640,3,length(imgseq1));
rgbd = zeros(480,640,3*length(imgseq1));

%for each image frame in the directory
%express rgb image in the depth camera reference frame 
for i = 1: length (imgseq1)
    
    %load rgb image and corresponding depth image 
    im = imread(imgseq1(i).rgb);    
    load(imgseq1(i).depth);
    
    %matrix containing the depth (in millimiters) of each pixel
    imgsd(:,:,i) = double(depth_array);
    
    %find coordinates of pixels with non-zero values 
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); 
    im_vec = reshape(imgsd(:,:,i),[480*640,1]);
    
    %compute the xyz pointCloud for the depth camera 
    xyz_depth(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    
    %express rgb values in the depth camera reference frame 
    [rgbd(:,:,i:i+2), u_temp, v_temp, xyz_rgb_temp] = get_rgbd(xyz_depth(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb);

end

%perform background subtraction and identify image components
[im_label, num_components] = bg_subtraction(length(imgseq1), imgsd);

colour = zeros(15*2,max(num_components),length(imgseq1));
box = zeros(3*8,max(num_components),length(imgseq1));

for i =  1 : length(imgseq1)
    
    %obtain hue and saturation from the rgb image
    [hue, sat, int] = rgb2hsv(rgbd(:,:,i:i+2)); 
    hue = reshape(hue, 480*640,1);
    sat = reshape(sat, 480*640,1);
    hold all
    
    %for each component of each image frame 
    %compute hue and saturation histograms
    %and compute a 3D box 
    for j = 1 : num_components(i)
        
       indx = find(im_label(:,:,i)==j);
       
       %for the current component, find maximum and minimum x, y and z values
       M = [min(xyz_depth(indx,1,i)),min(xyz_depth(indx,2,i)),min(xyz_depth(indx,3,i)), max(xyz_depth(indx,1,i)), max(xyz_depth(indx,2,i)), max(xyz_depth(indx,3,i))];
       
       %compute box containing the current component
       box(:,j,i) = [M(1), M(2), M(3), M(1), M(2), M(6)... 
                      M(1), M(5), M(6), M(1), M(5), M(3)...
                      M(4), M(2), M(3), M(4), M(2), M(6)...
                      M(4), M(5), M(6), M(4), M(5), M(3)];
       
       %compute hue and saturation histograms for the current component 
       histHue = histogram(hue(indx),15); histSat = histogram(sat(indx),15);
       
       %obtain a colour descriptor for the current component 
       colour(:,j,i) = cat(2,histHue.Values,histSat.Values)';
       
       %remove zero values from the colour descriptors 
       %to adjust the descriptors to the tracking algorithm 
       if isempty(find(colour(:,j,i)==0)) == 0
        colour(find(colour(:,j,i)==0),j,i)=-1;
       end
       
    end
    
end


%store sequence of components along the images,
%using components of the first image as a starting point
track(:,1) = 1:num_components(1);

%compare components of consecutive frames
for i = 1 : length(imgseq1)-1
    
    j = i + 1; 
    assign = [];
    
    %remove zero values from the box matrices, which correspond
    %to components that don't exist in the current frame
    box_j = reshape(nonzeros(box(:,:,j)), 3*8, num_components(j)); 
    box_i = reshape(nonzeros(box(:,:,i)), 3*8, num_components(i));
    
    %normalize box descriptors for cost function 
    box_i_norm = box_i./sqrt(sum(box_i.^2));
    box_j_norm = box_j./sqrt(sum(box_j.^2));
    
    box_i = reshape(box_i, 24*num_components(i),1);
    box_i_norm = reshape(box_i_norm, 24*num_components(i),1);
    
    %remove zero values from the colour matrices, which correspond
    %to components that don't exist in the current frame 
    colour_j = reshape(nonzeros(colour(:,:,j)), 30, num_components(j));
    colour_i = reshape(nonzeros(colour(:,:,i)), 30, num_components(i));
     
    %normalize colour descriptors for cost function 
    colour_i_norm = colour_i./sqrt(sum(colour_i.^2));
    colour_j_norm = colour_j./sqrt(sum(colour_j.^2));
    
    colour_i = reshape(colour_i, 30*num_components(i),1);
    colour_i_norm = reshape(colour_i_norm, 30*num_components(i),1);
    
    if ~isempty(box_j) && ~isempty(box_i)
           
        %compute euclidean distance between the normalized box descriptors 
        %of all pairs of components from frame i and frame i+1
        difBox = repmat(box_j_norm,num_components(i),1) - repmat(box_i_norm,1,num_components(j));
        normBox = sqrt(sum((reshape(difBox, 24, num_components(i)*num_components(j))).^2, 1));
        
        %compute euclidean distance between the normalized colour 
        %descriptors of all pairs of components from frame i and frame i+1
        difColour = repmat(colour_j_norm,num_components(i),1) - repmat(colour_i_norm,1,num_components(j));
        normColour = sqrt(sum((reshape(difColour, 30, num_components(i)*num_components(j))).^2,1));  

        %compute cost matrix (using box proximity and colour)
        costmat = reshape(normBox, num_components(i), num_components(j));
        costmat = costmat + reshape(normColour,num_components(i), num_components(j));
        
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
                aux = ones(1, num_components(i));
                aux(1, index_r) = 0;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            %correct assign values for the removed columns
            aux = zeros(1, num_components(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
        end

        %put the assignments into the tracking matrix 
        for index = 1 : length(assign)
            
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
    
    if length(assign) < num_components(j)
        
        %add a new object to the track matrix if number
        %of components of the current frame is greater
        %than number of components of the previous frame 
        new = setdiff(1:num_components(j),assign);
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
            objects(j).X = cat(1, objects(j).X, box(1:3:end-2,track(j,i),i)');
            objects(j).Y = cat(1, objects(j).Y, box(2:3:end-1,track(j,i),i)');
            objects(j).Z = cat(1, objects(j).Z, box(3:3:end,track(j,i),i)');
            objects(j).frames_tracked = cat(2, objects(j).frames_tracked,i);
        end
    end
end

% Plot the boxes 
% for j = 1 : size(track,1)
%     for i = 1 : length(objects(j).framestracked)
%         frame = objects(j).framestracked(i);
%         figure(1)
%         pcshow(xyz_depth(:,:,frame),'VerticalAxis','Z');
%         hold on 
%         plot3(objects(j).X(i,:)' , objects(j).Y(i,:)' , objects(j).Z(i,:)' , 'r*');
%         axis([min(xyz_depth(:,1,frame)) max(xyz_depth(:,1,frame)) min(xyz_depth(:,2,frame)) max(xyz_depth(:,2,frame)) min(xyz_depth(:,3,frame)) max(xyz_depth(:,3,frame))])
%         hold off
%     end
% end
end
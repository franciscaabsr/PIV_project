function objects = track3D_part1(imgseq1, cam_params)
%clear all
%close all 

% Prepare parameters and access to functions

addpath('hungarian_method/');

%imgs = []; NAO PRECISAMOS DISTO
imgsd = zeros(480,640,length(imgseq1));
xyz_depth = zeros(480*640,3,length(imgseq1));
rgbd = zeros(480,640,3*length(imgseq1));

% Computation of rgbd and xyz (rgbd: rgb image expressed in depth reference frame)

for i = 1: length (imgseq1)
    im = imread(imgseq1(i).rgb);
    %imgs = cat(3,imgs,im); 
    load(imgseq1(i).depth);
    imgsd(:,:,i) = double(depth_array); % depth_array in mm
    % select (i,j) that correspond to entries non zero
    [r,c] = ind2sub(size(imgsd(:,:,i)),find(imgsd(:,:,i))); 
    im_vec = reshape(imgsd(:,:,i),[480*640,1]);
    % compute xyz in depth reference frame
    xyz_depth(:,:,i) = get_xyz_asus(im_vec, [480, 640], [r, c], cam_params.Kdepth, 1, 0);
    % compute rgb corresponding to xyz_depth
    [rgbd(:,:,i:i+2), u_temp, v_temp, xyz_rgb_temp] = get_rgbd(xyz_depth(:,:,i), im, cam_params.R, cam_params.T, cam_params.Krgb);
end

% Background subtraction and identification of components
[im_label, num_components] = bg_subtraction(length(imgseq1), imgsd);

colour = zeros(15*2,max(num_components),length(imgseq1));
% each box is composed by 8 points, each with (x,y,z)
box = zeros(3*8,max(num_components),length(imgseq1));

for i =  1 : length(imgseq1)
    [hue, sat, int] = rgb2hsv(rgbd(:,:,i:i+2)); % with RBGD????
    hue = reshape(hue, 480*640,1);
    sat = reshape(sat, 480*640,1);
    hold all
    
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
    
end

% Tracking of "objects" identified with background subtraction

% to store sequence of components along the images, with components of first
% image as a starting point

track(:,1) = 1:num_components(1);

% compare components of image pair by pair

for i = 1 : length(imgseq1)-1
    j = i + 1;
    assign = [];
    % to compare all combinations of components from the images i and i+1 
    box_j = reshape(nonzeros(box(:,:,j)), 3*8, num_components(j)); % nonzeros eliminate zeros -> no component
    box_i = reshape(nonzeros(box(:,:,i)), 3*8, num_components(i));
    box_i = reshape(box_i, 24*num_components(i),1);
    
    colour_j = reshape(nonzeros(colour(:,:,j)), 30, num_components(j));
    colour_i = reshape(nonzeros(colour(:,:,i)), 30, num_components(i));
    colour_i = reshape(colour_i, 30*num_components(i),1);
    
    if ~isempty(box_j) && ~isempty(box_i)
        % compute norm of difference between the two images, for box (24
        % values) and for colour (30 values)
        difBox = repmat(box_j,num_components(i),1) - repmat(box_i,1,num_components(j));
        normBox = sqrt(sum((reshape(difBox, 24, num_components(i)*num_components(j))).^2, 1));
        %normBox = vecnorm(reshape(difBox, 24, num_components(i)*num_components(j)));

        difColour = repmat(colour_j,num_components(i),1) - repmat(colour_i,1,num_components(j));
        normColour = sqrt(sum((reshape(difColour, 30, num_components(i)*num_components(j))).^2,1));
        %normColour = vecnorm(reshape(difColour, 30, num_components(i)*num_components(j)));   

        % cost matrix (using proximity and colour)
        costmat = reshape(normBox, num_components(i), num_components(j));
        costmat = costmat + reshape(normColour,num_components(i), num_components(j));
        
        % if cost too high -> not to be chosen by Hungarian Method
        ind = find(costmat > 1e+06); %VERIFY THRESHOLD
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
                aux = zeros(1, num_components(i));
                aux(1, index_r) = 1;
                aux(logical(aux)) = assign;
                assign = aux;
            end

            % correct assign values considering columns removed
            aux = zeros(1, num_components(i));
            for k = 1:length(index_c)
                ind = find(assign >= index_c(k));
                aux(ind) = 1;
                assign = assign + aux;
            end
        end

        for index = 1 : length(assign)
            %track only for assign non zero
            if assign(index) ~= 0
                indextrack = find(track(:,i)==index);
                track(indextrack,j) = assign(index);
            end
        end
    else
        track =  cat(2,track,zeros(size(track,1),1));
    end
    
    if length(assign) < num_components(j)
        new = setdiff(1:num_components(j),assign);
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
            objects(j).X = cat(1, objects(j).X, box(1:3:end-2,track(j,i),i)');
            objects(j).Y = cat(1, objects(j).Y, box(2:3:end-1,track(j,i),i)');
            objects(j).Z = cat(1, objects(j).Z, box(3:3:end,track(j,i),i)');
            objects(j).framestracked = cat(2, objects(j).framestracked,i);
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
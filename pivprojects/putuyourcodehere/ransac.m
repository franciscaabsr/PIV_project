function [R_final, T_final, max_inliers] = ransac(xyz1, xyz2)

max_inliers = [];

    for i=1:100
        %select 4 pairs of values from the total number of matches
        index_sample = randsample((1:size(xyz1,2)), 4);

        %compute centroid for 4 pairs
        cent1=mean(xyz1(:,index_sample)')';
        cent2=mean(xyz2(:,index_sample)')';

        xyz_1=xyz1(:,index_sample)-repmat(cent1,1,4);
        xyz_2=xyz2(:,index_sample)-repmat(cent2,1,4);

        %fit model for the selected 4 pairs of xyz by
        %solving the Procrustes problem to find the rotation (R) 
        %and translation (T) from kinect 2 to world
        %consider kinect_1 as world
        [u s v]=svd(xyz_1*xyz_2');
        R_21 = u*v'; 
        T_21 = cent1 - R_21*cent2; 

        %find inliers and outliers for the model fitted to the 4 pairs 
        error = (xyz1 - (R_21*xyz2 + repmat(T_21,1,size(xyz2,2))));
        
        %inliers = 1; outliers = 0;
        inliers = sum(error.*error)<(0.3^2) ;
        
        if sum(inliers) > size(max_inliers)
            
            %update maximum number of inliers if the model's
            %number of inliers exceeds the previous maximum 
            max_inliers = find(inliers);
            
        end
        
    end 

%use only the inliers returned by the model with maximum 
%number of inliers to solve the Procrustes problem 

%determine centroid of pointCloud of inliers in kinect_1
cent1=mean(xyz1(:,max_inliers)')';
xyz_1=xyz1(:,max_inliers)-repmat(cent1,[1,size(max_inliers,2)]);

%determine centroid of pointCloud of inliers in kinect_2
cent2=mean(xyz2(:,max_inliers)')';
xyz_2=xyz2(:,max_inliers)-repmat(cent2,[1,size(max_inliers,2)]);

%fit model for the pointcloud of inliers by
%solving the Procrustes problem to find the rotation (R) 
%and translation (T) from kinect 2 to world
%consider kinect_1 as world
[u s v]=svd(xyz_1*xyz_2');
R_final = u*v'; 
T_final = cent1-R_final*cent2; 

end
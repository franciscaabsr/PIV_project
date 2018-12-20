function [R_final, T_final, max_inliers] = ransac(xyz1, xyz2)

max_inliers = [];

    for i=1:100
        %select 4 pairs of values of the number of matches we have
        index_sample = randsample((1:size(xyz1,2)), 4);

        %compute centroid for 4 pairs
        cent1=mean(xyz1(:,index_sample)')';
        cent2=mean(xyz2(:,index_sample)')';

        xyz_1=xyz1(:,index_sample)-repmat(cent1,1,4);
        xyz_2=xyz2(:,index_sample)-repmat(cent2,1,4);

        %apply SVD to determine rotation matrix from 2 to 1 considering kinect_1 as the world
        [u s v]=svd(xyz_1*xyz_2');
        R_21 = u*v'; %rotation matrix
        T_21 = cent1 - R_21*cent2; %translation
        %model fitted for these 4 pairs of xyz

        %find inliers and outliers
        error = (xyz1 - (R_21*xyz2 + repmat(T_21,1,size(xyz2,2))));
        inliers = sum(error.*error)<(0.3^2) ; %1 corresponds to inlier and 0 corresponds to outlier
        if sum(inliers) > size(max_inliers)
            max_inliers = find(inliers);
        end
    end 

%inds_inliers = find(inds_classification);
%we have the inliers saved in max_inliers

%determine centroid of each pointcloud to subtract it
%imagesc(xyz_matches_1(:,inds_inliers));
cent1=mean(xyz1(:,max_inliers)')';
cent2=mean(xyz2(:,max_inliers)')';
xyz_1=xyz1(:,max_inliers)-repmat(cent1,[1,size(max_inliers,2)]);
xyz_2=xyz2(:,max_inliers)-repmat(cent2,[1,size(max_inliers,2)]);

%apply SVD to determine rotation matrix from 2 to 1 considering kinect_1 as the world
[u s v]=svd(xyz_1*xyz_2');
R_final = u*v'; %rotation matrix
T_final = cent1-R_final*cent2; %translation

end
function [R_final, T_final] = ransac(xyz1, xyz2, num_outliers, noise)

%add outliers to the xyz2 to be removed by the ransac
if num_outliers ~= 0
   aux = randperm(size(xyz1,2),num_outliers);
   for i=1:size(aux,2)
       xyz2(:,aux(i)) = rand(3,1)*1000;
   end
end

%add noise to xyz2 to check if it still works
if noise == 1
    for i=1:size(xyz2,2)
        xyz2(:,i) = xyz2(:,i) + rand(3,1)*1.0e-04;
    end
end

max_inliers = [];

for i=1:100
    %select 4 pairs of values of the 200 we have
    index_sample = randsample((1:size(xyz1,2)), 4);
    
    %compute centroid for 4 pairs
    cent1=mean(xyz1(:,index_sample)')';
    cent2=mean(xyz2(:,index_sample)')';
    
    xyz_1=xyz1(:,index_sample)-repmat(cent1,1,4);
    xyz_2=xyz2(:,index_sample)-repmat(cent2,1,4);
    
    %apply SVD to determine rotation matrix from 2 to 1 considering kinect_1 as the world
    [u s v]=svd(xyz_2*xyz_1');
    R_12 = u*v'; %rotation matrix
    T_12 = cent2 - R_12*cent1; %translation
    %model fitted for these 4 pairs of xyz
    
    %find inliers and outliers
    error = (xyz2 - (R_12*xyz1 + repmat(T_12,1,size(xyz1,2))));
    inliers = sum(error.*error)<(0.5^2) ; %1 corresponds to inlier and 0 corresponds to outlier
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
[u s v]=svd(xyz_2*xyz_1');
R_final = u*v'; %rotation matrix
T_final = cent2-R_final*cent1; %translation

end
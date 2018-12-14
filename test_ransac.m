%TEST RANSAC

clear all

%generate randomly xyz for 200 points, with values between 0 and 1
xyz1 = rand(3,200);

%generate random matrix 3x3, to apply SVD and obtain random R
[u s v] = svd(rand(3,3));
R = u*v'; %rotation matrix

%generate random translation
T = rand(3,1);

%compute xyz' using rigid motion: xyz' = R*xyz + T
xyz2 = R*xyz1 + T;

%% First test ransac without outliers and noise

[R_final, T_final, max_inliers] = ransac(xyz1,xyz2);

%error_rotation = vecnorm(R-R_final)
%error_translation = vecnorm(T-T_final)

%we reached the same results for R and T, without outliers
%% Second, test ransac with outliers, but without noise

% 20% of outliers -> 40 points
num_outliers = 0.2*size(xyz2,2);
%add outliers to the xyz2 to be removed by the ransac

aux = randperm(size(xyz2,2),num_outliers);
for i=1:size(aux,2)
    xyz2(:,aux(i)) = rand(3,1)*1000;
end

[R_final_outliers, T_final_outliers, max_inliers] = ransac(xyz1, xyz2);

%error_rotation = vecnorm(R-R_final_outliers)
%error_translation = vecnorm(T-T_final_outliers)

%we reached the same results for R and T, without outliers
%% Finally, test ransac with outliers and noise

% 20% of outliers -> 40 points and noise == true

%add noise to xyz2 to check if it still works
for i=1:size(xyz2,2)
    xyz2(:,i) = xyz2(:,i) + rand(3,1)*1.0e-04;
end

[R_final_outliers_noise, T_final_outliers_noise, max_inliers] = ransac(xyz1, xyz2);

%error_rotation = vecnorm(R-R_final_outliers_noise)
%error_translation = vecnorm(T-T_final_outliers_noise)
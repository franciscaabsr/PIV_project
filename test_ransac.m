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

[R_final, T_final] = ransac(xyz1,xyz2,0,0);

%error_rotation = vecnorm(R-R_final)
%error_translation = vecnorm(T-T_final)

%we reached the same results for R and T, without outliers
%% Second, test ransac with outliers, but without noise

% 20% of outliers -> 40 points
num_outliers = 0.2*size(xyz1,2);

[R_final_outliers, T_final_outliers] = ransac(xyz1, xyz2, num_outliers, 0);

%error_rotation = vecnorm(R-R_final_outliers)
%error_translation = vecnorm(T-T_final_outliers)

%we reached the same results for R and T, without outliers
%% Finally, test ransac with outliers and noise

% 20% of outliers -> 40 points and noise == true
num_outliers = 0.2*size(xyz1,2);

[R_final_outliers_noise, T_final_outliers_noise] = ransac(xyz1, xyz2, num_outliers, 1);

%error_rotation = vecnorm(R-R_final_outliers_noise)
%error_translation = vecnorm(T-T_final_outliers_noise)
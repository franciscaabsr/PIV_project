function xyz_ret = get_xyz_asus(im_vec, im_orig_size, good_inds, K, alpha, beta)
% im_vec - depth image vectorized (Nx1)
% im_orig_size - original image size (HxW) : [H, W]
% goot_inds - indexes of the image that are valid, i.e., different from 0.

persistent u;
persistent v;
persistent im_size;
persistent xyz;
persistent z;

%intrinsic paramenters
Kx = K(1,1);
Cx = K(1,3);
Ky = K(2,2);
Cy = K(2,3);

if isempty(im_size)
    %     im_size = size(im);
    im_size = im_orig_size;
    
    u = repmat(1:im_size(2),im_size(1),1); %1 to 640 vector for all 480 rows
    u = u(:)-Cx; %subtract for each entry of matrix; u is in the principal point ref frame
    v = repmat((1:im_size(1)'),im_size(2),1); %1 to 480 vector' for all 640 rows
    v=v(:)-Cy; 
    xyz=zeros(length(u),3); 
end

% tmp = im(:);
xyz(:,3) = double(im_vec)*0.001; % Convert to meters
xyz(good_inds,3) = alpha*xyz(good_inds,3) + beta; %alpha = 1 beta = 0
xyz(:,1) = (xyz(:,3)/Kx) .* u ;
xyz(:,2) = (xyz(:,3)/Ky) .* v;

%plot3(x,y,z,'.');axis equal
xyz_ret = xyz;

end
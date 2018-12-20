function [im_label, num_components] = bg_subtraction(num_iter, imgsd)
    %compute median of every pixel in terms of time (third dimension in list of matrices)
    bgdepth = median(imgsd,3);
    
    im_label = zeros(480,640,num_iter);
    num_components = zeros(1, num_iter);
    
    for i=1:num_iter

        %compute the gradient of the depth image to obtain the outlines
        [im_x_grad, im_y_grad] = gradient(imgsd(:,:,i));
        im_gradient = sqrt(im_x_grad.^2 + im_y_grad.^2);
        im_outlines = abs(im_gradient) < 150; %outlines = 0, non outlines = 1
        %find the points where a depth camera error occurred (z=0)
        im_errors = imgsd(:,:,i) ~= 0;%error = 0, valid px = 1

        imdiff=abs(imgsd(:,:,i)-bgdepth)>200; 
        %subtracting the background to the image in order to obtain moving
        %objects: 1 foreground, 0 background

        mask = im_outlines.*im_errors;
        mask = reshape( mask,480,640);
        imdiff = imdiff.*mask; 

        % with each image obtained, we can see that these images have white contours that are actually background 
        % so we can "clean" these, considering the neighbourhood of each pixel
        imdiff = imerode(imdiff, strel('disk',2));
        %ignore objects with less than 1000 pixels
        imdiff = bwareaopen(imdiff,1000);


        %store for each image: matrix with labels and number of components
        [im_label(:,:,i), num_components(i)] = bwlabel(imdiff);

        %figure(2);
        %imagesc(im_label(:,:,i));

        %remove components that are full of holes, more thatn 50% non component pixels in the area 
        for j = 1 : num_components(i)
            comp = im_label(:,:,i)==j;
            fill_comp = imdilate(comp, strel('disk',2));
            holes_indx = find(fill_comp-comp == 1);

            if length(holes_indx) > 0.5*length(find(comp == 1))
                filterd_comp = reshape(im_label(:,:,i), 480*640, 1);
                filterd_comp(comp) = 0;
                im_label(:,:,i) = reshape(filterd_comp, 480,640);
            else
                filled_indx = find(imfill(comp, 'holes')~=0);
                filled_im = reshape(im_label(:,:,i), 480*640, 1);
                filled_im(filled_indx) = 1;
                im_label(:,:,i) = reshape(filled_im, 480,640).*im_errors;
            end 
            [im_label(:,:,i), num_components(i)] = bwlabel(im_label(:,:,i));
        end

%         figure(1);
%         imagesc(im_label(:,:,i));
%         title('Connected components');
    end

end
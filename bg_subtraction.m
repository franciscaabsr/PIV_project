function [im_label, num_components] = bg_subtraction(num_iter, imgsd)
    
    %compute median of every pixel across all frames
    bgdepth = median(imgsd,3);
    
    im_label = zeros(480,640,num_iter);
    num_components = zeros(1, num_iter);
    
    for i=1:num_iter

        %compute the gradient of the depth image to obtain the outlines
        [im_x_grad, im_y_grad] = gradient(imgsd(:,:,i));
        im_gradient = sqrt(im_x_grad.^2 + im_y_grad.^2);
        
        %outlines = 0, non outlines = 1
        im_outlines = abs(im_gradient) < 150; 
       
        %find the points where a depth camera error occurred (z=0)
        %error = 0, valid px = 1
        im_errors = imgsd(:,:,i) ~= 0;

        %subtract the background to the image to obtain moving objects
        %1 foreground, 0 background
        imdiff=abs(imgsd(:,:,i)-bgdepth)>200; 

        %delete outlines and camera errors from the object image 
        mask = im_outlines.*im_errors;
        mask = reshape(mask,480,640);
        imdiff = imdiff.*mask; 
        
        %clean objects with structuring element strel of radius 2
        imdiff = imerode(imdiff, strel('disk',2));
        
        %ignore objects with less than 1000 pixels
        imdiff = bwareaopen(imdiff,1000);

        %store labels and number of components for each frame 
        [im_label(:,:,i), num_components(i)] = bwlabel(imdiff);
        

        %remove components that have #holes > 0.5 #pixels
        for j = 1 : num_components(i)
            
            comp = im_label(:,:,i)==j;
            
            %dilate object  with structuring element strel of radius 2
            fill_comp = imdilate(comp, strel('disk',2));
            
            %find number of pixels filled by dilating the object (#holes)
            holes_indx = find(fill_comp-comp == 1);
            
            if length(holes_indx) > 0.5*length(find(comp == 1))
                
                %remove objects that have #holes > 0.5 #pixels
                filterd_comp = reshape(im_label(:,:,i), 480*640, 1);
                filterd_comp(comp) = 0;
                im_label(:,:,i) = reshape(filterd_comp, 480,640);
                
            else
                
                %fill holes of objects that have #holes =< 0.5 #pixels
                filled_indx = find(imfill(comp, 'holes')~=0);
                filled_im = reshape(im_label(:,:,i), 480*640, 1);
                filled_im(filled_indx) = 1;
                
                %delete again points with camera errors (z=0)
                %to delete possible errors that were filled by imfill
                im_label(:,:,i) = reshape(filled_im, 480,640).*im_errors;
                
            end 
            
            %update image with labels and number of components for each frame 
            [im_label(:,:,i), num_components(i)] = bwlabel(im_label(:,:,i));
            
        end
        
    end

end
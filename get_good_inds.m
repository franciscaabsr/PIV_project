function good_inds = get_good_inds(imgsd) 

good_inds = [];
for i = 1:size(imgsd,1)
    for j = 1:size(imgsd,2)
        if imgsd(i,j) ~= 0
            good_inds = cat(1,good_inds, [i j]); 
        end
    end
end 
    
end 
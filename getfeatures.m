im1=imread('rgb_image1_1.png');
im2=imread('rgb_image2_1.png');
[f1,d1]=vl_sift(single(rgb2gray(im1)));
[f2,d2]=vl_sift(single(rgb2gray(im2)));
figure(1);
imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
figure(2);
imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;

[matches, scores] = vl_ubcmatch(d1, d2) ;

figure(3); clf;
imagesc(cat(2,im1,im2));

%obtain the coordinates (u,v) for the matches in the original pair of rgb
u1_m = f1(1,matches(1,:));
v1_m = f1(2,matches(1,:));
u2_m = f2(1,matches(2,:));
v2_m = f2(2,matches(2,:));

u2_m_plot = f2(1,matches(2,:))+size(im1,2);

hold on;
h = line([u1_m; u2_m_plot],[v1_m;v2_m]);
set(h, 'linewidth', 1, 'color', 'b');
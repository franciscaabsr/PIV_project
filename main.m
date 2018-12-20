% Prepare parameters and access to functions
clear all
close all

load ('./CalibData/cameraparametersAsus.mat');
addpath('natsortfiles/');

% Insert data in array of structures

r1 = dir ('./rgb_image1_*'); %lists the files in directory .png (all the images)
d1 = dir('./depth1_*'); %lists the files in directory .mat (all the depth images)
r2 =  dir ('./rgb_image2_*'); %lists the files in directory .png (all the images)
d2 = dir('./depth2_*'); %lists the files in directory .mat (all the depth images)

% Sort in alphanumeric order
r1_sorted = natsortfiles({r1.name})';
d1_sorted = natsortfiles({d1.name})';
r2_sorted = natsortfiles({r2.name})';
d2_sorted = natsortfiles({d2.name})';

% Add everyhting to the imgseq arrays for both cameras
for i=1:length(r1)
    imgseq1(i).rgb= char(r1_sorted(i));
    imgseq2(i).rgb= char(r2_sorted(i));
    imgseq1(i).depth= char(d1_sorted(i));
    imgseq2(i).depth= char(d2_sorted(i));
end

prompt = 'Insert 1 if you wish to run project for 1 camera or 2 if you wish to run for 2 cameras: ';
choice = input(prompt);

if choice == 1
    objects = track3D_part1(imgseq1, cam_params);
elseif choice == 2
    [objects, cam2toW] = track3D_part2(imgseq1, imgseq2, cam_params);
end
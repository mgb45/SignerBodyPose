clc;
clear all;
close all;

addpath ./UpperBodyTracking;
% Reformat data for various videos
fid = fopen('videos.txt');
video_names = textscan(fid,'%s');
fclose(fid);
name = video_names{1};
frame_count = load('frames.txt');

video = struct('name',name);
idx = [0;find(diff(frame_count)~=1);length(frame_count)];


% Example 1 video
testVid = video(randi(length(video),1,1));
% im_dir = strcat('./tracking-groundtruth-sequences/',testVid.name,'/');

im_dir = './tracking-groundtruth-sequences/06May_2010_Thursday_heute_default-4/';
% im_dir = './tracking-groundtruth-sequences/06April_2010_Tuesday_heute_default-5/';

Files = dir(im_dir);
invalid = false(length(Files),1);
RegularExpression = '(\w+\.(jpg)|(jpeg)|(gif)|(bmp)|(png)|(ppm))$';
for i=1:numel(Files)
    invalid(i) = isempty(regexpi(Files(i).name, RegularExpression));
end
Files(invalid) = [];

N = 10;
colormap gray
im_patch = 40*ones(40,40);
r1 = 180; r2 = 180; c1 = 40; c2 = 220;
for j = 1:length(Files)
    
    subplot(1,2,1)
    cla;
    im = double(imread(strcat(im_dir,Files(j).name)));

    im = round((65.481*im(:,:,1) + 128.553*im(:,:,2) + 24.966*im(:,:,3) + 16)/255);
     

%     im = rgb2ycbcr(im);
%     im = rgb2ycbcr(im);
%     im = double(im(:,:,1));
    imagesc(im)
    hold on;
    rectangle('Position',[81,181,40,40],'LineWidth',2,'EdgeColor','m') 
    im_patch = im_patch*0.99+ 0.01*double(im(181:220,81:120));
    x = hist(im_patch(:),1:255/N:255);
    x = interp1(1:255/N:255,x,1:255);
    x = x/sum(isfinite(x));

    [tf,locs] = ismember(im(:), 1:length(x));
    im(tf) = x(locs(locs>0));
%     for k = 1:length(x)
%         im(im==k) = x(k);
%     end
%     medfilt2(im,[5 5]);
    
    im(im>1) = 1;
    
    x1 = sum(im(5:end-5,1:size(im,2)/2),2);
    x2 = sum(im(5:end-5,size(im,2)/2:size(im,2)),2);
    r1 = 0.5*r1 + 0.5*(5+find(x1==max(x1),1,'first')); % Exp weighted filt to avoid clutter noise
    r2 = 0.5*r2 + 0.5*(5+find(x2==max(x2),1,'first'));
    
    c1 = c1*0.5 + 0.5*find(abs(diff(im(int16(r1),1:size(im,2)/2)))>0,1,'first');
    if (isempty(c1))
        c1 = 10;
    end
    c2 = 0.5*c2 + 0.5*(size(im,2)/2 + find(abs(diff(im(int16(r2),size(im,2)/2:size(im,2))))>0,1,'last'));
    if (isempty(c2))
        c2 = size(im,2)-10;
    end
    plot(c1,1:size(im,1),'b')
    plot(1:size(im,2)/2,r1,'b')
    plot(size(im,2)/2:size(im,2),r2,'g')
    plot(c2,1:size(im,1),'g')
    
    subplot (1,2,2)
    cla;
    imagesc(log(double(im)))
%     hold all;
%     plot(flipud(sum(im(:,1:size(im,2)/2),2)),1:size(im,1))
%     plot(flipud(sum(im(:,size(im,2)/2:end),2)),1:size(im,1))
    
    pause(0.05);
        
end

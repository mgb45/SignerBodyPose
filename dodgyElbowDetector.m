function [r1,c1,r2,c2,im_patch] = dodgyElbowDetector(im,im_patch,r1,c1,r2,c2)
%Dodgy elbow detector: returns elbow positions in image
% uses torso patch to segment body, widest image section corresponds to
% elbow row, first/last diff to col, has hard coded image locations and dirty heuristics!

% Get luminance channel
% im = rgb2ycbcr(im);
% im = double(im(:,:,1));
im = round((65.481*im(:,:,1) + 128.553*im(:,:,2) + 24.966*im(:,:,3) + 16)/255);

% Extract torso patch, average with previous (exp weighted filt) 
im_patch = im_patch*0.99+ 0.01*double(im(181:220,81:120));

N = 10;
% Build histogram from patch
x = hist(im_patch(:),1:255/N:255);
x = interp1(1:255/N:255,x,1:255);
x = x/sum(isfinite(x));

% Back project hist (likelihood approx)
 [tf,locs] = ismember(im(:), 1:length(x));
 im(tf) = x(locs(locs>0));
% for k = 1:length(x)
%     im(im==k) = x(k);
% end
% Threshhold to get binary image (sort of)
im(im>1) = 1;

% Elbow rows are widest non-zero part in respective sides of image
x1 = sum(im(5:end-5,1:size(im,2)/2),2);
x2 = sum(im(5:end-5,size(im,2)/2:size(im,2)),2);
r1 = 0.5*r1 + 0.5*(5+find(x1==max(x1),1,'first')); % Exp weighted filt to avoid clutter noise
r2 = 0.5*r2 + 0.5*(5+find(x2==max(x2),1,'first'));

% Elbow cols are first change in threshold on given row
c1 = c1*0.5 + 0.5*find(abs(diff(im(int16(r1),1:size(im,2)/2)))>0,1,'first');
if (isempty(c1))
    c1 = 10;
end
c2 = 0.5*c2 + 0.5*(size(im,2)/2 + find(abs(diff(im(int16(r2),size(im,2)/2:size(im,2))))>0,1,'last')); % Exp weighted filt to avoid clutter noise
if (isempty(c2))
    c2 = size(im,2)-10;
end

end


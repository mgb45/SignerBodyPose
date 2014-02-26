clc;
clear all;
close all;

addpath ./UpperBodyTracking;
% Reformat data for various videos
% Hand position annotations
n = load('hand_id.txt');
x = reshape(load('hand_x.txt'),3,[]);
y = reshape(load('hand_y.txt'),3,[]);
% Video folder names
fid = fopen('videos.txt');
video_names = textscan(fid,'%s');
fclose(fid);
name = video_names{1};
frame_count = load('frames.txt');

video = struct('name',name);
idx = [0;find(diff(frame_count)~=1);length(frame_count)];

for j = 1:length(idx)-1
    video(j).frames = frame_count(idx(j+1));
    video(j).joints_x = x(:,idx(j)+1:idx(j+1));
    video(j).joints_y = y(:,idx(j)+1:idx(j+1));
    video(j).joint_labels = {'Left hand','Right hand','Head'};
end

% Load GMM parameters - trained using ros gmm_training package
% Right arm model
WeightsR = load('./UpperBodyTracking/model/Training20-02/WeightsR3D.txt');
CovsR = load('./UpperBodyTracking/model/Training20-02/CovsR3D.txt');
MeansR = load('./UpperBodyTracking/model/Training20-02/MeansR3D.txt');
% Left arm model
WeightsL = load('./UpperBodyTracking/model/Training20-02/WeightsL3D.txt');
CovsL = load('./UpperBodyTracking/model/Training20-02/CovsL3D.txt');
MeansL = load('./UpperBodyTracking/model/Training20-02/MeansL3D.txt');

% Camera intrinsics - used for 3D reconstruction
K = [1550 0.000000 130;
    0.000000 1550 105;
    0.000000 0.000000 1.000000];

testVid = video(randi(length(video),1,1));

% for vI = 1:length(video) % Iterate through videos
%     testVid = video(vI);
    
    % Get image list in test video directory
    im_dir = strcat('./tracking-groundtruth-sequences/',testVid.name,'/');
    Files = dir(im_dir);
    invalid = false(length(Files),1);
    RegularExpression = '(\w+\.(jpg)|(jpeg)|(gif)|(bmp)|(png)|(ppm))$';
    for i=1:numel(Files)
        invalid(i) = isempty(regexpi(Files(i).name, RegularExpression));
    end
    Files(invalid) = [];
    
    % Initialise MKF's
    model_right = initialiseTracker(WeightsR,MeansR,CovsR);
    model_left = initialiseTracker(WeightsL,MeansL,CovsL);
    
    
    cols=['b','g','m','r'];
    
    % vidObj = VideoWriter('test.avi');
    % open(vidObj);
    % fig = figure;
    im_patch = 40*ones(40,40);
    r1 = 180; r2 = 180; c1 = 40; c2 = 220;
    
    for j = 1:length(Files)
        
        % Progress display
%         [vI length(video) j testVid.frames+1]
        
        im = imread(strcat(im_dir,Files(j).name));
               
        [r1,c1,r2,c2,im_patch] = dodgyElbowDetector(double(im),im_patch,r1,c1,r2,c2);
        
        % Populate measurement matrices for right and left hand models using
        % manual annotations in groundtruth.mat
        measurementR = [testVid.joints_x(3,j);testVid.joints_y(3,j);testVid.joints_x(2,j);testVid.joints_y(2,j);testVid.joints_x(3,j);testVid.joints_y(3,j)+45;c1;r1];
        measurementL = [testVid.joints_x(3,j);testVid.joints_y(3,j);testVid.joints_x(1,j);testVid.joints_y(1,j);testVid.joints_x(3,j);testVid.joints_y(3,j)+45;c2;r2];
        
        % Add eps to weights so all mixtures contribute to the model
        model_right.w = model_right.w+5e-2;
        model_left.w = model_left.w+5e-2;
        model_left.w = model_left.w/sum(model_left.w);
        model_right.w = model_right.w/sum(model_right.w);
        
        % Update pose estimate using measurement
        model_right = updatePoseEstimate(model_right,measurementR);
        model_left = updatePoseEstimate(model_left,measurementL);
        
        % Get MAP point estimate
        [rEstimate,rCamPosEstimate] = getMAPPointEstimate(model_right);
        [lEstimate,lCamPosEstimate] = getMAPPointEstimate(model_left);
        
        % Get 3D pose
        % 3D pose vector order is: Hand (x y z) Elbow (x y z) Shoulder (x y z) Neck (x y z) Head (x y z) 
        [pos3DR,pos3DL] = get3DPoseEstimate(rEstimate,rCamPosEstimate,lEstimate,lCamPosEstimate,K);
        
        % Rotate 3D body points using camera rotation (we know camera is static, hence any rotation should be due to the body)
        pos3DR = (rotEuler(lCamPosEstimate(2),lCamPosEstimate(3),lCamPosEstimate(1))*pos3DR + rotEuler(rCamPosEstimate(2),rCamPosEstimate(3),rCamPosEstimate(1))*pos3DR)/2;
        pos3DL = (rotEuler(lCamPosEstimate(2),lCamPosEstimate(3),lCamPosEstimate(1))*pos3DL + rotEuler(rCamPosEstimate(2),rCamPosEstimate(3),rCamPosEstimate(1))*pos3DL)/2;
        
        clf;
        % Plot 2D results
        subplot(1,2,1)
        
        hold off
        imshow(im);
        hold on;
        for jidx = 4:-1:1
            line([rEstimate(3*jidx-2) rEstimate(3*(jidx+1)-2)],[rEstimate(3*jidx-1) rEstimate(3*(jidx+1)-1)],'Color',cols(jidx),'LineWidth',4)
            line([lEstimate(3*jidx-2) lEstimate(3*(jidx+1)-2)],[lEstimate(3*jidx-1) lEstimate(3*(jidx+1)-1)],'Color',cols(jidx),'LineWidth',4)
        end
        
        % Plot 3D results
        angs = [0,0;0,90;90,0];
        for im_idx = 1:3
            subplot(3,2,2*im_idx)
            
            hold on;
            for jidx = 1:4
                line([pos3DR(1,jidx) pos3DR(1,jidx+1)],[pos3DR(3,jidx) pos3DR(3,jidx+1)],-[pos3DR(2,jidx) pos3DR(2,jidx+1)],'Color',cols(jidx),'LineWidth',4)
                line([pos3DL(1,jidx) pos3DL(1,jidx+1)],[pos3DL(3,jidx) pos3DL(3,jidx+1)],-[pos3DL(2,jidx) pos3DL(2,jidx+1)],'Color',cols(jidx),'LineWidth',4)
            end
            axis([-0.6 0.6,-0.6 0.2,-0.6 0.2])
            axis equal
            view(angs(im_idx,1),angs(im_idx,2))
            grid on;
        end
        
        %
        %     currFrame = getframe(fig);
        %     writeVideo(vidObj,currFrame);
        
        pause(0.01)
    end
    % close(vidObj);
    
% end

function [pos3DR,pos3DL] = get3DPoseEstimate(rEstimate,rCamPosEstimate,lEstimate,lCamPosEstimate,K)
% Solve for 3D using point estimates and scales

camAngleL = lCamPosEstimate(1:3);
camPosL = lCamPosEstimate(4:6);
    
camAngleR = rCamPosEstimate(1:3);
camPosR = rCamPosEstimate(4:6);

ProjL = K*[rotEuler(camAngleL(2),camAngleL(3),camAngleL(1)) camPosL'];
ProjR = K*[rotEuler(camAngleR(2),camAngleR(3),camAngleR(1)) camPosR'];
im_pointsL = reshape(lEstimate(1:15),3,[])';
im_pointsL = im_pointsL.*[repmat(im_pointsL(:,3),1,2) ones(5,1)];
pos3DL = (ProjL(:,1:3))\bsxfun(@minus,im_pointsL,ProjL(:,4)')';

im_pointsR = reshape(rEstimate(1:15),3,[])';
im_pointsR = im_pointsR.*[repmat(im_pointsR(:,3),1,2) ones(5,1)];
pos3DR = (ProjR(:,1:3))\bsxfun(@minus,im_pointsR,ProjR(:,4)')';

% Average head and neck positions - independent left/right tracking
pos3DR(:,4) = (pos3DR(:,4)+pos3DL(:,4))/2;
pos3DL(:,4) = pos3DR(:,4);
pos3DR(:,5) = (pos3DR(:,5)+pos3DL(:,5))/2;
pos3DL(:,5) = pos3DR(:,5);
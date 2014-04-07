function [poseEstimate,camPosEstimate] = getMAPPointEstimate_MKF(model)

N = length(model.w);
d = size(model.Xk_k{1},1);
particles = reshape(cell2mat(model.Xk_k),d,N)';

% Get point estimates
poseEstimate = sum(bsxfun(@times,model.w,[particles(:,1:9) particles(:,13:15) particles(:,10:12)]));
camPosEstimate = [sum(bsxfun(@times,model.w,particles(:,16:18))) sum(bsxfun(@times,model.w,particles(:,19:21)))];

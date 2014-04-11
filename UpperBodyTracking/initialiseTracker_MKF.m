function [model,state] = initialiseTracker_MKF(Weights,Means,Covs,nParticles)

N = length(Weights);
d = size(Covs,2);

Xk_k = cell(N,1);
Pk_k = cell(N,1);

% State selection for measurement
H = [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

R = 1*eye(8); % Measurement noise
R(5,5) = 25;
R(6,6) = 25;
R(7,7) = 25;
R(8,8) = 25;
Sigma_a = diag([560 560 1 56 56 1 25 25 1 25 25 1 25 25 1 0.5 0.5 0.5 0.01 0.01 0.01]); % Transition noise estimate

% Initial normalised weights
% w = Weights'/sum(Weights);

% Initial parameter means and covs
for i = 1:N
        
    % Kalman filter matrices
    Sigma_i = Covs(d*i-(d-1):d*i,:);
    Q(d*i-(d-1):d*i,:) = inv(inv(Sigma_a) + inv(Sigma_i));
    F(d*i-(d-1):d*i,:) = Q(d*i-(d-1):d*i,:)/Sigma_a;
    
    
    B(d*i-(d-1):d*i,:) = Q(d*i-(d-1):d*i,:)/Sigma_i*Means(i,:)';
end

for i = 1:nParticles
    Xk_k{i} = Means(randi(N,1,1),:)';
    Pk_k{i} = 5000*eye(d);
end
    

model = struct('B',{B},'F',{F},'H',{H},'Q',{Q},'init_weights',Weights,'R',{R});
state = struct('Xk_k',{Xk_k},'Pk_k',{Pk_k},'w',{ones(nParticles,1)/nParticles});
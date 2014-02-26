function model = initialiseTracker(Weights,Means,Covs)

N = length(Weights);
d = size(Covs,2);

Xk_k = cell(N,1);

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
R(5,5) = 124;
R(6,6) = 144;
R(7,7) = 124;
R(8,8) = 144;
Sigma_a = diag([560 560 1 56 56 1 25 25 1 25 25 1 25 25 1 0.5 0.5 0.5 0.01 0.01 0.01]); % Transition noise estimate

% Initial normalised weights
w = Weights'/sum(Weights);

% Initial parameter means and covs
for i = 1:N
    Xk_k{i} = Means(i,:)';
    Pk_k(d*i-(d-1):d*i,:) = 5000*eye(d);
    
    
    % Kalman filter matrices
    Sigma_i = Covs(d*i-(d-1):d*i,:);
    Q(d*i-(d-1):d*i,:) = inv(inv(Sigma_a) + inv(Sigma_i));
    F(d*i-(d-1):d*i,:) = Q(d*i-(d-1):d*i,:)/Sigma_a;
    
    
    B(d*i-(d-1):d*i,:) = Q(d*i-(d-1):d*i,:)/Sigma_i*Means(i,:)';
end

model = struct('Xk_k',{Xk_k},'Pk_k',{Pk_k},'R',{R},'w',{w},'B',{B},'F',{F},'H',{H},'Q',{Q},'init_weights',Weights);
function model = updatePoseEstimate(model,measurement)

d = size(model.H,2);

for i = 1:length(model.w) % Integrate conditionals with KF
    Pk_k_1 = model.F(d*i-(d-1):d*i,:)*model.Pk_k(d*i-(d-1):d*i,:)*model.F(d*i-(d-1):d*i,:)' + model.Q(d*i-(d-1):d*i,:);
    Xk_k_1 = model.F(d*i-(d-1):d*i,:)*model.Xk_k{i} + model.B(d*i-(d-1):d*i,:);

    ye = measurement - model.H*Xk_k_1;
    S = model.H*Pk_k_1*model.H' + model.R;

    model.w(i) = model.init_weights(i)*mymvnpdf(ye, 0,S)*model.w(i);

    K = Pk_k_1*model.H'/S;

    model.Xk_k{i} = Xk_k_1 + K*ye;

    model.Pk_k(d*i-(d-1):d*i,:) = (eye(d) - K*model.H)*Pk_k_1;
end

% Reset if GMM collapses -- happens when mislabeled images cause zero
% weights
if (sum(model.w) == 0)
    model.w = model.init_weights';
end
model.w = model.w./sum(model.w);


    
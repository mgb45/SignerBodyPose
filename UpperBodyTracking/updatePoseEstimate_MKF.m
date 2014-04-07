function [state_out] = updatePoseEstimate_MKF(model,measurement,indicators,state)

d = size(model.H,2);
state_out = state;

for n = 1:length(indicators) % Integrate conditionals with KF
    i = indicators(n);
    Pk_k_1 = model.F(d*i-(d-1):d*i,:)*state.Pk_k{n}*model.F(d*i-(d-1):d*i,:)' + model.Q(d*i-(d-1):d*i,:);
    Xk_k_1 = model.F(d*i-(d-1):d*i,:)*state.Xk_k{n} + model.B(d*i-(d-1):d*i,:);

    ye = measurement - model.H*Xk_k_1;
    S = model.H*Pk_k_1*model.H' + model.R;

    state_out.w(n) = mymvnpdf(ye, 0,S)*state.w(n);

    K = Pk_k_1*model.H'/S;

    state_out.Xk_k{n} = Xk_k_1 + K*ye;

    state_out.Pk_k{n} = (eye(d) - K*model.H)*Pk_k_1;
end

% Reset if GMM collapses
if (sum(state_out.w) == 0)
     state_out.w = ones(length(indicators),1)/length(indicators);%model.init_weights';
end
state_out.w = state_out.w./sum(state_out.w);
% model.w = model.w./sum(model.w);


    
function y = mymvnpdf(X, Mu, Sigma)

% Matlab's mvnpdf with all the unnecessary removed

d = size(X,2);

X0 = bsxfun(@minus,X,Mu)';

Sigma = mean(cat(3,Sigma,Sigma'),3);

R = cholcov(Sigma,0);
xRinv = X0 / R;
logSqrtDetSigma = sum(log(diag(R)));
quadform = sum(xRinv.^2, 2);

y = exp(-0.5*quadform - logSqrtDetSigma - d*log(2*pi)/2);

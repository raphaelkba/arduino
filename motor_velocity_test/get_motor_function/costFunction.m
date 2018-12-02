function [J, grad] = costFunctionReg(theta, X, y, lambda)
%   costFunctionReg computes the cost and gradient of a nonlinear
%   regression

% Initialize vairables
m = length(y); % number of training examples
grad = zeros(size(theta));

% Calculate cost (regularized cost does not count theta(1))
J = (1/2*m)*sum((X*theta - y).^2);
J = J + (lambda/(2*m))*sum(theta(2:end).^2);

% Calculate cost (regularized gradient does not count theta(1))
grad(1) = sum((X*theta - y).*X(:,1));
grad(2:end) = sum((X*theta - y).*X(:,2:end))' + (lambda/m)*theta(2:end);
end

%% Get motors profile
% find a profile for the motors solving a regression problem
%% Initialization
clear ; close all; clc
%% Data 
% Data from Arduino

% motor velocity
x_right = [0.105000 0.102188 0.104062 0.104062 0.104062 0.100312 0.099375...
    0.099375 0.096562 0.095625 0.095625 0.093750 0.093750 0.090000 ...
    0.090000 0.088125 0.085312 0.082500 0.081562 0.079688 0.079688 ...
    0.075937 0.075937 0.072187 0.072187 0.066812 0.062812 0.059063 ...
    0.057187 0.053437 0.048750 0.045000 0.041250 0.035625 0.030937]';
x_left = [0.092813 0.088125 0.087187 0.087187 0.087187 0.080625 0.080625...
    0.079688 0.079688 0.077813 0.075813 0.074062 0.074062 0.070313 ...
    0.069375 0.066562 0.066562 0.064687 0.062812 0.059063 0.058125 ...
    0.055312 0.049687 0.043500 0.043500 0.043500 0.042188 0.040313 ...
    0.038437 0.036000 0.035000 0.034000 0.033000 0.032000 0.030000]';

% motor input
y = linspace(255,85,size(x_right,1))';

data_points = [x_right x_left];

for i=1:size(data_points,2)
    
X = data_points(:,i);
figure()
plot(y, X,'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
hold on

% Choose polynomial degree
degree = 3;
% Set regularization parameter lambda to 1
lambda = 0.0000005;
%% Polynomial fitting
% fits data in a polynom and plots
coeff_fit = fit_polynom(X, y, degree);
fprintf('Theta polynomial fit: %f \n', coeff_fit);
%% Optimization Problem

% gets the polynom features with ones at the front
X_ = ones(size(X,1),1);
for i = 1:degree
    X_(:, end+1) = X.^(i);
end

% Initialize fitting parameters
theta = 0*rand(size(X_, 2), 1);

% Initialize some useful values
m = length(y); % number of training examples
grad = zeros(size(theta));

% Calculate cost (regularized cost does not count theta(1))
J = (1/2)*sum((X_*theta - y).^2);
J = J + (lambda/(2*m))*sum(theta(2:end).^2);
fprintf('Cost at initial theta (zeros): %f\n', J);
% Calculate cost (regularized gradient does not count theta(1))
grad(1) = sum((X_*theta - y).*X_(:,1));
grad(2:end) = sum((X_*theta - y).*X_(:,2:end))' + (lambda/m)*theta(2:end);
fprintf('Gradient at initial theta (zeros):\n');
fprintf(' %f \n', grad(1:degree));

% Set Options for the local optimizer
options = optimoptions('fminunc','Algorithm','quasi-newton',...
    'SpecifyObjectiveGradient',false,'OptimalityTolerance', 1e-10,...
    'MaxIter', 100000,'StepTolerance',1e-10);
% Local Optimizer
[theta_local, J_local, exit_flag] = ...
	fminunc(@(t)(costFunction(t, X_, y, lambda)), theta, options);
fprintf('Theta local optimizer: %f \n', theta_local);
fprintf('Cost local optimizer: %f \n', J_local);

% Global Optimizer
gs = GlobalSearch;
func = @(theta)(1/2*m)*sum((X_*theta - y).^2) + (lambda/(2))*sum(theta(2:end).^2);
problem = createOptimProblem('fmincon','x0',theta,...
    'objective',func);
theta = run(gs,problem);

fprintf('Theta global optimizer: %f \n', theta);
fprintf('Cost global optimizer: %f \n', J);

plot_x = linspace(min(x_right(:,1)),max(x_right(:,1)), 20);

% gets the polynom features
plot_y = get_polynom(theta, plot_x, degree);
plot(plot_y, plot_x,'g-');

% gets the polynom features
plot_y_local = get_polynom(theta_local, plot_x, degree);
plot(plot_y_local, plot_x,'k-');
hold on
  
% Legend, specific for the exercise
legend('Data points', 'Polynomal Fit', 'Global opt', 'Local opt')
axis('tight')
end

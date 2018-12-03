function [theta] = fit_polynom(x, y, degree)
% fit_polynom function returns the weights of a polynom of a given degree
% for the data x and y. Furthermore, it plots the data

% gets the coefficients for the polynom
coeff = polyfit(x,y,degree);

% prepare the data to plot
x_ = linspace(min(x(:,1)),max(x(:,1)), 50);
theta = flip(coeff');

% gets the polynom features
f1 = get_polynom(theta, x_, degree);

% plot data
plot(f1,x_,'r-');
end
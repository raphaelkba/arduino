function [f] = get_polynom(theta, x, degree)
% get_polynom gives the polynom correspoding to the given polynom degree
% for data x and weights theta

    f = 0;
    for i=1:degree+1
       f = f + theta(i).*x.^(i-1);
    end
    
end
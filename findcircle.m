function [rad, xc, yc, MSE] = findcircle(x,y)
    % set up an overconstrained system of linear equations
    % A*w = b
    A = [x y ones(size(x))];
    B = -x.^2 - y.^2;
    w = A\B;
    mse = x.^2 + y.^2 + w(1).*x + w(2).*y + w(3);
    MSE = sum((mse.^2))/length(mse);
    % convert from the least squares solution to the more familiar parameters
    % of a circle.
    xc = -w(1)/2;
    yc = -w(2)/2;
    rad = sqrt(xc.^2 + yc.^2 - w(3));
end
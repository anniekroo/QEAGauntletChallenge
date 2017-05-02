function V = potentialFeild(px, py, m, b, endpoints, circle, radius)
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
C = 40;
V0 = 0;
for line = 1:size(m, 1)
    start1 = endpoints(line, 1);
    stop1 = endpoints(line, 3);
    if start1 > stop1
        start = stop1;
        stop = start1;
    else
        start = start1;
        stop = stop1;
    end
    dx = @(x) C.*(sqrt(m(line).^2 + 1)./((sqrt((px-x).^2 + (py - m(line).*x - b(line)).^2))));
    Dx = integral(dx,start,stop);
    V0 = V0 + Dx;
end
dp = -200./(sqrt((px - circle(1)).^2 + (py - circle(2)).^2));
V = V0 + dp;
end
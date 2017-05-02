function Vmacro2(cleanx, cleany, m, b, endpoints, circle, C1, C2)
    %edit your ranges to display here.  important to not include the actual
    %location of your object in this grid of points or it will give you
    %infinities

    [px,py]=meshgrid(-2:.15:2,-2:.15:2);
    [xlim,ylim] = size(px);
    V = zeros(xlim, ylim);
    V0 = 0;

    for i=1:xlim
        for j=1:ylim
            %this is the equation and integral with ranges for a specific object:  you
            %should be able to figure out what this is and edit appropriately to get
            %what you want
            V0(i,j) = 0;
            for line = 1:size(m, 1)
                start1 = endpoints(line, 1);
                start2 = endpoints(line, 2);
                stop1 = endpoints(line, 3);
                stop2 = endpoints(line, 4);
                dist = sqrt((start1-stop1).^2 + (start2-start1).^2);
                if dist < .2
                    C1 = C1*1;
                end
                if start1 > stop1
                    start = stop1;
                    stop = start1;
                else
                    start = start1;
                    stop = stop1;
                end
                dx = @(x)  C1.*(sqrt(m(line).^2 + 1)./(sqrt((px(i,j)-x).^2 + (py(i,j) - m(line).*x - b(line)).^2)) );
                Dx(i,j) = integral(dx,start,stop);
                V0(i,j) = V0(i,j) + Dx(i,j);
            end
            value = V0(i,j);
            if value > C1*100
                value = 0;
            end
            dp(i,j) = C2./(sqrt((px(i,j) - circle(1)).^2 + (py(i,j) - circle(2)).^2));
            V(i,j) = value + dp(i,j);
        end
    end
    figure

    contour(px,py,V)
    [Ex,Ey] = gradient(V);
    hold on
    %quiver(px,py,-Ex.,-Ey); axis('equal')
    plot(cleanx,cleany, '*')
    quiver(px,py,-Ex./(sqrt(Ex.^2 + Ey.^2)),-Ey./(sqrt(Ex.^2 + Ey.^2))); axis('equal')
    
end
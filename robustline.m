function [m,b,outliers,inliers] = robustline(x,y)
    n = length(x);
    threshhold = .2;
    xrange = abs(min(x)-max(x));
    yrange = abs(min(y)-max(y));
    threshhold = threshhold * mean([xrange yrange])/20;
    mb = [1; 1];
    maxcount = 0;
    for i = 1:n
        %getting random indexes of points
        ran = randi([1 length(x)],1,2);
        i1 = ran(1);
        i2 = ran(2);

        %determining random points
        x1 = x(i1);
        x2 = x(i2);

        y1 = y(i1);
        y2 = y(i2);

        Q1 = [x1; y1; 0];
        Q2 = [x2; y2; 0];

        %equation
        rise = y2 - y1;
        run = x2 -x1;
        slope = rise/run;
        b = y1 - slope*x1;

        count = 0; %how many points consecutively on line
        out = []; %keeps track of indexes of outliers
        ins = []; %keeps track of indexes of inliers
        
        
        for m = 2:length(x)
            P0 = [x(m-1); y(m-1); 0]; %point before current point
            P = [x(m); y(m); 0]; %current point
            
            %distance between the points
            distancepoints = sqrt((x(m)-x(m-1)).^2+(y(m)-y(m-1)).^2);
            
            %distance point to line
            d = point_to_line(P0, Q1, Q2);
            
            %adds point to line if it is not too far away
            if d <= threshhold && distancepoints < .1
                ins(end+1) = m;
                count = count +1;
            elseif d > threshhold && count > 0
                break
            elseif distancepoints > .1 && count > 0
                break
            end
        end
        
        %if line has most points repleace line with most points
        if count > maxcount
            maxcount = count;
            mb = [slope; b];
            finalin = ins;
        end
        if i == n-1 && maxcount <= 1
            finalin = 1:length(x);
        end
    end
    
    m = mb(1); %final slope
    b = mb(2); %final intercept
    
    inliers = finalin; %indexes of the points on line
    outs = 1:length(x); %creates a matrix of indexes
   
    outs(finalin) = []; %takes out indexes of inliers
    outliers = outs; %ship it!
    

    function d = point_to_line(pt, v1, v2)
          a = v1 - v2;
          c = pt - v2;
          d = norm(cross(a,c)) / norm(a);
    end
end
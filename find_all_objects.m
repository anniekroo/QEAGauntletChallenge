
function [slope, intercept, endpoints, circle, radius, circlevalue] = find_all_objects(x, y)
    theradius = 0.1192547989;
    xoutlist{1} =x; %list of list of x coordinates that are outliers
    youtlist{1} =y; %list of list of y coordinates that are outliers
    xinlist = {}; %keeps the x coordinates inliers for each line segment
    yinlist = {}; %keeps the y coordinates inliers for each line segment
    inliers = 0; %keeps track of how many inliers there are
    outliers = 1:length(x); %stores all points not in lines yet
    m = []; %stores the slope of all the lines
    b = []; %stores the intercept of all the lines
    bucket = [0, 0, 0];
    index = 1; %keeps track of how many lines there are
    yes = 1; 

    while yes == 1
        %finds best line 
        [slp, intr,outliers,ins] = robustline(xoutlist{index},youtlist{index}); 
        %checks if points used in line are a circle
        [rad, xc, yc, MSE] = findcircle(xoutlist{index}(ins)',youtlist{index}(ins)');
        inliers = length(x) - length(outliers); %changes length of inliers by how many are in line/circle

        %creates new list of outliers for next round
        xoutlist{index+1} = xoutlist{index}(outliers); 
        youtlist{index+1} = youtlist{index}(outliers); 

        %checks if radius is reasonable and if so saves circle
        difradius = abs(rad - theradius);
        if difradius <= .01 && abs(bucket(1)-theradius) > difradius && MSE < 1e-4
            bucket= [rad, xc, yc];
        else %otherwise adds the points to the line lists
            xinlist{end+1} = xoutlist{index}(ins);
            yinlist{end+1} = youtlist{index}(ins);
            m(end+1) = slp;
            b(end+1) = intr;
        end

        %if the length of the outliers lists are the same no new lines are
        %being created
        if length(xoutlist{index+1}) == length(xoutlist{index})
            break
        end
        %if all the points are accounted for stop the program
        if inliers == length(x)
            break
        end

        index = index +1;

    end
    figure
    hold on
    plot(x, y,'.')

    %plots the lines
    for i = 1:length(m)-1
        yline = m(i).* xinlist{i} + b(i);
        plot(xinlist{i}, yline, 'r')
    end

    %plots the bucket
    xc = bucket(2);
    yc = bucket(3);
    rad = bucket(1);
    viscircles([xc yc], rad)
    
    slope = m';
    intercept = b';
    endpts = zeros(length(xinlist), 4);
  
    for i = 1:length(xinlist)
        endpts(i,1) = xinlist{i}(1);
        endpts(i,2) = yinlist{i}(1);%xinlist{i}(1)*m(i)+b(i);
        endpts(i,3) = xinlist{i}(end);
        endpts(i,4) = yinlist{i}(end);%xinlist{i}(end)*m(i)+b(i);
    end
    if bucket(1) == 0
        endpoints = endpts;
        circle = [bucket(2), bucket(3)];
        radius = bucket(1);
        circlevalue = 0;
    else
        endpoints = endpts;
        circle = [bucket(2), bucket(3)];
        radius = bucket(1);
        circlevalue = 1;
    end
end
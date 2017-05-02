%{
function [xclean, yclean] = cleandata()
    load 'playpensample.mat';
    clean_r = [];
    clean_t = [];
    
    %takes out the zeros
    for i = 1:length(r)
        if r(i) ~= 0
            clean_r(end+1) = r(i);
            clean_t(end+1) = theta(i);
        end
    end

    %converts to x and y from polar
    x = clean_r.* cosd(clean_t);
    y = clean_r.* sind(clean_t);
    i = 1;

    %takes out points outside of the pen
    while i < length(x)
        distance = sqrt(((x(i).^2) + (y(i).^2)));
        if distance > 2.5 || distance < .1
            x(i) = [];
            y(i) =[];
        else
            i = i+1;
        end

    end
    xclean = x;
    yclean = y;
end
%}


function [xclean, yclean] = cleandata()
    sub = rossubscriber('/stable_scan');

    % Collect data at the room origin
    scan_message = receive(sub);
    r = scan_message.Ranges(1:end-1);
    theta = [0:359]';
    
        clean_r = [];
    clean_t = [];
    
    %takes out the zeros
    for i = 1:length(r)
        if r(i) > .05
            clean_r(end+1) = r(i);
            clean_t(end+1) = theta(i);
        end
    end

    %converts to x and y from polar
    x = clean_r.* cosd(clean_t);
    y = clean_r.* sind(clean_t);
    i = 1;

    %takes out points outside of the pen
    while i < length(x)
        distance = sqrt(((x(i).^2) + (y(i).^2)));
        if distance > 2.5 
            x(i) = [];
            y(i) =[];
        else
            i = i+1;
        end

    end
    display('Cleaned!')
    xclean = x;
    yclean = y;
%     r_clean = r(r~=0);
%     theta_clean = theta(r~=0);
% 
%     xclean = (r_clean).*cosd(theta_clean);
%     yclean = (r_clean).*sind(theta_clean);

end

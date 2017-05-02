clear
tracker = NeatoTracker;
tracker.resetNeatoTransform

speedMult = 0.04;
turnMult = 0.2;
d = 0.24765;                            %distance between wheels in meters

%equations for determining wheel velocities
syms w v;
vL = v - (w*d/2);
vR = v + (w*d/2);

%accessing sensors
sub_enc = rossubscriber('/encoders');   %set up subscriber for encoders to be written to later
sub_bump = rossubscriber('/bump');      %set up subscriber for bump to be written to later
pub = rospublisher('/raw_vel');         %set up publisher for velocity to be written to later
msg = rosmessage(pub);                  %establishes message object containing publisher

[cleanx, cleany] = cleandata();
[m, b, endpoints, circle, radius, circlevalue] = find_all_objects(cleanx, cleany);
pnt = [111; 55; 1];
Vmacro2(cleanx, cleany, m, b, endpoints, pnt, 40, -10)

while  1 == 1 %searching loop
    
    tracker.resetNeatoTransform
    
    %cleaning things
    [cleanx, cleany] = cleandata();
    
    [m, b, endpoints, circle, radius, circlevalue] = find_all_objects(cleanx, cleany);
    
   
    if circlevalue == 0
        pnt = [111; 55; 1];
        tic
        while true
            runningtime = toc;
            if runningtime >= 1
                break;
            end
            %recording position for later mapping purposes
            [R, T] = tracker.getNeatoTransform();
            pos = T(:, 3);
            x = .2032;
            y = .05;
            robotPnt = [1; 0; 1];
            angle = -acos(R(1,1));

            V = potentialFeild1(x, y, m, b, endpoints, robotPnt);
            Vx = potentialFeild1((x+.01), (y), m, b, endpoints, robotPnt);
            Vy = potentialFeild1((x), (y+.01), m, b, endpoints, robotPnt);
            grad = [(V-Vx)/.01 , ((V-Vy)/0.01)];
            idealTheta = atan2(grad(2), grad(1));
            
            %Vmacro2(cleanx, cleany, m, b, endpoints, pnt, 40, -10)
            hold on
             quiver(0,0,grad(1) / norm(grad), grad(2) / norm(grad), 'k')
             hold off
            %moving to path
            dtheta = idealTheta - angle;
            w = turnMult.*dtheta./sqrt(abs(dtheta));
            v = speedMult./sqrt(abs(dtheta));
            display([double(subs(vL)), double(subs(vR))]);
            msg.Data = [double(subs(vL)), double(subs(vR))];        %writes velocities to message data object
            send(pub, msg);                             %sends velocities to wheels

            %stops if bump sensor is pressed
            bumpy = receive(sub_bump);
            if any(bumpy.Data)
                break;
            end
        end
        msg.Data = [0,0];                       %writes zero velocities to data message object neto
        send(pub, msg);                         %sends zero velocities to robot
        bumpy = receive(sub_bump);
        if any(bumpy.Data)
            break;
        end
        %pause;
    elseif circlevalue == 1
        tracker.resetNeatoTransform
        msg.Data = [double(subs(vL)), double(subs(vR))];        %writes velocities to message data object
        send(pub, msg);                             %sends velocities to wheels
        pause(1);
        msg.Data = [0,0];        %writes velocities to message data object
        send(pub, msg);
        oldxc = circle(1);
        oldyc = circle(2);
        [cleanx,cleany] = cleandata();
        [m, b, endpoints, circle, radius, circlevalue] = find_all_objects(cleanx, cleany);
        [R, T] = tracker.getNeatoTransform();
        xc = circle(1) + T(1, 3);
        yc = circle(2) + T(2, 3);
        difference = sqrt((oldxc-xc).^2 + (oldyc - yc).^2);
        if difference <= .15
            break
        end
    end
end

%tracker.resetNeatoTransform
%[cleanx, cleany] = cleandata();
%[m, b, endpoints, circle, radius, circlevalue] = find_all_objects(cleanx, cleany);
while true
    %recording position for later mapping purposes
    [R, T] = tracker.getNeatoTransform();
    pos = T(:, 3);
    x = pos(1);
    y = pos(2);
    angle = -acos(R(1,1));
    
    V = potentialFeild(x(end), y(end), m, b, endpoints, circle, radius);
    Vx = potentialFeild((x+.001), (y), m, b, endpoints, circle, radius);
    Vy = potentialFeild((x), (y+.001), m, b, endpoints, circle, radius);
    grad = [(V-Vx)/.001 , ((V-Vy)/0.001)];
    idealTheta = atan(grad(2)/grad(1));
    
    %moving to path
    dtheta = idealTheta - angle;
    w = turnMult.*dtheta./sqrt(abs(dtheta));
	v = speedMult./sqrt(abs(dtheta));
    display([double(subs(vL)), double(subs(vR))]);
    msg.Data = [double(subs(vL)), double(subs(vR))];        %writes velocities to message data object
    send(pub, msg);                             %sends velocities to wheels
    
    %stops if bump sensor is pressed
    bumpy = receive(sub_bump);
    if any(bumpy.Data)
        break;
    end
end
msg.Data = [0,0];                       %writes zero velocities to data message object neto
send(pub, msg); 



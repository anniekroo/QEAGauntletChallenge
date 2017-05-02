classdef NeatoTracker
    % This class provides a wrapper around ROS's coordinate transform
    % infrastructure.  The class provides methods for getting the
    % transformation matrices (R and T) from either the Neato frame to the
    % Room frame or from the Scan (LIDAR) frame to the Room frame.  These
    % transforms can be either be queried at the latest time available, or
    % at a specific time (e.g., based on the timestamp of a particular
    % laser scan).
    %
    % In addition to providing these transformation helpers, the class also
    % provides a method for resetting the position of the Neato in the room
    % frame to x = 0, y = 0, theta = 0.  This is useful for the Gauntlet
    % challenge when the Neato is starting off from the origin marked in
    % tape on the floor.
    properties
        tf          % a handle to ROS's tf infrastructure (rostf)
    end
    methods
        function obj = NeatoTracker()
            % During intialization of the tracker a connection is made to
            % ROS's tf infrastructure.
            obj.tf = rostf();
            % pause to allow for a few transforms to be received.
            pause(2);
        end

        function [R, T] = getScanTransform(obj, stampOrScan)
            % Get the transformation matrices that map a point in the laser
            % scanner's coordinate system to the room coordinate system.
            % 
            % stampOrScan: this is an optional argument that can be either
            % a laser scan message or a ROS timestamp.  In either case, the
            % transform returned will be valid for the given timestamp (in
            % the case passing in a scan message, the timestamp is
            % extracted from the scan's header).
            %
            % As an example, suppose we have a point x, y in Cartesian
            % coordinates in the laser scan frame.  We can transform this
            % point into the room frame using the lines of code below.
            %
            % [R, T] = getScanTransform(tracker);
            % roomPoint = T*R*[x; y; 1]
            %
            % Please note that it is important that the matrices be
            % applied in the order above to receive the correct results.
            % To map the other direction use this code.
            %
            % scanPoint = R'*inv(T)*roomPoint
            if nargin == 2
                if strcmp(class(stampOrScan), 'robotics.ros.msg.sensor_msgs.LaserScan')
                    [R, T] = getTransformHelper(obj, 'base_laser_link', true, stampOrScan.Header.Stamp);
                else
                    [R, T] = getTransformHelper(obj, 'base_laser_link', true, stampOrScan);
                end
            else
                [R, T] = getTransformHelper(obj, 'base_laser_link', true);
            end
        end

        function [R, T] = getNeatoTransform(obj, timeStamp)
            % Get the transformation matrices that map a point in the
            % Neato's coordinate system to the room coordinate system.
            % 
            % timeStamp: this is an optional argument that specifies the
            % timestamp of the coordinate transform you'd like.
            %
            % As an example, suppose we have a point x, y in Cartesian
            % coordinates in the Neato's frame.  We can transform this
            % point into the room frame using the lines of code below.
            %
            % [R, T] = getNeatoTransform(tracker);
            % roomPoint = T*R*[x; y; 1]
            %
            % Please note that it is important that the matrices be
            % applied in the order above to receive the correct results.
            % To map the other direction use this code.
            %
            % neatoPoint = R'*inv(T)*roomPoint
            if nargin == 2
                [R, T] = getTransformHelper(obj, 'base_link', false, timeStamp);
            else
                [R, T] = getTransformHelper(obj, 'base_link', false);
            end
        end

        function resetNeatoTransform(obj)
            % Reset the neato's position to x = 0, y = 0, theta = 0.
            % Calling this will cause the transformation matrices R and
            % T relating the Neato's position to the room to be identity
            % matrices.
            %
            % tracker = NeatoTracker();
            % % drive the Neato around for a while
            % resetNeatoTransform(tracker);
            % [R, T] = getNeatoTransform(tracker)
            %
            % R =
            %
            %   1     0     0
            %   0     1     0
            %   0     0     1
            %
            %
            % T =
            %
            %   1     0     0
            %   0     1     0
            %   0     0     1
            odomResetClient = rossvcclient('/reset_odom');
            resetReq = rosmessage(odomResetClient);
            [~] = call(odomResetClient, resetReq, 'Timeout',3);
        end
    end
    methods (Access=private)
        function [R, T] = getTransformHelper(obj, baseFrame, flip, timeStamp)
            % This is a private function that fetches the coordinate
            % transform from the specified base frame to the odom frame.
            %
            % flip: if true the coordinate system should be rotated by pi
            % (this applies, for instance to the Neato's base_laser_link
            % frame.
            %
            % timeStamp: this is the timestamp at which you'd like the
            % coordinate transform.
            try
                if nargin == 4
                    odom_pose = getTransform(obj.tf, 'odom', baseFrame, timeStamp);
                else
                    odom_pose = getTransform(obj.tf, 'odom', baseFrame);
                end
            catch ME
                odom_pose = [];
            end
            if isempty(odom_pose)
                disp('unable to get transform.... Ignore the returned R and T');
                R = [];
                T = [];
                return;
            end
            trans = odom_pose.Transform.Translation;
            quat = odom_pose.Transform.Rotation;
            rot = quat2eul([quat.W quat.X quat.Y quat.Z]);
            yaw = rot(1);
            if flip
                yaw = yaw + pi;
            end
            R = [cos(yaw) sin(-yaw) 0;...
                 sin(yaw) cos(yaw) 0;...
                 0 0 1];
            T = [1 0 trans.X;
                 0 1 trans.Y;
                 0 0 1];
        end
    end
end
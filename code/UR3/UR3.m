classdef UR3 < RobotInterface
    %UR3 Class for the UR3 robot
    
    properties
        
    end
    
    methods
        function self = UR3(base, name, q0)
            %UR3 Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                base (4,4) {mustBeNumeric} = se3(se2(0, 0, 0));
                name {mustBeText} = 'UR3';
                q0 = [0, -0.7854, 1.5708, -0.7854, 1.5708, 0];
            end

            limits = [ -2*pi 2*pi];

            L1 = Link('d', 0.1519,'a',0,'alpha',pi/2,'qlim', limits);
            L2 = Link('d', 0,'a',-0.24355,'alpha',0,'qlim', limits);
            L3 = Link('d', 0,'a',-0.2132,'alpha',0,'qlim', [deg2rad(-170),deg2rad(170)]); %helps prevent random twisting and self collision
            L4 = Link('d', 0.11235,'a',0 ,'alpha',pi/2,'qlim', limits);
            L5 = Link('d', 0.08535,'a',0,'alpha',-pi/2,'qlim', limits);
            L6 = Link('d', 0.0819,'a',0,'alpha',0, 'qlim', limits); %no limits on last joint
            
            % TODO: change default plot3dopts to have the path to the 3d
            % model of dobot
            self.robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name, 'base', base);
            self.robot.plot(q0, 'noarrow');
            
        end

        function r = getRMRCTrajectory(self, trGoal, steps, qGuess, dt)
            % gets a trajectory using a trapezoidal profile, returns a
            % matrix of joint configurations
            % tr_f is the pose (4x4 transform) of the goal location
            % steps are the number of steps desired for trajectory
            % qGuess if not specified is the current position
            % dt is the time step increment
            % uses the current joint state as initial state 
            arguments
                self
                trGoal (4,4) {mustBeNumeric}
                steps {mustBeNumeric} = 50;
                qGuess (1,:) {mustBeNomeric} = self.robot.getpos();
                dt = 0.2;
            end
            
            trInitial = self.getEndEffector(); %get transform of current end effector position

            x_0 = [trInitial(1:3,4)' tr2rpy(trInitial)]';     % extract the cartesian and rpy values from the initial transform
            x_f = [trGoal(1:3,4)' tr2rpy(trGoal)]';     % extract the cartesian and rpy values from the goal transform

            x = zeros(6,steps);     % create vector for storing waypoints
            s = lspb(0,1,steps);    % Create interpolation scalar using trapezoidal profile
            
            % Create waypoints along x,y,z,r,p,y
            for i = 1:steps
                x(:,i) = x_0*(1-s(i)) + s(i)*x_f; 
            end

            qMatrix = nan(steps,length(self.robot.links));

            qMatrix(1,:) = self.robot.ikine(trGoal, qGuess, [1 1 1 1 1 1]);  % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;               % Calculate velocity at discrete time step
                J = ur3.robot.jacob0(qMatrix(i,:));              % Get the Jacobian at the current state
                qdot = inv(J)*xdot;                              % Solve velocitities via RMRC
                qMatrix(i+1,:) = qMatrix(i,:) + dt*qdot';    % Update next joint state
            end

            r = qMatrix;

        end

    end
end



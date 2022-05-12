classdef RobotInterface < handle
    % this is the robot interface class.
    % all robotic models should inherit from this class
    % it is designed to provide the base level of interactability
    % for a given robot
    
    properties
        robot;  % serialLink object
        q0;     % nice initial joint configuration
    end
    
    methods
        % getters
        function r = getBase(self)
            % returns the base pose of the robot
            r = self.robot.base;
        end
        
        function r = getJoints(self)
            % get joint positions from SerialLink model
            r = self.robot.getpos();
            
        end
        
        function r = getEndEffector(self)
            % solve fkine from current point position
            r = self.robot.fkine(getJoints());
            
        end
        
        function r = getTrajectory(self, trGoal, steps, qGuess, mask)
            % gets the trajectory using a quintic polynomial profile and
            % returns a matrix of joint configurations.
            % goal is the pose (4x4 transform) of the location 
            % steps are the number of steps desired for trajectory 
            % qGuess is an optional parameter to specify a guess for
            % solving inverse kinematics.
            % Uses the current joint state as initial state
            arguments
                self
                trGoal
                steps
                qGuess
                mask
            end

            q0 = self.robot.getpos();
            qf = self.robot.ikcon(trGoal, qGuess, 'mask', mask);
        
            qMatrix = zeros(steps, length(self.robot.links));
            qMatrix = jtraj(q0, qf, steps);

            r = qMatrix;
        
        end
                
        % setters
        function setBase(self, pose)
            % sets the robot base to the given pose
            self.robot.base = pose;
        end
        
        % functions
        % TODO: there may not be a need for agnostic functions, put here as
        % a placeholder
        function plot(self)
            self.robot.plot(self.getJoints());
        end
        
    end
        
end
classdef RobotInterface < handle
    % this is the robot interface class.
    % all robotic models should inherit from this class
    % it is designed to provide the base level of interactability
    % for a given robot
    
    properties
        robot;  % serialLink object
    end
    
    methods
        % getters
        function r = getBase(self)
            % returns the base pose of the robot
            r = self.robot.base;
        end
        
        function r = getJoints(self)
            
        end
        
        function r = getEndefector(self)
            
        end
        
        function r = getTrajectory(self, newQ)
            % gets the trajectory using a trapezoidal trajectory
            % newQ is the new joint configuration
            % uses the current configuration

        end
        
        % setters
        function setBase(self, pose)
            % sets the robot base to the given pose
            self.robot.base = pose;
        end
        
        % functions
        % TODO: there may not be a need for agnostic functions, put here as
        % a placeholder
        
    end
        
end
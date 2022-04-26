% note this might not be the best way to do this, however it is a start at
% producing an interface class for all objects that are added to the
% simulated environment

classdef EnvironmentInterface < handle
    % an interface class that forms the basis of environment objects.
    % (meshes)
    
    properties
        mesh;  % the object mesh
        pose;  % the pose of the object
    end
    
    methods
        function self = EnvironmentInterface(initial_pose, object_file)
            % constructor for the environment interface
            % accepts an initial pose and an object file to generate
        end
        
        % setters
        function updatePose(self, new_pose)
            % updates the pose of the object
        end
        
        function setScale(self, scale)
            % sets/alters the scale of the objects mesh
            % TODO: is this possible, haven't attempted yet
        end
        
        % getters
        function r = getPose(self)
            % gets the pose of the object
        end
        
        
    end
end
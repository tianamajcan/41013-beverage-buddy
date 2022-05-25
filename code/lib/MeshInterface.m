% note this might not be the best way to do this, however it is a start at
% producing an interface class for all objects that are added to the
% simulated environment

classdef MeshInterface < handle
    % an interface class that forms the basis of environment objects.
    % (meshes)
    
    properties (Access = private)
        model;  % the object mesh
        vertices;  % vertices of object in local coordinates
        global_vertices; %verticles of the object in global coordinates
        pose;  % the pose of the object
    end
    
    methods
        function self = MeshInterface(object_file, initial_pose)
            % constructor for the environment interface
            % accepts an initial pose and an object file to generate
            
            arguments
                object_file;
                initial_pose = eye(4);  % default position
            end
            self.model = PlaceObject(object_file);
            self.vertices = get(self.model, 'Vertices');
            self.updatePose(initial_pose);
        end
        
        function delete(self)
            %  deletes the object
            delete(self.model);
        end
        
        % setters
        function updatePose(self, new_pose)
            % updates the pose of the object
            trVertices = [self.vertices, ones(size(self.vertices, 1),1)] * new_pose';
            self.pose = new_pose;
            self.global_vertices = trVertices(:,1:3);
            set(self.model, 'Vertices', trVertices(:,1:3));
        end
        
        function setScale(self, scale)
            % sets/alters the scale of the objects mesh
            % TODO: is this possible, haven't attempted yet
        end
        
        % getters
        function r = getPose(self)
            % gets the pose of the object
            r = self.pose;
        end
        
        function r = getModel(self)
            % gets the objects model
            r = self.model;
        end

        function r = getVertices(self)
            r = self.global_vertices;  % vertices at the object pose
        end
        
    end
end
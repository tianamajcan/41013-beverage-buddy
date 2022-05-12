classdef Drink < MeshInterface
    %
    
    properties
        type;  % accepts either 'coke', 'sprite', 'fanta', or 'beer'
        material; % set the material for each drink type
    end
    
    methods
        function self = Drink(type, initial_pose)
            % constructor for a drink
            % accepts a drink type, this should be either can or glass
            
            arguments
                type {mustBeText};
                initial_pose = eye(4);
            end
            
            % selecting the model to use, will raise error if something
            % unrecognised is input
            if strcmp(type, 'coke')
                model = 'coke_can.ply';
                material = 'aluminium';

            elseif strcmp(type, 'sprite')
                model = 'sprite_can.ply';
                material = 'aluminium';

            elseif strcmp(type, 'fanta')
                model = 'fanta_can.ply';
                material = 'aluminium';

            elseif strcmp(type, 'beer')
                model = 'beer_bottle.ply';
                material = 'glass';
            else
                error("Please enter a valid drink type");
            end

%             self.pose = initial_pose;
            
            % ??? this part is confusing, wtf is going on here?
            % --------------------------------%

            self@MeshInterface(model, initial_pose);
            
            % class properties need to be set after the super constructor
            self.type = type;
            self.material = material;
            % --------------------------------%
        end
        
        % getters
        function r = getDrinkType(self)
            r = self.type;
        end

        function r = getDrinkMaterial(self)
            r = self.material;
        end
        
    end
end
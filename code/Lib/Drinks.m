classdef Drinks < MeshInterface
    %
    
    properties
        drink_type;  % accepts either 'can' or 'glass'
    end
    
    methods
        function self = Drinks(drink_type, initial_pose)
            % constructor for a drink
            % accepts a drink type, this should be either can or glass
            
            arguments
                drink_type;
                initial_pose = eye(4);
            end
            
            % selecting the model to use, will default to glass if
            % something unrecognised is input
            if drink_type == 'can'
                % use the can model
                model = 'brick.ply';
            else
                % use the glass model
                model = 'brick.ply';
            end
            
            self@MeshInterface(model, initial_pose);
            
            % class properties need to be set after the super constructor
            self.drink_type = drink_type;
        end
        
        % getters
        function r = getDrinkType(self)
            r = self.drink_type;
        end
        
    end
end
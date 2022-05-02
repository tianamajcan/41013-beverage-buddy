classdef Drinks < ObjectInterface
    %
    
    properties
        drink_type;
    end
    
    methods
        function self = Drinks(drink_type)
            % constructor for a drink
            % accepts a drink type, this should be either can or glass
            
            arguments
                drink_type;
            end
            
            self.drink_type = drink_type;
        end
        
        % getters
        function r = getDrinkType(self)
            r = self.drink_type;
        end
        
    end
end
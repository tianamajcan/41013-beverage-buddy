classdef BeverageBuddy < handle
    
    properties
        buddy;
    end
    
    methods
        function self = BeverageBuddy()  % TODO: probably needs to take some arguments?
            % Beverage buddy class
            % constructs a beverage buddy object. This is used to handle
            % all the different features of the beverage buddy
        end
        
        function getDrink(self, drink)
            % performs the task of getting the drink
        end
        
        function vsExample(self)
            % runs the visual servoing example using an object to back away
            % from
        end
        
        function lightCurtain(self)
            % has a foreign object enter through the light curtain causing
            % it to stop
        end
        
        function animate(self)
            % runs the animate command for the beverage buddy
            % on top of this it polls for the emergency stop
        end
        
        
    end
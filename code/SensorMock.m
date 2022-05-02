classdef SensorMock < handle
    properties
        beverages;
    end
    
    methods
        function self = SensorMock(bevvys)
            % constructor for the mock sensor class
            self.beverages = bevvys;
        end
        
        % getters
        function r = getBeveragePositions(self)
            positions = {}; %zeros(size(self.beverages));
            for i = 1:size(self.beverages')
                positions{i} = self.beverages(i).pose;
            end
            
            r = positions;
        end
    end
end
classdef SensorLib < handle
    properties
        sensor;
    end
    
    methods
        function self = SensorLib(mock)
            % Sensor constructor
            % mock boolean if true uses the mock library
            
            if mock
                self.sensor = SensorMock();  % not sure if i can do this have to test
            end
        end
        
        function getCanPos(self)
            % returns the pos of the beverage/beverages
            % determines the pose of an OOI
        end
    end
end

classdef LightCurtain < SensorMock
    properties
        curtain_plane;  % the planet that the curtain occupies
    end
    
    methods
        function self = LightCurtain(sensed_objects, curtain_plane, sensor_name)
            % class for the light curtain
            % accepts a list of objects that can pass through the light
            % curtain plane
            
            arguments
                sensed_objects;
                curtain_plane;
                sensor_name {mustBeText} = '';
            end
            sensor_type = 'LightCurtain';
            
            % calling the sensor mock constructor
            self@SensorMock(sensor_type, sensed_objects, sensor_name);
            
            self.curtain_plane = curtain_plane;
        end
        
        function makeVisible(self)
            % what it says on the tin
        end
        
        function makeInvisible(self)
            % again, what it says on the tin
        end
            
        function r = getSensorResult(self)
            % returns a boolean value representing if the light curtain has
            % been interrupted or not.
            % 1 is interrupted
            % 0 is uninterrupted
            
            % test the collision
            r = 0;
        end
    end
    
end
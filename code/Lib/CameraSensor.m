% hypothetically we could use a real camera and put an invisible sphere in
% any object of interest - probably don't have the time to get it working
% food for thought if anyone else has spare time haha
% also would mean that the vscam could just use a camera sensor

classdef CameraSensor < SensorMock
    properties
    end
    
    methods
        function self = CameraSensor(sensed_objects, sensor_name)
            % class for the camera sensor
            % accepts a list of objects that can be sensed and an optional
            % name for the sensor, if more than one camera is being used
            
            arguments
                sensed_objects;
                sensor_name {mustBeText} = '';
            end
            sensor_type = 'Camera';

            % calling the sensor mock constructor
            self@SensorMock(sensor_type, sensed_objects, sensor_name);
        end

        
        function r = getSensorResult(self)
            % returns the result for all objects sensed by the camera
            % it is assumed that the camera can see all of its objects all
            % of the time for simplicity
            r = self.getObjectPoses();
        end
        
        
    end
    
end
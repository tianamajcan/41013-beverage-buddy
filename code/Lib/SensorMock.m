classdef SensorMock < handle
    properties
        sensor_type;  % type of sensor being used, such as camera or light curtain
        sensed_objects;  % objects tracked by the sensor
        sensor_name;  % name of the specific sensor
    end
    
    methods
        function self = SensorMock(sensor_type, sensed_objects, sensor_name)
            % SensorMock is a base class for use in simulating various
            % sensors. It is sensor agnostic and is used primarily for
            % recording sensable objects
            arguments
                sensor_type {mustBeText};
                sensed_objects;
                sensor_name {mustBeText} = '';
            end
            
            self.sensor_name = sensor_name
            self.sensor_type = sensor_type;
            self.sensed_objects = sensed_objects;
        end
        
        % getters
        function r = getObjectPoses(self)
            % a function that returns the poses of all the sensed objcets
            % may not be able to be applied on all objects
            
            try
                disp('i tried');
                r = self.sensed_objects.pose;
                disp('i succeded');
            catch ME
                disp('Unable to automatically recieve objects pose \n manual pose extraction must be used');
            end
            
        end
            
        function r = getSensorResult(self)
            % overwritten function by sensor type
            error('Function needs to be implemented by sub class')
        end
    end
end
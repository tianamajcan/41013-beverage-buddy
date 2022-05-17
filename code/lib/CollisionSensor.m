classdef CollisionSensor < SensorMock
    % COLLISIONSENSOR Detects proximity of robots to other objects in the
    % environment. 
    % Uses point in ellipsoid method to check for distance til collision
    
    properties
        sensed_robots
    end
    
    methods
        function self = CollisionSensor(robots, objects, name)
            %COLLISIONSENSOR Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                robots
                objects
                name = "Collision Sensor"
            end
            self.sensed_objects = objects;
            self.sensed_robots = robots;
            self.sensor_name = name;

        end
        
        function [robots, objects] = getSensorResult(self)
            %METHOD1 Checks the proximity of if any of the robots are close
            % to collision with the objects
           
        end

    end
end


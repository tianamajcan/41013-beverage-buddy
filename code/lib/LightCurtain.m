classdef LightCurtain < SensorMock
    properties
        plane_normal;  % the plane that the curtain occupies
        plane_point;
        corner_points;
        plane_plot;
    end
    
    methods
        
        function self = LightCurtain(sensed_objects, corner_points, sensor_name)
            % class for the light curtain
            % accepts a list of objects that can pass through the light 
            % curtain plane
            % NOTE: this will only work with robot for now
            
            arguments
                sensed_objects;
                corner_points (4,3);
                sensor_name {mustBeText} = 'LightCurtain';
            end
            sensor_type = 'LightCurtain';

            
            % calling the sensor mock constructor
            self@SensorMock(sensor_type, sensed_objects, sensor_name);
            
            self.corner_points = corner_points;
            self.plane_point = corner_points(1,:);

            % get the plane normal from the points
            V1 = corner_points(1,:) - corner_points(2,:);
            V2 = corner_points(3,:) - corner_points(2,:);

            self.plane_normal = cross(V1, V2);
        end
        
        function makeVisible(self)
            % grab the x, y, z corners from the corner points array 
            X = [self.corner_points(1:2,1)'; self.corner_points(3:4,1)'];
            Y = [self.corner_points(1:2,2)'; self.corner_points(3:4,2)'];
            Z = [self.corner_points(1:2,3)'; self.corner_points(3:4,3)'];

            % plot transparent plane
            self.plane_plot = surf(X, Y, Z, 'FaceColor', 'r', 'FaceAlpha', 0.3,'EdgeColor','r'); 
        end
        
        function makeInvisible(self)
            delete(self.plane_plot);
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
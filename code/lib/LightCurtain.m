classdef LightCurtain < SensorMock
    properties
        plane_normal;  % the plane that the curtain occupies
        plane_point;
        corner_points;
        plane_plot;
    end
    
    methods
        
        function self = LightCurtain(sensed_objects, sensed_robots, corner_points, sensor_name)
            % class for the light curtain
            % accepts a list of objects that are being tracked 
            % curtain plane
            % NOTE: this will only work with robot for now
            
            arguments
                sensed_objects;
                sensed_robots;
                corner_points (4,3);
                sensor_name {mustBeText} = 'LightCurtain';
            end
            sensor_type = 'LightCurtain';

            
            % calling the sensor mock constructor
            self@SensorMock(sensor_type, sensed_objects, sensed_robots, sensor_name);
            
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

%             for i = 1:length(self.sensed_objects)
%                 obj = self.sensed_objects{i};
%                 % check the objects superclasses
%                 classes = superclasses(obj);
% 
%                 for j = 1:length(classes)
%                     % check if sensed object is a robot
%                     if (strcmp('RobotInterface', classes{j}))
%                         lineSegments = obj.getLinksAsLines();
            for i = 1:length(self.sensed_robots)
                lineSegments = self.sensed_robots{i}.getLinksAsLines();
                        % check if any of the robot links intersect the plane, if
                        % so return 1 
                        for k = 1:size(lineSegments,3)
                            % check if any links are intersecting
                            [point, check] = LinePlaneIntersection(self.plane_normal, self.plane_point, lineSegments(1,:,k), lineSegments(2,:,k));

                            if ((check == 1) || (check == 2)) 
                                % since the plane is mathematically defined
                                % as infinite, we must check if the
                                % intersection has occures within the
                                % bounds of the user defined "window" in
                                % which the light curtain physically exists

                                % get the max and min x,y,z coordinates on
                                % the window
                                ranges = [min(self.corner_points(:,1)), max(self.corner_points(:,1));
                                          min(self.corner_points(:,2)), max(self.corner_points(:,2));
                                          min(self.corner_points(:,3)), max(self.corner_points(:,3))];
                                
                                % only check within the dimensions where
                                % there is a difference between the max and
                                % min values
                                empty = ((ranges(:,2) - ranges(:,1)) ~= 0);
                                ranges = ranges(empty,:);
                                checkPoint = point(empty)';

                                % checks if the point is within the bounds
                                if ((ranges(:,1) < checkPoint) & (checkPoint < ranges(:,2)))
                                    disp(sprintf("!!!COLLISION DETECTED!!!\nRobot: %s  Point [%.3f,%.3f,%.3f]\nEmergency stop initiated.", self.sensed_robots{i}.robot.name, point(1), point(2), point(3)));
                                    r = 1;
                                    return
                                end
                            end
                        end
                    end                
%                 end
%             end
            
            r = 0;
        end

    end

end
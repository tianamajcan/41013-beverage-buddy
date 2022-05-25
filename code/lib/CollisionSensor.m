classdef CollisionSensor < SensorMock
    % COLLISIONSENSOR Detects proximity of robots to other objects in the
    % environment. 
    % Uses point in ellipsoid method to check for distance til collision
    
    properties
    end
    
    methods
        function self = CollisionSensor(sensed_objects, sensed_robots, sensor_name)
            %COLLISIONSENSOR Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                sensed_objects;
                sensed_robots;
                sensor_name = "Collision Sensor";
            end

            sensor_type = 'Collision Sensor';

            % calling the sensor mock constructor
            self@SensorMock(sensor_type, sensed_objects, sensed_robots, sensor_name);

        end
        
        function [r, robot, object] = getSensorResult(self)
            %METHOD1 Checks the proximity of if any of the robots are close
            % to collision with the objects, where r = 1 means an upcoming
            % collision has been detected and r = 0 means no incoming
            % collisions detected

            % loop through the robots current in the scene
            for i = 1:length(self.sensed_robots)
                n = length(self.sensed_robots{i}.robot.links);                  % num links
                joints = self.sensed_robots{i}.getJointTransforms();    %transforms of the joins
                
                % loop through every object detected by the sensor
                for j = 1: length(self.sensed_objects)
                    vertices = self.sensed_objects{j}.getVertices();
                    
                    % loops through every link (including the base, i.e.
                    % every ellipsoid) EXCEPT the end effector for the
                    % purposes of this simulation 
                    for k = 1:n
                        % add column of ones to be allow calculating the transform the object points
                        verticesAndOnes = [inv(joints(:,:,k)) * [vertices,ones(size(vertices,1),1)]']';
                        updatedVertices = verticesAndOnes(:,1:3);   % remove column of ones after trasnform
                        % return the (normalised) distance of a set of
                        % points from the center of the ellipsoid. If the
                        % distance is > 1 then the point is not within the
                        % sphere, and if its greater than it is
                        dist = getAlgebraicDist(updatedVertices, self.sensed_robots{i}.ellipsoid_centres(k,:), self.sensed_robots{i}.ellipsoid_radii(k,:));
                        pointsInside = find(dist<1);
                        
                        if pointsInside > 1
                            % return true
                            r = 1;
                            robot = self.sensed_robots{i};
                            object = self.sensed_objects{j};
                            pose = object.getPose();
                            disp(sprintf("WARNING: PERFORMING EMERGENCY STOP TO AVOID COLLISION!\n" + ...
                                "Upcoming collision detected on the %dth link of robot %s " + ...
                                "with object at location %.3f, %.3f, %.3f", k, robot.robot.name, pose(1,4), pose(2,4), pose(3,4)))
                            return
                        else
                            r = 0;
                            robot = nan;
                            object = nan;
                        end
                    end
                end
            end
        end

    end

end


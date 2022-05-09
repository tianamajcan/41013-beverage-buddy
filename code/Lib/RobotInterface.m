classdef RobotInterface < handle
    % this is the robot interface class.
    % all robotic models should inherit from this class
    % it is designed to provide the base level of interactability
    % for a given robot
    
    properties
        robot;  % serialLink object
        cam;
    end
    
    methods
        % getters
        function r = getBase(self)
            % returns the base pose of the robot
            r = self.robot.base;
        end
        
        function r = getJoints(self)
            % returns the joint state of the robot
            r = self.robot.getpos;
        end
        
        function r = getEndefector(self)
            % returns the endefector position of the robot
            r = self.robot.fkine(self.robot.getpos);
        end
        
        function r = getTrajectory(self, newQ)
            % gets the trajectory using a trapezoidal trajectory
            % newQ is the new joint configuration
            % uses the current configuration

        end
        
        % setters
        function setBase(self, pose)
            % sets the robot base to the given pose
            self.robot.base = pose;
        end
        
        function setJoints(self, joints)
            % animates the robot to the given joints
            self.robot.animate(joints)
        end
        
        % functions
        % TODO: there may not be a need for agnostic functions, put here as
        % a placeholder
        
        function showRobot(self)
            % plots the robot in the default configuration. Not the only
            % way to get the robot to be displayed.
            % this may be removed if there is a better way to do this.
            % this uses plot, so it won't update the view of the robot
            links = size(self.robot.links);
            self.robot.plot(zeros(links));
        end
        
        function updateAll(self)
            % animate the robot to the current position to force an update
            % draw the camera
            % does nothing if the paramaters are not yet set
            try
                self.robot.animate(self.robot.getpos());
            end
            try
                self.vsUpdateCamera();
            end
        end
        
        % visual servoing
        % visual servoing uses a real camera
        function vsCreateCamera(self)
            % creates a camera for visual servoing
            % defaults to not display the camera
            
            % check that the robot is being displayed
            % if its not don't create the camera
            try
                pos = self.getEndefector;
            catch
                error(['robot not displayed. cannot create camera.\n',...
                      'make sure the robot has been plotted']);
            end
            
            % create and plot the camera
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024, 1024], 'centre', [512, 512], ...
                'name', strcat('vscam:', self.robot.name));
            
            % plotting the robot
            self.cam.plot_camera;
            self.cam.T = pos;
        end

        function vsDestroyCamera(self)
            % destroys the camera for visual servoing
            self.cam.delete;
            self.cam = [];  % need to reset the cam variable to []
        end
        
        function vsUpdateCamera(self)
            % updates the visual servoing camera to the current
            % endeffector position
            try
                self.cam.T = self.getEndefector;
            catch
                error(['unable to update camera position\n',...
                       'possibly due to unrended robot']);
            end
        end
        
        function vsMove(self, object_points, target_points)
            % uses visual servoing to move the endeffector to the desired
            % target position
            
            q0 = self.getJoints;
            cam_pos = self.getEndefector;
            
            % plotting the camera display
            self.cam.plot(target_points, '*');
            self.cam.hold(true);
            self.cam.plot(object_points, 'Tcam', cam_pos, 'o');
            
            % variables
            lambda = 0.6;
            depth = 1.8;
            
            % motion loop
            steps = 200;
            for i = 1:steps
                % compute the view
                uv = self.cam.plot(object_points);  % gets an updated view of the points
                
                % get the error to the target points
                e = target_points - uv;
                e = e(:);
                
                % compute the jacobian of the current image
                J = self.cam.visjac_p(uv, depth);
                
                % compute the velocity
                v = lambda * pinv(J) * e;
                
                % compute joint velocities
                J2 = self.robot.jacobn(q0);  % cam_pos is the same as the endefector pos
                Jinv = pinv(J2);
                qp = Jinv * v;
                
                % TODO: add in the maximum angular velocity
                
                % update joint positions
                q = q0 + (1/25) * qp;
                self.setJoints(q');  % set the robot joints to the new configuration
                self.vsUpdateCamera;  % camera needs to be moved to the new position as well
                
                % draw
                drawnow
                
                % update the position
                q0 = q;
                
            end
            
            self.cam.hold(false);
            % display the object points on the camera display
            
        end
        
    end
        
end
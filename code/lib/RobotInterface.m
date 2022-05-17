classdef RobotInterface < handle
    % this is the robot interface class.
    % all robotic models should inherit from this class
    % it is designed to provide the base level of interactability
    % for a given robot
    
    properties
        robot;  % serialLink object
        q0;     % nice initial joint configuration
        cam;
        ellipsoid_radii;
        ellipsoid_centres;
        joint_trs;
    end
    
    methods
        % getters
        function r = getBase(self)
            % returns the base pose of the robot
            r = self.robot.base;
        end
        
        function r = getJoints(self)
            % get joint positions from SerialLink model
            r = self.robot.getpos();
            
        end
        
        function r = getEndEffector(self)
            % solve fkine from current point position
            r = self.robot.fkine(self.robot.getpos());
            
        end
        
        function r = getTrajectory(self, trGoal, steps, qGuess)
            % gets the trajectory using a quintic polynomial profile and
            % returns a matrix of joint configurations.
            % goal is the pose (4x4 transform) of the location 
            % steps are the number of steps desired for trajectory 
            % qGuess is an optional parameter to specify a guess for
            % solving inverse kinematics.
            % Uses the current joint state as initial state
            arguments
                self;
                trGoal;
                steps = 50;
                qGuess = self.q0;
            end

            q0 = self.robot.getpos();
            qf = self.robot.ikcon(trGoal, qGuess);
        
            qMatrix = zeros(steps, length(self.robot.links));
            qMatrix = jtraj(q0, qf, steps);

            r = qMatrix;
        
        end

        function r = getLinksAsLines(self)
            % gets each link as a line segment defined by beginning point and end
            % point
            L = self.robot.links;  
            n = length(L); 
            q = self.getJoints();
           
            trs = zeros(4, 4, n+1);  % array to hold the pose of every joint
            linePoints = zeros(2, 3, n); % array to hold the line segment points
            
            trs(:,:,1) = self.robot.base;   % first transform is the base pose
            
            % calculate transform of every joint using forward kinematics
            for i = 1:n
                trs(:,:,i+1) = trs(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            % get the cartesian coordinates of each joint
            for i = 1:n
                linePoints(1,:,i) = trs(1:3,4, i)';
                linePoints(2,:,i) = trs(1:3,4, i+1)';
            end
            
            r = linePoints;
        end

        function r = getJointTransforms(self)
            L = self.robot.links;  
            n = length(L); 
            q = self.getJoints();
           
            trs = zeros(4, 4, n+1);  % array to hold the pose of every joint
            trs(:,:,1) = self.robot.base;   % first transform is the base pose
            
            % calculate transform of every joint using forward kinematics
            for i = 1:n
                trs(:,:,i+1) = trs(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            % get the cartesian coordinates of each joint
            for i = 1:n+1
                self.joint_trs(i,:) = trs(1:3,4, i)';
            end

            r = self.joint_trs;
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

        function setLinksAsEllipsoids(self)
            % sets the robot model to have each link approximated by an
            % ellipsoid
            n = length(self.robot.links);

            % get links as lines
            linePoints = self.getLinksAsLines();

            radii = zeros(length(n+1), 3);      % preallocate
            vertices = cell(n+1, 3);
            centrePoints = zeros(n+1, 3);

            % make the base ellipsoid a short sphere
            radii(1,:) = [0.05, 0.05, 0.05];
            [vertices{1,1},vertices{1,2},vertices{1,3}] = ellipsoid(0, 0, 0, radii(1,1), radii(1,2), radii(1,3));
            centrePoints(1,:) = [0 0 0];

            % get the rest of the ellipsoids for each link
            for i = 2:size(linePoints, 3)+1
                % find the midpoint of each link
                dx = abs((linePoints(1,1,i-1) - linePoints(2,1,i-1)));
                dy = abs((linePoints(1,2,i-1) - linePoints(2,2,i-1)));
                dz = abs((linePoints(1,3,i-1) - linePoints(2,3,i-1)));
                
                % check if the link has a component in the z direction
                if (self.robot.links(i-1).d ~= 0)
                    if self.robot.links(i-1).alpha == pi/2
                        radii(i,2) = max([dx dy dz])/2 + 0.05;     % the prinicple radii should half the length of the longest axis plus a little extra
                        radii(i,1) = radii(i,2)*0.3;    % the other radii can be proportional to the main one
                        radii(i,3) = radii(i,2)*0.3;    % the other radii can be proportional to the main one
                        centrePoints(i,:) = [0, 0-max([dx dy dz])/2, 0];
                        [vertices{i,1},vertices{i,2},vertices{i,3}] = ellipsoid(centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radii(i,1), radii(i,2), radii(i,3));

                    else
                        radii(i,2) = max([dx dy dz])/2 + 0.05;     % the prinicple radii should half the length of the longest axis plus a little extra
                        radii(i,1) = radii(i,2)*0.3;    % the other radii can be proportional to the main one
                        radii(i,3) = radii(i,2)*0.3;    % the other radii can be proportional to the main one
                        centrePoints(i,:) = [0, 0+max([dx dy dz])/2, 0];
                        [vertices{i,1},vertices{i,2},vertices{i,3}] = ellipsoid(centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radii(i,1), radii(i,2), radii(i,3));
                    end
                
                % check if the link has a component in the x direction
                elseif (self.robot.links(i-1).a ~= 0)
                    radii(i,1) = max([dx dy dz])/2 + 0.1;     % the prinicple radii should half the length of the longest axis plus a little extra
                    radii(i,2:3) = radii(i,1)*0.3;    % the other radii can be proportional to the main one
                    centrePoints(i,:) = [0+radii(i,1)-0.1, 0, 0];
                    [vertices{i,1},vertices{i,2},vertices{i,3}] = ellipsoid(centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radii(i,1), radii(i,2), radii(i,3));
                else
                    % do default
                    radii(i,1) = max([dx dy dz])/2 + 0.05;     % the prinicple radii should half the length of the longest axis plus a little extra
                    radii(i,2:3) = radii(i,1)*0.25;    % the other radii can be proportional to the main one
                    centrePoints(i,:) = [0 0 0];
                    [vertices{i,1},vertices{i,2},vertices{i,3}] = ellipsoid(centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radii(i,1), radii(i,2), radii(i,3));
                end

                if i == (size(linePoints, 3)+1)
                    radii(i,3) = max([dx dy dz])/2 + 0.05;     % the prinicple radii should half the length of the longest axis plus a little extra
                    radii(i,1:2) = radii(i,3)*0.3;    % the other radii can be proportional to the main one
                    centrePoints(i,:) = [0, 0, 0-max([dx dy dz])/2];
                    [vertices{i,1},vertices{i,2},vertices{i,3}] = ellipsoid(centrePoints(i,1), centrePoints(i,2), centrePoints(i,3), radii(i,1), radii(i,2), radii(i,3));
                end
            end

            for i = 1:n+1   
                self.robot.points{i} = [vertices{i,1}(:), vertices{i,2}(:),vertices{i,3}(:)];
                warning off
                self.robot.faces{i} = delaunay(self.robot.points{i});
                warning on
            end

            self.ellipsoid_centres = centrePoints;
            self.ellipsoid_radii = radii;

        end

        % functions
        function plot(self)
            % plots the robot, depricated by showRobot
            self.robot.plot(self.getJoints());
        end
        
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
        function vsCreateCamera(self)
            % creates a camera for visual servoing
            % defaults to not display the camera
            
            % check that the robot is being displayed
            % if its not don't create the camera
            try
                pos = self.getEndEffector;
            catch
                error(['robot not displayed. cannot create camera.\n',...
                      'make sure the robot has been plotted']);
            end
            
            % create and plot the camera
            self.cam = CentralCamera('focal', 0.09, 'pixel', 10e-5, ...
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
            % EndEffector position
            try
                self.cam.T = self.getEndEffector;
            catch
                error(['unable to update camera position\n',...
                       'possibly due to unrended robot']);
            end
        end
        
        function vsMove(self, object_points, target_points)
            % uses visual servoing to move the EndEffector to the desired
            % target position
            
            q0 = self.getJoints';  % this needs to be transposed, either that or everything else needs to be (they just gotta be the same and this is how its done in the tut so don't go breaking it)
            cam_pos = self.getEndEffector;
            
            % plotting the camera display
            self.cam.plot(target_points, '*');
            self.cam.hold(true);
            self.cam.plot(object_points, 'Tcam', cam_pos, 'o');
            
            % variables
            lambda = 0.6;
            depth = 1.8;
            
            % motion loop
            steps = 100;
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
                J2 = self.robot.jacobn(q0);  % cam_pos is the same as the EndEffector pos
                Jinv = pinv(J2);
                qp = Jinv * v;
                
                % TODO: add in the maximum angular velocity
                ind=find(qp>pi);
                 if ~isempty(ind)
                     qp(ind)=pi;
                 end
                 ind=find(qp<-pi);
                 if ~isempty(ind)
                     qp(ind)=-pi;
                 end
                
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

        function vsDisplayCamera(self, object_points, target_points)
            % used to display the vs camera for testing purposes
            
            % plotting the camera display
            self.cam.plot(target_points, '*');
            self.cam.hold(true);
            self.cam.plot(object_points, 'Tcam', self.getEndEffector, 'o');
            self.cam.hold(false);
            pause;
            
        end
    end
end
classdef BeverageBuddy < handle
    
    properties
        drinks;
        objects;
        ur3;
        dobot;
        sensors;
        drinkOffset;
        estop = 0;
        resume = 0;
    end
    
    methods
        function self = BeverageBuddy()  % TODO: probably needs to take some arguments?
            % Beverage buddy class
            % constructs a beverage buddy object. This is used to handle
            % all the different features of the beverage buddy

            % should either launch GUI, or launch an instance of beverage buddy
            % from the GUI (probably the latter)

            % pass the poses of the drinks?
            
            % set up the environment
            [self.drinks, self.objects] = environmentSetup(figure(1));
            self.drinkOffset = transl(0, 0, -0.08);
            
            % place robots
            self.ur3 = UR3(transl(-0.7, -0.6,0.9)*trotz(-pi/2));
            self.dobot = Dobot(transl(-0.6, -1.4,0.85));

            % place sensors
            self.sensors{1} = LightCurtain(self.objects, {self.ur3, self.dobot}, [-0.5, 0, 0.78; -0.5, 0, 1.5; -1.5, 0, 0.78; -1.5, 0, 1.5], "Light Curtain");
            self.sensors{2} = CollisionSensor(self.objects, {self.ur3,self.dobot}, "Magic Collision Sensor");

            self.sensors{1}.makeVisible()
            
        end
        
        % getting drinks
        
        function getDrink(self, drink, drink_index)
            % performs the task of getting the drink
            
            % moving to the can
            qGuess = [0.6283, -1.0367, 0.9061, 0.0942, 0.5655, 0]
            steps = 50;
            traj = self.ur3.getTrajectory(self.drinks{drink_index,drink}.getPose()*transl(0, 0, 0.08)*trotx(pi/2)*troty(-pi/2), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % taking the can out
            steps = 20;

            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0,0,-0.3), steps, self.ur3.getJoints());

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % move sideways, holding the can upright
            steps = 50;
            qGuess = [0.7263   -1.9189    1.8735    0.0189    2.2941   -0.0176];
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.2,0,0)*trotz(-pi/2), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % move to the coaster
            steps = 50;
            R = self.ur3.getEndEffector();
            R(1:3,4) = 0;       % get the rotation of the end effector
            traj = self.ur3.getRMRCTrajectory(transl(-0.75,-0.1,0.87)*R, steps);    % hard code coaster location
%             traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.1,-0.4,0.4)*transl(0,0.07,0), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % move away from the coaster
            steps = 20;
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(0,0.2,-0.2), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                self.checkEstop();
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
            end
            
        end
        
        function giveToBuddy(self, drink, drink_index)
            % go to pick up the can again
            steps = 20;
            traj = self.ur3.getRMRCTrajectory(self.drinks{drink_index,drink}.getPose()*transl(0, 0, 0.08)*trotx(pi/2), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % grab can and move away
            steps = 30;
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.2,0.2,-0.1), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end
            
            % give the can to the dobot
            steps = 100;
            qGuess = ([-3.5023, -1.0329, 1.6342, -0.6268, 0.8303, 0.0170]);
            traj = self.ur3.getTrajectory(transl(-0.6,-1.09,0.86)*trotx(pi/2), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end

            steps = 20;
            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0, 0,-0.2), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end

            self.drinks{drink_index,drink}.updatePose(self.drinks{drink_index,drink}.getPose()*trotz(pi/2));

            % dobot grab the empty can/bottle
            steps = 20;
            traj = self.dobot.getTrajectory(self.drinks{drink_index,drink}.getPose()*trotx(pi)*trotz(pi/2)*transl(0,0,-0.13), steps);

            for i = 1:steps
                self.dobot.robot.animate(traj(i,:));
                
                if (self.sensors{1}.getSensorResult() == 1)
                    self.toggleEstop();
                end
                self.checkEstop();
            end

            % dobot grab the can
            if drink == 4  % bottle
                % move to the yellow bin
                steps = 50;

                traj = self.dobot.getTrajectory(transl(-0.25,-1.2,0.8)*trotx(pi)*trotz(pi/2), steps);

                for i = 1:steps
                    self.dobot.robot.animate(traj(i,:));
                    tr = self.dobot.getEndEffector();
                    self.drinks{drink_index,drink}.updatePose(tr*trotx(pi)*transl(0,0,-0.13));
                    
                    if (self.sensors{1}.getSensorResult() == 1)
                        self.toggleEstop();
                    end
                    self.checkEstop();
                end

            else
%                 % move to the red bin
%                 steps = 20;
%                 traj = self.dobot.getTrajectory(self.drinks{drink_index,drink}.getPose()*trotx(pi)*trotz(pi/2)*transl(0,0,-0.13), steps);
% 
%                 for i = 1:steps
%                     self.dobot.robot.animate(traj(i,:));
%                     self.checkEstop();
%                     if (self.sensors{1}.getSensorResult() == 1)
%                         self.toggleEstop();
%                     end
%                 end

                % put in the red bin
                steps = 50;

                traj = self.dobot.getTrajectory(transl(-0.25,-1.55,0.8)*trotx(pi)*trotz(pi/2), steps);

                for i = 1:steps
                    self.dobot.robot.animate(traj(i,:));
                    tr = self.dobot.getEndEffector();
                    self.drinks{drink_index,drink}.updatePose(tr*trotx(pi)*transl(0,0,-0.13));
                    
                    if (self.sensors{1}.getSensorResult() == 1)
                        self.toggleEstop();
                    end

                    self.checkEstop();
                end
            end

            % drink falling in the bin
            for i = 1:10
                self.drinks{drink_index,drink}.updatePose(self.drinks{drink_index,drink}.getPose()*transl(0,0,-0.01*i));
                pause(0.05);
                drawnow();
            end

        end
        
        % translation for UR3
        
        function translateUR3(self, translation)
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*translation, 2);
            disp(traj(2,:))
            disp(self.ur3.getJoints)
            self.ur3.robot.animate(traj(2,:));
        end
        
        %translation for Dobot
        
        function translateDobot(self, translation)
            traj = self.dobot.getRMRCTrajectory(self.dobot.getEndEffector()*translation, 2);
            disp(traj(2,:))
            disp(self.dobot.getJoints)
            self.dobot.robot.animate(traj(2,:));
        end 
        
        % visual servoing example
        
        function vsExample(self)
            % runs the visual servoing example using an object to back away
            % from
            % create the visual servoing object (points + object)
            warn = self.objects{1}.getPose;
            x = warn(1,4);
            y = warn(2,4);
            z = warn(3,4);
            P=[x-0.25,x-0.25,x+0.25,x+0.25;
                y,y,y,y;
                z+0.5,z+1,z+0.5,z+1];  % corners of the warning sign
            pStar = [312 312 712 712;
                     312 712 312 712];  % target
            plot_sphere(P, 0.05, 'g'); %  don't need to show this
            
            % create the camera
            self.ur3.vsCreateCamera;
            
            % move the head into a position where it can see object
            start_pos = [-3.39292006587698,-0.911061869541039,1.21475579163921,-0.282743338823082,1.31946891450771,8.88178419700125e-16];  % arbitrary point located with teach
            self.ur3.setJoints(start_pos);
            self.ur3.vsUpdateCamera;
            
            % run vsmove
            self.ur3.vsMove(P, pStar);

        end
        
        % light curtain example: May be depricated
        
        function lightCurtain(self)
            % has a foreign object enter through the light curtain causing
            % it to stop
        end

        function collisionAvoidanceExample(self)
            ur3.setLinksAsEllipsoids()
            ur3.plot3d(ur3.q0)

            [r, robot, object] = sensors{2}.getSensorResult(ur3, objects{4})
            if r == 1
                self.toggleEstop()
            end
        end
        
        % estop functions
        
        function checkEstop(self)
            % polls the estop and waits for the reset
            if self.estop == 1
                disp('EMERGENCY STOP');
                self.resume = 0;  % ensure that the resume state hasn't been accidentally set
                while self.resume == 0
                    pause(1);  % just don't want it to run too fast
                end
                self.resume = 0;  % reset resume, estop should already be reset.
                disp('RESUMING');
            end
        end
        
        function toggleEstop(self)
            % toggles the estop
            if self.estop == 1
                self.estop = 0;
            else
                self.estop = 1;
            end
            
            self.estop
        end
        
    end
end
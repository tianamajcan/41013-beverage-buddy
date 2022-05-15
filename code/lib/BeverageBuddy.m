classdef BeverageBuddy < handle
    
    properties
        drinks;
        objects;
        ur3;
        dobot;
        drinkOffset;
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
            
        end
        
        function getDrink(self, drink, drink_index)
            % performs the task of getting the drink
            
            % moving to the can
            steps = 50;
            traj = self.ur3.getTrajectory(self.drinks{drink_index,drink}.getPose()*transl(0, 0, 0.08)*trotx(pi/2)*troty(-pi/2), steps, self.ur3.q0);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
            end
            
            % taking the can out
            steps = 20;

            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0,0,-0.3), steps, self.ur3.getJoints());

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
            end
            
            % move sideways, holding the can upright
            steps = 50;

            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.2,0,0)*trotz(-pi/2), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
            end
            
            % move to the coaster
            steps = 50;
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.1,-0.4,0.4)*transl(0,0.07,0), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
            end
            
            % move away from the coaster
            steps = 20;
            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0,0.2,-0.3), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
            end
            
            % go to pick up the can again
            steps = 20;
            traj = self.ur3.getRMRCTrajectory(self.drinks{drink_index,drink}.getPose()*transl(0, 0, 0.08)*trotx(pi/2), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
            end
            
            % grab can and move away
            steps = 30;
            traj = self.ur3.getRMRCTrajectory(self.ur3.getEndEffector()*transl(-0.2,0.2,-0.1), steps);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
            end
            
            % give the can to the dobot
            steps = 100;
            qGuess = ([-3.5023, -1.0329, 1.6342, -0.6268, 0.8303, 0.0170]);
            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0,-0.15,-0.9)*troty(pi), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
                tr = self.ur3.getEndEffector();
                self.drinks{drink_index,drink}.updatePose(tr*trotx(-pi/2)*self.drinkOffset);
            end

            steps = 20;
            traj = self.ur3.getTrajectory(self.ur3.getEndEffector()*transl(0, 0,-0.2), steps, qGuess);

            for i = 1:steps
                self.ur3.robot.animate(traj(i,:));
            end

            self.drinks{drink_index,drink}.updatePose(self.drinks{drink_index,drink}.getPose()*trotz(pi/2));
            
            % dobot grab the can
            if drink == 4  % bottle
                % move to the yellow bin
            else
                % move to the red bin
                steps = 20;
                traj = self.dobot.getTrajectory(self.drinks{drink_index,drink}.getPose()*trotx(pi)*trotz(pi/2)*transl(0,0,-0.13), steps);

                for i = 1:steps
                    self.dobot.robot.animate(traj(i,:));
                end
                
                % put in the red bin
                steps = 50;

                traj = self.dobot.getTrajectory(transl(-0.25,-1.55,0.8)*trotx(pi)*trotz(pi/2), steps);

                for i = 1:steps
                    self.dobot.robot.animate(traj(i,:));
                    tr = self.dobot.getEndEffector();
                    self.drinks{drink_index,drink}.updatePose(tr*trotx(pi)*transl(0,0,-0.13));
                end
                
                % drink falling in the bin
                for i = 1:10
                    self.drinks{drink_index,drink}.updatePose(self.drinks{drink_index,drink}.getPose()*transl(0,0,-0.01*i));
                    pause(0.05);
                    drawnow();
                end
            end
            
        end
        
        function vsExample(self)
            % runs the visual servoing example using an object to back away
            % from
        end
        
        function lightCurtain(self)
            % has a foreign object enter through the light curtain causing
            % it to stop
        end
        
        
        function checkEstop(self)
            % polls the estop and waits for the reset
        end
        
        
    end
end
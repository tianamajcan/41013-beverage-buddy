% main program for the robot project

[drinks, objects] = environmentSetup(figure(1));

% place robots
ur3 = UR3(transl(-0.7, -0.6,0.9)*trotz(-pi/2));
dobot = Dobot(transl(-0.6, -1.4,0.85));

%% go to can
drinkOffset = transl(0, 0, -0.08);

steps = 50;
traj = ur3.getTrajectory(drinks{3,1}.getPose()*transl(0, 0, 0.08)*trotx(pi/2)*troty(-pi/2), steps, ur3.q0);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
end
 
%% take can out
steps = 20;
qGuess = [0.8501   -1.0351    0.7713    0.2241    0.8594    0.0375]
traj = ur3.getTrajectory(ur3.getEndEffector()*transl(0,0,-0.3), steps, qGuess);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
    tr = ur3.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(-pi/2)*drinkOffset);
end

%% use rmrc to move sideways
steps = 50;
qGuess = [0.7263   -1.9189    1.8735    0.0189    2.2941   -0.0176];
traj = ur3.getRMRCTrajectory(ur3.getEndEffector()*trotz(-pi/2), steps, qGuess);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
    tr = ur3.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(-pi/2)*drinkOffset);
end

%% use rmrc to move to coaster %%% fix to global coordinates
steps = 50;
% traj = ur3.getRMRCTrajectory(ur3.getEndEffector()*transl(-0.1,-0.4,0.4)*transl(0,0.07,0), steps);
R = ur3.getEndEffector();
R(1:3,4) = 0;
traj = ur3.getRMRCTrajectory(transl(-0.75,-0.1,0.87)*R, steps);
% transl(-0.75,-0.1,0.78)
% bot = ur3.getEndEffector();
% goal = objects{5}.getPose();

for i = 1:steps
    ur3.robot.animate(traj(i,:));
    tr = ur3.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(-pi/2)*drinkOffset);
end

%% use rmrc to move backwards
steps = 20;
traj = ur3.getTrajectory(ur3.getEndEffector()*transl(0,0.2,-0.3), steps);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
end

%% go to can again
steps = 20;
traj = ur3.getRMRCTrajectory(drinks{3,1}.getPose()*transl(0, 0, 0.08)*trotx(pi/2), steps);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
end

%% grab can and move upwards
steps = 30;

traj = ur3.getRMRCTrajectory(ur3.getEndEffector()*transl(-0.2,0.2,-0.1), steps);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
    tr = ur3.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(-pi/2)*drinkOffset);
end

%% give can to dobot
steps = 100;

qGuess = ([-3.5023, -1.0329, 1.6342, -0.6268, 0.8303, 0.0170]);

traj = ur3.getTrajectory(ur3.getEndEffector()*transl(0,-0.15,-0.9)*troty(pi), steps, qGuess);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
    tr = ur3.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(-pi/2)*drinkOffset);
end

steps = 20;

traj = ur3.getTrajectory(ur3.getEndEffector()*transl(0, 0,-0.2), steps, qGuess);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
end

drinks{3,1}.updatePose(drinks{3,1}.getPose()*trotz(pi/2));

%% dobot grab can and put in red bin
steps = 20;
traj = dobot.getTrajectory(drinks{3,1}.getPose()*trotx(pi)*trotz(pi/2)*transl(0,0,-0.13), steps);

for i = 1:steps
    dobot.robot.animate(traj(i,:));
end

%% put in red bin
steps = 50;

traj = dobot.getTrajectory(transl(-0.25,-1.55,0.8)*trotx(pi)*trotz(pi/2), steps);

for i = 1:steps
    dobot.robot.animate(traj(i,:));
    tr = dobot.getEndEffector();
    drinks{3,1}.updatePose(tr*trotx(pi)*transl(0,0,-0.13));
end

%% drink falling in

for i = 1:10;
    drinks{3,1}.updatePose(drinks{3,1}.getPose()*transl(0,0,-0.01*i));
    pause(0.05);
    drawnow();
end

%%
ur3.setLinksAsEllipsoids()
ur3.robot.plot3d(ur3.q0)

sensor = CollisionSensor({objects{4}}, {ur3}, "Magic Collision Sensor");

%%
[result, robot, object] = sensor.getSensorResult()
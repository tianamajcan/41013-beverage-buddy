% main program for the robot project

[drinks, objects] = environmentSetup(figure(1));

% place robots
ur3 = UR3(transl(-0.7, -0.8, 0.8)*trotz(-pi/2));
dobot = Dobot(objects{3}.getPose()*transl(0.15,0,1));

%% get can
steps = 50;
traj = ur3.getTrajectory(drinks{3,4}.getPose()*transl(0,0,0.08)*trotx(pi/2)*troty(-pi/2), steps, ur3.q0);

for i = 1:steps
    ur3.robot.animate(traj(i,:));
end

% sort?
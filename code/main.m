% main program for the robot project

[drinks, objects] = environmentSetup(figure(1));

% place robots
ur3 = UR3(transl(-0.7, -0.8, 0.8)*trotz(-pi/2));
dobot = Dobot(objects{3}.getPose()*transl(0.15,0,1));

% grab a can

% sort?
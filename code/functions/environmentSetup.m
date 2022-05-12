function [drinks, objects] = environmentSetup(figureHandle)
% ENVIRONMENTSETUP Performs the setup of the simulation environment, pass
% the desired figure handle to the function to ensure the environment is
% plotted in the same figure as robots

figureHandle
hold on;
axis equal;
camlight RIGHT;
camlight LEFT;
zlim([0 3]);
xlim([-2 2]);
ylim([-4 4])
view([3 -2 1.5]);

% add floor
surf([-2,-2;2,2],[-4,4;-4,4],[0.01,0.01;0.01,0.01],'CData',imread('grass.jpg'),'FaceColor','texturemap');

% add walls
surf([-1.99,-1.99;-1.99,-1.99],[-4,4;-4,4], [3,3;0,0],'CData',imread('fence.jpg'),'FaceColor','texturemap');
surf([-2,2;-2,2], [3.99,3.99;3.99,3.99], [0,0;3,3],'CData',imread('brick.jpg'),'FaceColor','texturemap');

% add table
table = MeshInterface('environment_assets/table.ply', se3(se2(-1, 0, pi/2)));

% add chair
chair = MeshInterface('chair.ply', se3(se2(0.25, 0.5, -pi/2)));

%add rubbish bin
redbin = MeshInterface('redbin_rubbish.ply');

%add recycling bin


% add fridge
fridge = MeshInterface('fridge.ply', transl(-1.2, -1.4, 0));

% add some drinks in the fridge

% preallocate cell for storing drink handles
drinks = cell(3,4);

% get position and rotation matrix of fridge shelf
[R, T] = tr2rt(fridge.getPose());
shelf = [T(1:2)', T(3)+0.7];

% put drinks in the fridge, line up in y direction
for i = 1:3
    drinks{i, 1} = Drink('fanta', transl((shelf(1)-0.2)+(0.1*i), shelf(2)-0.1, shelf(3)));
    drinks{i, 2} = Drink('sprite', transl((shelf(1)-0.2)+(0.1*i), shelf(2), shelf(3)));
    drinks{i, 3} = Drink('coke', transl((shelf(1)-0.2)+(0.1*i), shelf(2)+0.1, shelf(3)));
    drinks{i, 4} = Drink('beer', transl((shelf(1)-0.2)+(0.1*i), shelf(2)+0.2, shelf(3)));
end


objects = {table, chair, fridge, redbin};

end


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
chair = MeshInterface('chair.ply', se3(se2(0.25, 0.75, -pi/2)));

% add coaster (where the drink must be placed)
coaster = MeshInterface('coaster.ply', transl(-0.75,0.1,0.78));

%add rubbish bin
redbin = MeshInterface('redbin_rubbish.ply', transl(-0.4,-1.5,0));

%add recycling bin
yellowbin = MeshInterface('yellowbin_recycling.ply', transl(-1,-1.5,0));

% add fridge
fridgeBase = MeshInterface('fridge_base.ply', transl(-1.3, -0.6, 0.78)*trotz(pi/2));
% fridgeDoor = MeshInterface('fridge_door.ply', transl(-0.85, -1.75, 0)*trotz(pi/2));

% add some drinks in the fridge

% preallocate cell for storing drink handles
drinks = cell(3,4);

% get position and rotation matrix of fridge shelf
[R, T] = tr2rt(fridgeBase.getPose());
shelf = [T(1:2)', T(3)+0.37];

% put drinks in the fridge, line up in y direction
for i = 1:3
    drinks{i, 1} = Drink('fanta', transl((shelf(1)-0.2)+(0.1*i), shelf(2)-0.1, shelf(3)));
    drinks{i, 2} = Drink('sprite', transl((shelf(1)-0.2)+(0.1*i), shelf(2), shelf(3)));
    drinks{i, 3} = Drink('coke', transl((shelf(1)-0.2)+(0.1*i), shelf(2)+0.1, shelf(3)));
    drinks{i, 4} = Drink('beer', transl((shelf(1)-0.2)+(0.1*i), shelf(2)+0.2, shelf(3)));
end


objects = {table, chair, fridgeBase, coaster, yellowbin, redbin};

end


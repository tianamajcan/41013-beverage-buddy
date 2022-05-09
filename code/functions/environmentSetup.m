function objects = environmentSetup(figureHandle)
%ENVIRONMENTSETUP Performs the setup of the simulation environment, pass
%the desired figure handle to the function to ensure the environment is
%plotted in the same figure as robots

figureHandle
hold on;
axis equal;
camlight RIGHT;
camlight LEFT;
zlim([0 3]);
xlim([-2 2]);
ylim([-4 4])
view([3 -2 1.5]);

%add floor
surf([-2,-2;2,2],[-4,4;-4,4],[0.01,0.01;0.01,0.01],'CData',imread('grass.jpg'),'FaceColor','texturemap');

%add walls
surf([-1.99,-1.99;-1.99,-1.99],[-4,4;-4,4], [3,3;0,0],'CData',imread('fence.jpg'),'FaceColor','texturemap');
surf([-2,2;-2,2], [3.99,3.99;3.99,3.99], [0,0;3,3],'CData',imread('brick.jpg'),'FaceColor','texturemap');

%add table
table = MeshInterface('environment_assets/table.ply', se3(se2(-1, 0, pi/2)));

%add some cans
fanta = Drinks('can', transl(-1, 0, 0.8));
sprite = MeshInterface('environment_assets/sprite.jpg', transl(-1, 0.2, 0.8))
%add chair

%add fridge


objects = {table, fanta};
end


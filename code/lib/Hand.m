classdef Hand < RobotInterface
    %HAND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function self = Hand()
            self.robot = SerialLink(Link('d', 0,'a',0.5,'alpha',0,'qlim', 0), 'name', 'hand', 'base', transl(-0.7, 0.8, 1));
            % get 3d model data from ply file
            [faceData, vertexData] = plyread('the_hand.ply','tri');

            self.robot.faces = {faceData,[]};
            self.robot.points = {vertexData*rotz(-pi/2),[]};
            self.robot.plot3d(0);
        end

    end
end


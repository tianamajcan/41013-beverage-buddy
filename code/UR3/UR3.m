classdef UR3 < RobotInterface
    %UR3 Class for the UR3 robot
    
    properties
        
    end
    
    methods
        function self = UR3(base, name)
            %UR3 Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                base (4,4) {mustBeNumeric} = se3(se2(0, 0, 0));
                name {mustBeText} = 'UR3';
            end

            limits = [ -2*pi 2*pi];

            L1 = Link('d', 0.1519,'a',0,'alpha',pi/2,'qlim', limits);
            L2 = Link('d', 0,'a',-0.24355,'alpha',0,'qlim', limits);
            L3 = Link('d', 0,'a',-0.2132,'alpha',0,'qlim', [deg2rad(-170),deg2rad(170)]); %helps prevent random twisting and self collision
            L4 = Link('d', 0.11235,'a',0 ,'alpha',pi/2,'qlim', limits);
            L5 = Link('d', 0.08535,'a',0,'alpha',-pi/2,'qlim', limits);
            L6 = Link('d', 0.0819,'a',0,'alpha',0, 'qlim', limits); %no limits on last joint
            
            % TODO: change default plot3dopts to have the path to the 3d
            % model of dobot
            self.robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name, 'base', base);            
        end
    end
end



classdef Dobot < RobotInterface
    % DOBOT Class for the Dobot robot
    
    properties
        
    end
    
    methods
        function self = Dobot(base, name)
            %DOBOT Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                base (4,4) {mustBeNumeric} = se3(se2(0, 0, 0));
                name {mustBeText} = 'Dobot';
            end

            % using suggested joint limits from Subject Resources
            L1 = Link('d',0.082,'a', 0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L2 = Link('d',0,'a',0.135,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(5),deg2rad(85)]);
            L3 = Link('d',0,'a',0.147,'alpha',0,'offset',pi/4,'qlim',[deg2rad(-10),deg2rad(90)]);
            L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L5 = Link('d',0.05,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-85),deg2rad(85)]);
            
            % TODO: change default plot3dopts to have the path to the 3d
            % model of dobot
            self.robot = SerialLink([L1 L2 L3 L4 L5], 'name', name, 'base', base);
            
        end
    end
end


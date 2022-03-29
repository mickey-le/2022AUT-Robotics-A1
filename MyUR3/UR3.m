classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.326 2];
        
        %> Flag to indicate if gripper is used
%         useGripper = false;
    end
    
    methods%% Class for UR3 robot simulation
        %         function self = UR3(useGripper)
        function self = UR3()
            %             if nargin < 1
            %                 useGripper = false;
            %             end
            %             self.useGripper = useGripper;
            
            %> Define the boundaries of the workspace
            
            
            % robot =
            self.GetUR3Robot();
            % robot =
            % self.PlotAndColourRobot();%robot,workspace);
        end
        
        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self)
            %     if nargin < 1
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %     end
            
            L1 = Link('theta',pi,'a',0,'alpha',pi/2,'offset',0,'qlim',[-0.8,0]); % Prismatic link
            L2 = Link('d',0.1519,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
            L3 = Link('d',0,'a',-0.24365,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-180),deg2rad(180)]);
            L4 = Link('d',0,'a',-0.21325,'alpha',0,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
            L5 = Link('d',0.11235,'a',0,'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-180),deg2rad(180)]);
            L6 = Link('d',0.08535,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
            L7 = Link('d',0.08190,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
            
            self.model.base = self.model.base * transl(0,0,0) * rpy2tr(pi/2,0,0);
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                %                 if self.useGripper && linkIndex == self.model.n
                %                     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                %                 else
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                %                 end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end
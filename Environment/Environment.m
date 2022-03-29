classdef Environment < handle
    % Private variables for Environment class
    properties (Access = private)
        pose;
        poseUpdate;
        vertexCount;
        enviMesh;
        enviVertices;
    end
    % Public functions for usage
    methods
        % Class constructor
        function self = Environment()
            enviPose = transl(0,-0.1,0.62) * rpy2tr(0,0,0);
            self.SetEnviPose(enviPose);
        end
        % Setter for environment pose
        function SetEnviPose(self,enviPose)
            self.pose = enviPose;
        end
        % Getter for environment pose
        function enviPose = GetEnviPose(self)
            enviPose = self.pose;
        end
        % Function for plotting the environment
        function PlotEnvironment(self)
            surf([-3,-3;3,3],[-2.5,2;-2.5,2],[-0.316,-0.306;-0.316,-0.306],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            hold on;
            [face,vertices,data] = plyread('FullEnvi.ply','tri');
            self.vertexCount = size(vertices,1);
            midPoint = sum(vertices)/self.vertexCount;
            self.enviVertices = vertices - repmat(midPoint,self.vertexCount,1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.enviMesh = trisurf(face, self.enviVertices(:,1), self.enviVertices(:,2), self.enviVertices(:,3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
            self.poseUpdate = [self.pose * [self.enviVertices,ones(self.vertexCount,1)]']';
            self.enviMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
    end
end
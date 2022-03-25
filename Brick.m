classdef Brick < handle
    % Brick dimensions: L = 2*|x| = 0.266, W = 2*|y| = 0.132, H = |-0.04| + 0.027 = 0.067
    properties (Constant)
        length = 0.266;
        width = 0.132;
        height = 0.067;
    end
    properties
        pose;
        poseUpdate;
        
        vertexCount;
        brickMesh;
        brickVertices;
    end
    
    methods
        function self = Brick(brickPose) 
            
            self.SetBrickPose(brickPose);
        end
        
        function SetBrickPose(self,brickPose)
            self.pose = brickPose;
        end
        
        function brickPose = GetBrickPose(self)
            brickPose = self.pose;
        end
        
        function PlotBrickModel(self)
            
            [face,vertices,data] = plyread('Brick.ply','tri');
            
            self.vertexCount = size(vertices,1);
            
            midPoint = sum(vertices)/self.vertexCount;
            
            self.brickVertices = vertices - repmat(midPoint,self.vertexCount,1);
            
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.brickMesh = trisurf(face, self.brickVertices(:,1), self.brickVertices(:,2), self.brickVertices(:,3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting','flat');
            
            self.poseUpdate = [self.pose * [self.brickVertices,ones(self.vertexCount,1)]']';
            
            self.brickMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
        
        function MoveBrick(self,goalPose)
%             self.pose = self.pose * goalPose;
            self.SetBrickPose(goalPose);
            self.poseUpdate = [self.pose * [self.brickVertices,ones(self.vertexCount,1)]']';
            self.brickMesh.Vertices = self.poseUpdate(:,1:3);
            drawnow();
        end
    end
end
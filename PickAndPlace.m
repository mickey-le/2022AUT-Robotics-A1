% Custom class for Pick and Place operations
classdef PickAndPlace < handle
    methods (Static)
        % Move a robot to pick brick and update its q after pick
        function qAfterPick = OneRobotPickBrick(robot,qCurr,brick,steps)
            % Acquire pick up position
            posRdyBrick = brick.GetBrickPose() * transl(0,0,-0.07);
            posBrick = brick.GetBrickPose() * transl(0,0,-0.035);
            % Compute pick up q
            qRdyBrick = robot.model.ikcon(posRdyBrick,qCurr);
            qBrick = robot.model.ikcon(posBrick,qRdyBrick);
            % Pick up brick
            MoveIt.MoveOneRobot(robot,qCurr,qRdyBrick,steps);
            MoveIt.MoveOneRobot(robot,qRdyBrick,qBrick,steps);
            MoveIt.MoveOneRobotAndBrick(robot,qBrick,qRdyBrick,steps,brick);
            % Update q
            qAfterPick = qRdyBrick;
        end
        
        % Move a robot to place brick and update its q after pick
        function qAfterPlace = OneRobotPlaceBrick(robot,qCurr,grabBrick,targetBrick,steps)
            % Acquire place down position
            posRdyTargetBrick = targetBrick.GetBrickPose() * transl(0,0,-0.07 - Brick.height);
            posTargetBrick = targetBrick.GetBrickPose() * transl(0,0,-0.035 - Brick.height);
            % Compute place down q
            qRdyTargetBrick = robot.model.ikcon(posRdyTargetBrick,qCurr);
            qTargetBrick = robot.model.ikcon(posTargetBrick,qRdyTargetBrick);
            % Place down q
            MoveIt.MoveOneRobotAndBrick(robot,qCurr,qRdyTargetBrick,steps,grabBrick);
            MoveIt.MoveOneRobotAndBrick(robot,qRdyTargetBrick,qTargetBrick,steps,grabBrick);
            MoveIt.MoveOneRobot(robot,qTargetBrick,qRdyTargetBrick,steps);
            % Update q
            qAfterPlace = qRdyTargetBrick;
        end
        
        function [qAfterPick1,qAfterPick2] = TwoRobotPickBrick(robot1,qCurr1,brick1,robot2,qCurr2,brick2,steps)
            posRdyBrick1 = brick1.GetBrickPose() * transl(0,0,-0.07);
            posRdyBrick2 = brick2.GetBrickPose() * transl(0,0,-0.07);
            posBrick1 = brick1.GetBrickPose() * transl(0,0,-0.035);
            posBrick2 = brick2.GetBrickPose() * transl(0,0,-0.035);
            
            qRdyBrick1 = robot1.model.ikcon(posRdyBrick1,qCurr1);
            qRdyBrick2 = robot2.model.ikcon(posRdyBrick2,qCurr2);
            
            qBrick1 = robot1.model.ikcon(posBrick1,qRdyBrick1);
            qBrick2 = robot2.model.ikcon(posBrick2,qRdyBrick2);
            
            MoveIt.MoveTwoRobot(robot1,qCurr1,qRdyBrick1,robot2,qCurr2,qRdyBrick2,steps);
            MoveIt.MoveTwoRobot(robot1,qRdyBrick1,qBrick1,robot2,qRdyBrick2,qBrick2,steps);
            MoveIt.MoveTwoRobotAndTwoBrick(robot1,qBrick1,qRdyBrick1,robot2,qBrick2,qRdyBrick2,steps,brick1,brick2);
            
            qAfterPick1 = qRdyBrick1;
            qAfterPick2 = qRdyBrick2;
        end
        
        function [qAfterPlace1,qAfterPlace2] = TwoRobotPlaceBrick()
        end
        
        function [qAfterPick,qAfterPlace] = RobotsOnePickOnePlace()
        end
    end
end
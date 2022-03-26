% Custom class for move robots and bricks
classdef MoveIt < handle
    properties (Constant)
        brkZOffsetUR3 = 0.032;
        brkZOffsetUR5 = 0.083;
    end
    methods (Static)
        % Move one robot
        function MoveOneRobot(robot,qCurr,qGoal,steps)
            qMatrix = jtraj(qCurr,qGoal,steps);
            for i=1:steps
                robot.model.animate(qMatrix(i,:));
                drawnow();
            end
        end
        
        % Move one robot and brick at the same time
        function MoveUR3AndBrick(robot,qCurr,qGoal,steps,brick)
            qMatrix = jtraj(qCurr,qGoal,steps);
            for i=1:steps
                robot.model.animate(qMatrix(i,:));
                endEffPos = robot.model.fkine(qMatrix(i,:));
                % 'Move' the brick parallel with the end effector pose
                endEffPos(3,4) = endEffPos(3,4) - MoveIt.brkZOffsetUR3;
                brick.MoveBrick(endEffPos);
                drawnow();
            end
        end
        
        % Move one robot and brick at the same time
        function MoveUR5AndBrick(robot,qCurr,qGoal,steps,brick)
            qMatrix = jtraj(qCurr,qGoal,steps);
            for i=1:steps
                robot.model.animate(qMatrix(i,:));
                endEffPos = robot.model.fkine(qMatrix(i,:));
                % 'Move' the brick parallel with the end effector pose
                endEffPos(3,4) = endEffPos(3,4) - MoveIt.brkZOffsetUR5;
                brick.MoveBrick(endEffPos);
                drawnow();
            end
        end
        
        % Move two robots at the same time
        function MoveTwoRobot(robot1,qCurr1,qGoal1,robot2,qCurr2,qGoal2,steps)
            qMatrix1 = jtraj(qCurr1,qGoal1,steps);
            qMatrix2 = jtraj(qCurr2,qGoal2,steps);
            for i=1:steps
                robot1.model.animate(qMatrix1(i,:));
                robot2.model.animate(qMatrix2(i,:));
                drawnow();
            end
        end
        
        % Move two robots and bricks at the same time
        function MoveTwoRobotAndTwoBrick(robot1,qCurr1,qGoal1,robot2,qCurr2,qGoal2,steps,brick1,brick2)
            qMatrix1 = jtraj(qCurr1,qGoal1,steps);
            qMatrix2 = jtraj(qCurr2,qGoal2,steps);
            for i=1:steps
                robot1.model.animate(qMatrix1(i,:));
                robot2.model.animate(qMatrix2(i,:));
                
                endEff1Pos = robot1.model.fkine(qMatrix1(i,:));
                endEff1Pos(3,4) = endEff1Pos(3,4) - MoveIt.brkZOffsetUR3;
                endEff2Pos = robot2.model.fkine(qMatrix2(i,:));
                endEff2Pos(3,4) = endEff2Pos(3,4) - MoveIt.brkZOffsetUR5;
                
                brick1.MoveBrick(endEff1Pos);
                brick2.MoveBrick(endEff2Pos);
                drawnow();
            end
        end
        
        % Move two robots, one with brick and one without brick
        function MoveTwoRobotAndOneBrick(robotFree,qCurrFree,qGoalFree,robotBrick,qCurrBrick,qGoalBrick,steps,brick)
            qMatrixBrick = jtraj(qCurrBrick,qGoalBrick,steps);
            qMatrixFree = jtraj(qCurrFree,qGoalFree,steps);
            for i=1:steps
                robotBrick.model.animate(qMatrixBrick(i,:));
                robotFree.model.animate(qMatrixFree(i,:));
                
                endEffBrickPos = robot.model.fkine(qMatrixBrick(i,:));
                endEffBrickPos(3,4) = endEffBrickPos(3,4) - MoveIt.brkZOffsetUR3;
                
                brick.MoveBrick(endEffBrickPos);
                drawnow();
            end
        end
    end
end
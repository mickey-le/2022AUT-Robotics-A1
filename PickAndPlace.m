% Custom class for Pick and Place operations
classdef PickAndPlace < handle
    properties (Constant)
        rdyOffsetUR3 = -0.07;
        brkOffsetUR3 = -0.032;
        rdyOffsetUR5 = -0.11;
        brkOffsetUR5 = -0.085;
    end
    
    enumeration
        Top, Left, Right, Front, Back;
    end
    
    methods (Static)
        % Move UR3 to pick brick and update its q after pick
        function qAfterPick = UR3PickBrick(robot,qCurr,brick,steps)
            % Acquire pick up position
            posRdyBrick = brick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3);
            posBrick = brick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3);
            % Compute pick up q
            qRdyBrick = robot.model.ikcon(posRdyBrick,qCurr);
            qBrick = robot.model.ikcon(posBrick,qRdyBrick);
            % Pick up brick
            MoveIt.MoveOneRobot(robot,qCurr,qRdyBrick,steps);
            MoveIt.MoveOneRobot(robot,qRdyBrick,qBrick,steps);
            MoveIt.MoveUR3AndBrick(robot,qBrick,qRdyBrick,steps,brick);
            % Update q
            qAfterPick = qRdyBrick;
        end
        
        % Move UR5 to pick brick and update its q after pick
        function qAfterPick = UR5PickBrick(robot,qCurr,brick,steps)
            % Acquire pick up position
            posRdyBrick = brick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR5);
            posBrick = brick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR5);
            % Compute pick up q
            qRdyBrick = robot.model.ikcon(posRdyBrick,qCurr);
            qBrick = robot.model.ikcon(posBrick,qRdyBrick);
            % Pick up brick
            MoveIt.MoveOneRobot(robot,qCurr,qRdyBrick,steps);
            MoveIt.MoveOneRobot(robot,qRdyBrick,qBrick,steps);
            MoveIt.MoveUR5AndBrick(robot,qBrick,qRdyBrick,steps,brick);
            % Update q
            qAfterPick = qRdyBrick;
        end
        
        % Move UR3 to place brick and update its q after pick
        function qAfterPlace = UR3PlaceBrickOnBrick(robot,qCurr,grabBrick,targetBrick,steps)
            % Acquire place down position
            posRdyTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3 - Brick.height);
            posTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3 - Brick.height);
            % Compute place down q
            qRdyTargetBrick = robot.model.ikcon(posRdyTargetBrick,qCurr);
            qTargetBrick = robot.model.ikcon(posTargetBrick,qRdyTargetBrick);
            % Place down brick
            MoveIt.MoveUR3AndBrick(robot,qCurr,qRdyTargetBrick,steps,grabBrick);
            MoveIt.MoveUR3AndBrick(robot,qRdyTargetBrick,qTargetBrick,steps,grabBrick);
            MoveIt.MoveOneRobot(robot,qTargetBrick,qRdyTargetBrick,steps);
            % Update q
            qAfterPlace = qRdyTargetBrick;
        end
        
        % Move UR5 to place brick and update its q after pick
        function qAfterPlace = UR5PlaceBrickOnBrick(robot,qCurr,grabBrick,targetBrick,steps)
            % Acquire place down position
            posRdyTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR5 - Brick.height);
            posTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR5 - Brick.height);
            % Compute place down q
            qRdyTargetBrick = robot.model.ikcon(posRdyTargetBrick,qCurr);
            qTargetBrick = robot.model.ikcon(posTargetBrick,qRdyTargetBrick);
            % Place down brick
            MoveIt.MoveUR5AndBrick(robot,qCurr,qRdyTargetBrick,steps,grabBrick);
            MoveIt.MoveUR5AndBrick(robot,qRdyTargetBrick,qTargetBrick,steps,grabBrick);
            MoveIt.MoveOneRobot(robot,qTargetBrick,qRdyTargetBrick,steps);
            % Update q
            qAfterPlace = qRdyTargetBrick;
        end
        
        function [qAfterPick1,qAfterPick2] = TwoRobotPickBrick(robotUR3,qCurrUR3,brick1,robotUR5,qCurrUR5,brick2,steps)
            % Acquire pick up positions
            posRdyBrick1 = brick1.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3);
            posRdyBrick2 = brick2.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR5);
            posBrick1 = brick1.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3);
            posBrick2 = brick2.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR5);
            % Compute pick up q
            qRdyBrick1 = robotUR3.model.ikcon(posRdyBrick1,qCurrUR3);
            qRdyBrick2 = robotUR5.model.ikcon(posRdyBrick2,qCurrUR5);
            qBrick1 = robotUR3.model.ikcon(posBrick1,qRdyBrick1);
            qBrick2 = robotUR5.model.ikcon(posBrick2,qRdyBrick2);
            % Place down bricks
            MoveIt.MoveTwoRobot(robotUR3,qCurrUR3,qRdyBrick1,robotUR5,qCurrUR5,qRdyBrick2,steps);
            MoveIt.MoveTwoRobot(robotUR3,qRdyBrick1,qBrick1,robotUR5,qRdyBrick2,qBrick2,steps);
            MoveIt.MoveTwoRobotAndTwoBrick(robotUR3,qBrick1,qRdyBrick1,robotUR5,qBrick2,qRdyBrick2,steps,brick1,brick2);
            % Update q
            qAfterPick1 = qRdyBrick1;
            qAfterPick2 = qRdyBrick2;
        end
        
        function [qAfterPlace1,qAfterPlace2] = TwoRobotPlaceBrickOnBrick(robotUR3,qCurrUR3,grabBrickUR3,targetBrick1, ...
                                                                           robotUR5,qCurrUR5,grabBrickUR5,targetBrick2,steps)
            % Acquire place down positions
            posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3 - Brick.height);
            posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR5 - Brick.height);
            posTargetBrick1 = targetBrick1.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3 - Brick.height);
            posTargetBrick2 = targetBrick2.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR5 - Brick.height);
            % Compute place down q
            qRdyTargetBrick1 = robotUR3.model.ikcon(posRdyTargetBrick1,qCurrUR3);
            qRdyTargetBrick2 = robotUR5.model.ikcon(posRdyTargetBrick2,qCurrUR5);
            qTargetBrick1 = robotUR3.model.ikcon(posTargetBrick1,qRdyTargetBrick1);
            qTargetBrick2 = robotUR5.model.ikcon(posTargetBrick2,qRdyTargetBrick2);
            % Place down bricks
            MoveIt.MoveTwoRobotAndTwoBrick(robotUR3,qCurrUR3,qRdyTargetBrick1,robotUR5,qCurrUR5,qRdyTargetBrick2,steps,grabBrickUR3,grabBrickUR5);
            MoveIt.MoveTwoRobotAndTwoBrick(robotUR3,qRdyTargetBrick1,qTargetBrick1,robotUR5,qRdyTargetBrick2,qTargetBrick2,steps,grabBrickUR3,grabBrickUR5);
            MoveIt.MoveTwoRobot(robotUR3,qTargetBrick1,qRdyTargetBrick1,robotUR5,qTargetBrick2,qRdyTargetBrick2,steps);
            % Update q
            qAfterPlace1 = qRdyTargetBrick1;
            qAfterPlace2 = qRdyTargetBrick2;
        end
        
        function [qAfterPick,qAfterPlace] = TwoRobotOnePickOnePlaceOnBrick(robotPick,qCurrPick,brickPick,robotPlace,qCurrPlace,grabBrick,targetBrick,steps)
            % Acquire pick up and place down positions
            posRdyPickBrick = brickPick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3);
            posPickBrick = brickPick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3);
            posRdyTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.rdyOffsetUR3 - Brick.height);
            posTargetBrick = targetBrick.GetBrickPose() * transl(0,0,PickAndPlace.brkOffsetUR3 - Brick.height);
            % Compute pick up and place down q
            qRdyPickBrick = robotPick.model.ikcon(posRdyPickBrick,qCurrPick);
            qRdyTargetBrick = robotPlace.model.ikcon(posRdyTargetBrick,qCurrPlace);
            qPickBrick = robotPick.model.ikcon(posPickBrick,qRdyPickBrick);
            qTargetBrick = robotPlace.model.ikcon(posTargetBrick,qRdyTargetBrick);
            % Pick up and place down bricks
            % Status: Pick Robot (Free), Place Robot (Brick)
            MoveIt.MoveTwoRobotAndOneBrick(robotPick,qCurrPick,qRdyPickBrick,robotPlace,qCurrPlace,qRdyTargetBrick,steps,grabBrick);
            MoveIt.MoveTwoRobotAndOneBrick(robotPick,qRdyPickBrick,qPickBrick,robotPlace,qRdyTargetBrick,qTargetBrick,steps,grabBrick);
            % Status: Pick Robot (Brick), Place Robot (Free)
            MoveIt.MoveTwoRobotAndOneBrick(robotPlace,qTargetBrick,qRdyTargetBrick,robotPick,qPickBrick,qRdyPickBrick,steps,brickPick);
            % Update q
            qAfterPick = qRdyPickBrick;
            qAfterPlace = qRdyTargetBrick;
        end
        
        function [qAfterPlace1,qAfterPlace2] = TwoRobotPlaceBrick(robotUR3,qCurrUR3,grabBrickUR3,targetBrick1,placement1, ...
                                                                    robotUR5,qCurrUR5,grabBrickUR5,targetBrick2,placement2,steps)
            % Placement options for both robots.
            % Compute placement position for Robot 1
            if (placement1 == PickAndPlace.Top)
                posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, 0, PickAndPlace.rdyOffsetUR3 - Brick.height);
                posTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, 0, PickAndPlace.brkOffsetUR3 - Brick.height);
            elseif (placement1 == PickAndPlace.Left)
                posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(-Brick.length, 0, PickAndPlace.rdyOffsetUR3 - Brick.height);
                posTargetBrick1 = targetBrick1.GetBrickPose() * transl(-Brick.length, 0, PickAndPlace.brkOffsetUR3);
            elseif (placement1 == PickAndPlace.Right)
                posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(Brick.length, 0, PickAndPlace.rdyOffsetUR3 - Brick.height);
                posTargetBrick1 = targetBrick1.GetBrickPose() * transl(Brick.length, 0, PickAndPlace.brkOffsetUR3);
            elseif (placement1 == PickAndPlace.Front)
                posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, Brick.width, PickAndPlace.rdyOffsetUR3 - Brick.height);
                posTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, Brick.width, PickAndPlace.brkOffsetUR3);
            elseif (placement1 == PickAndPlace.Back)
                posRdyTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, -Brick.width, PickAndPlace.rdyOffsetUR3 - Brick.height);
                posTargetBrick1 = targetBrick1.GetBrickPose() * transl(0, -Brick.width, PickAndPlace.brkOffsetUR3);
            else
            end
            % Compute placement position for Robot 2
            if (placement2 == PickAndPlace.Top)
                posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, 0, PickAndPlace.rdyOffsetUR5 - Brick.height);
                posTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, 0, PickAndPlace.brkOffsetUR5 - Brick.height);
            elseif (placement2 == PickAndPlace.Left)
                posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(-Brick.length, 0, PickAndPlace.rdyOffsetUR5 - Brick.height);
                posTargetBrick2 = targetBrick2.GetBrickPose() * transl(-Brick.length, 0, PickAndPlace.brkOffsetUR5);
            elseif (placement2 == PickAndPlace.Right)
                posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(Brick.length, 0, PickAndPlace.rdyOffsetUR5 - Brick.height);
                posTargetBrick2 = targetBrick2.GetBrickPose() * transl(Brick.length, 0, PickAndPlace.brkOffsetUR5);
            elseif (placement2 == PickAndPlace.Front)
                posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, Brick.width, PickAndPlace.rdyOffsetUR5 - Brick.height);
                posTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, Brick.width, PickAndPlace.brkOffsetUR5);
            elseif (placement2 == PickAndPlace.Back)
                posRdyTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, -Brick.width, PickAndPlace.rdyOffsetUR5 - Brick.height);
                posTargetBrick2 = targetBrick2.GetBrickPose() * transl(0, -Brick.width, PickAndPlace.brkOffsetUR5);
            else
            end
            
            qRdyTargetBrick1 = robotUR3.model.ikcon(posRdyTargetBrick1,qCurrUR3);
            qRdyTargetBrick2 = robotUR5.model.ikcon(posRdyTargetBrick2,qCurrUR5);
            qTargetBrick1 = robotUR3.model.ikcon(posTargetBrick1,qRdyTargetBrick1);
            qTargetBrick2 = robotUR5.model.ikcon(posTargetBrick2,qRdyTargetBrick2);
            % Place down bricks
            MoveIt.MoveTwoRobotAndTwoBrick(robotUR3,qCurrUR3,qRdyTargetBrick1, ...
                                            robotUR5,qCurrUR5,qRdyTargetBrick2,steps,grabBrickUR3,grabBrickUR5);
            MoveIt.MoveTwoRobotAndTwoBrick(robotUR3,qRdyTargetBrick1,qTargetBrick1, ...
                                            robotUR5,qRdyTargetBrick2,qTargetBrick2,steps,grabBrickUR3,grabBrickUR5);
            MoveIt.MoveTwoRobot(robotUR3,qTargetBrick1,qRdyTargetBrick1, ...
                                robotUR5,qTargetBrick2,qRdyTargetBrick2,steps);
            % Update q
            qAfterPlace1 = qRdyTargetBrick1;
            qAfterPlace2 = qRdyTargetBrick2;
        end
    end
end
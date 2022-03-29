clear all; clf; clc;
addpath 'Brick'
addpath 'LinearUR3'
addpath 'MyUR3'
addpath 'Environment'
hold on;
%% Generate and plot the Environment
Envi = Environment();
Envi.PlotEnvironment();

%% Initialise UR3 model and its base position 
myUR3 = UR3();
myUR3.model.base = transl(0.65, -0.3, 0.34) * rpy2tr(pi/2, -pi/2, 0);
myUR3.PlotAndColourRobot();

%% Initialise UR5 Linear model and its base position
linearUR5 = LinearUR5();
linearUR5.model.base = transl(-0.35, 0.3, 0.34) * rpy2tr(pi/2, pi/2, 0);
linearUR5.PlotAndColourRobot();

%% Initialise the bricks and its position
% Determine brick poses
Brick1 = Brick(transl(0.2, 0, 0.37) * rpy2tr(pi, 0, -pi/4)); % Center

Brick2 = Brick(transl(-0.2, -0.15, 0.37) * rpy2tr(pi, 0, -pi/4)); 
Brick3 = Brick(transl(-0.5, -0.2, 0.37 + Brick.height) * rpy2tr(pi, 0, 0)); 
Brick4 = Brick(transl(-1, -0.2, 0.37) * rpy2tr(pi, 0, pi/4)); 

Brick5 = Brick(transl(-0.3, -0.5, 0.37) * rpy2tr(pi, 0, 0)); 
Brick6 = Brick(transl(-0.7, -0.2, 0.37) * rpy2tr(pi, 0, pi/2)); 
Brick7 = Brick(transl(-0.9, -0.5, 0.37) * rpy2tr(pi, 0, 0)); 

Brick8 = Brick(transl(-0.5, -0.2, 0.37) * rpy2tr(pi, 0, pi/2));
Brick9 = Brick(transl(-1.2, -0.3, 0.37) * rpy2tr(pi, 0, pi/2)); 

% Plot bricks
Brick1.PlotBrickModel();
Brick2.PlotBrickModel();
Brick3.PlotBrickModel();
Brick4.PlotBrickModel();
Brick5.PlotBrickModel();
Brick6.PlotBrickModel();
Brick7.PlotBrickModel();
Brick8.PlotBrickModel();
Brick9.PlotBrickModel();

%% Set goal
Goal = Brick(transl(0.7, 0.25, 0.37) * rpy2tr(pi, 0, 0));
Checkpoint = Brick(transl(0.2, 0, 0.37) * rpy2tr(pi, 0, pi/2));

%% 
input('Press Enter to start');

% Steps number
steps = 75;

% Initial q and Ready q
% UR3:
qInitUR3 = myUR3.model.getpos();
qHomeUR3 = deg2rad([0 0 -22 -90 22 90 180]);

% UR5:
qInitUR5 = linearUR5.model.getpos();
qHomeUR5 = deg2rad([0 0 18 -90 -18 90 0]);

% Move robot to Home position
MoveIt.MoveTwoRobot(myUR3,qInitUR3,qHomeUR3,linearUR5,qInitUR5,qHomeUR5,steps);

% Set current robot q
qCurrUR3 = qHomeUR3;
qCurrUR5 = qHomeUR5;

%% Test UR3 sequence
% %% Pick up Brick 6 and update current q
% qCurrUR3 = PickAndPlace.UR3PickBrick(myUR3,qCurrUR3,Brick6,steps);
% 
% %% Place Brick 6 onto Brick 2 and update current q
% qCurrUR3 = PickAndPlace.UR3PlaceBrick(myUR3,qCurrUR3,Brick6,Brick2,steps);
% 
% %% Pick up Brick 9 and update current q
% qCurrUR3 = PickAndPlace.UR3PickBrick(myUR3,qCurrUR3,Brick9,steps);
% 
% %% Place Brick 9 onto Brick 6 and update current q
% qCurrUR3 = PickAndPlace.UR3PlaceBrickOnBrick(myUR3,qCurrUR3,Brick9,Brick6,steps);

%% Official Build Wall Sequence

%% Build Wall Sequence
% UR3 pick Brick 2, UR5 pick Brick 4
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick2,linearUR5,qCurrUR5,Brick4,steps);

% UR3 place Brick 2 on Brick 1, UR5 place Brick 4 next to Brick 1
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick2,Brick1,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick4,Brick1,PickAndPlace.Left,steps);

% UR3 pick Brick 9, UR5 pick Brick 2
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick9,linearUR5,qCurrUR5,Brick2,steps);

% UR3 place Brick 9 on Brick 1, UR5 place Brick 2 on Brick 4
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick9,Brick1,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick2,Brick4,PickAndPlace.Top,steps);

% Adjust UR3, Adjust UR5
qMidwayUR3 = qHomeUR3;
qMidwayUR5 = qHomeUR5;
qMidwayUR3(1,1) = qCurrUR3(1,1);
qMidwayUR5(1,1) = qCurrUR5(1,1);
MoveIt.MoveTwoRobot(myUR3,qCurrUR3,qMidwayUR3,linearUR5,qCurrUR5,qMidwayUR5,steps);
qCurrUR3 = qMidwayUR3;
qCurrUR5 = qMidwayUR5;

% UR3 pick Brick 6, UR5 pick Brick 5
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick6,linearUR5,qCurrUR5,Brick5,steps);

% Adjust UR3, Adjust UR5
qMidwayUR3 = qHomeUR3;
qMidwayUR5 = qHomeUR5;
qMidwayUR3(1,1) = qCurrUR3(1,1);
qMidwayUR5(1,1) = qCurrUR5(1,1);
MoveIt.MoveTwoRobotAndTwoBrick(myUR3,qCurrUR3,qMidwayUR3,linearUR5,qCurrUR5,qMidwayUR5,steps,Brick6,Brick5);
qCurrUR3 = qMidwayUR3;
qCurrUR5 = qMidwayUR5;

% UR3 place Brick 6 on Brick 9, UR5 place Brick 5 next to Brick 1
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick6,Brick9,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick5,Brick1,PickAndPlace.Right,steps);

% Move UR3 to Init, Move UR5 to Home
MoveIt.MoveTwoRobot(myUR3,qCurrUR3,qInitUR3,linearUR5,qCurrUR5,qHomeUR5,steps);
qCurrUR5 = qHomeUR5;

% UR5 pick Brick 8
qCurrUR5 = PickAndPlace.UR5PickBrick(linearUR5,qCurrUR5,Brick8,steps);

% UR5 place Brick 8 on Brick 5
qCurrUR5 = PickAndPlace.UR5PlaceBrickOnBrick(linearUR5,qCurrUR5,Brick8,Brick5,steps);

% UR5 pick Brick 3
qCurrUR5 = PickAndPlace.UR5PickBrick(linearUR5,qCurrUR5,Brick3,steps);

% Adjust UR5
qMidwayUR5 = qHomeUR5;
qMidwayUR5(1,1) = qCurrUR5(1,1);
MoveIt.MoveUR5AndBrick(linearUR5,qCurrUR5,qMidwayUR5,steps,Brick3);
qCurrUR5 = qMidwayUR5;

% UR5 place Brick 3 on Brick 8
qCurrUR5 = PickAndPlace.UR5PlaceBrickOnBrick(linearUR5,qCurrUR5,Brick3,Brick8,steps);

% UR5 pick Brick 7
qCurrUR5 = PickAndPlace.UR5PickBrick(linearUR5,qCurrUR5,Brick7,steps);

% UR5 place Brick 7 on Brick 2
qCurrUR5 = PickAndPlace.UR5PlaceBrickOnBrick(linearUR5,qCurrUR5,Brick7,Brick2,steps);
MoveIt.MoveOneRobot(linearUR5,qCurrUR5,qInitUR5,steps);

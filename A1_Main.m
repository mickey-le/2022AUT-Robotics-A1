%% Initialise
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
myUR3.model.base = transl(0.65, -0.3, 0.345) * rpy2tr(pi/2, -pi/2, 0);
myUR3.PlotAndColourRobot();

%% Initialise UR5 Linear model and its base position
linearUR5 = LinearUR5();
linearUR5.model.base = transl(-0.35, 0.3, 0.345) * rpy2tr(pi/2, pi/2, 0);
linearUR5.PlotAndColourRobot();

%% Initialise the bricks and its position
% Determine brick poses
Brick1 = Brick(transl(0.3, 0.1, 0.37) * rpy2tr(pi, 0, pi)); % Center

Brick2 = Brick(transl(-0.15, -0.35, 0.37) * rpy2tr(pi, 0, pi)); 
Brick3 = Brick(transl(-0.6, -0.2, 0.37 + Brick.height) * rpy2tr(pi, 0, pi)); 
Brick4 = Brick(transl(-0.6, -0.4, 0.37) * rpy2tr(pi, 0, pi)); 

Brick5 = Brick(transl(-0.3, -0.5, 0.37) * rpy2tr(pi, 0, pi)); 
Brick6 = Brick(transl(-0.7, -0.2, 0.37) * rpy2tr(pi, 0, pi)); 
Brick7 = Brick(transl(-1, -0.5, 0.37) * rpy2tr(pi, 0, pi)); 

Brick8 = Brick(transl(-0.4, -0.2, 0.37) * rpy2tr(pi, 0, pi));
Brick9 = Brick(transl(-1.1, -0.3, 0.37) * rpy2tr(pi, 0, pi)); 

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
Goal = Brick(transl(0.9, 0.07, 0.37 - Brick.height) * rpy2tr(pi, 0, pi));
Checkpoint = Brick(transl(0.3, 0.1, 0.37 - Brick.height) * rpy2tr(pi, 0, pi));

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

%% Build Wall Sequence
% UR3 pick Brick 1, UR5 pick Brick 2
disp('UR3 pick 1, UR5 pick 2');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick1,linearUR5,qCurrUR5,Brick2,steps);

% UR3 place Brick 1 on Goal, UR5 place Brick 2 on Checkpoint
disp('UR3 place 1 at goal, UR5 place 2 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick1,Goal,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick2,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 2, UR5 pick Brick 3
disp('UR3 pick 2, UR5 pick 3');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick2,linearUR5,qCurrUR5,Brick3,steps);

% UR3 place Brick 2 on right side of Brick 1, UR5 place Brick 3 on Checkpoint
disp('UR3 place 2 right side 1, UR5 place 3 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick2,Brick1,PickAndPlace.Right ...
    ,linearUR5,qCurrUR5,Brick3,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 3, UR5 pick Brick 4
disp('UR3 pick 3, UR5 pick 4');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick3,linearUR5,qCurrUR5,Brick4,steps);

% UR3 place Brick 3 on left side of Brick 1, UR5 place Brick 4 on Checkpoint
disp('UR3 place 3 left side 1, UR5 place 4 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick3,Brick1,PickAndPlace.Left ...
    ,linearUR5,qCurrUR5,Brick4,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 4, UR5 pick Brick 5
disp('UR3 pick 4, UR5 pick 5');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick4,linearUR5,qCurrUR5,Brick5,steps);

% UR3 place Brick 4 on top of Brick 3, UR5 place Brick 5 on Checkpoint
disp('UR3 place 4 on 3, UR5 place 5 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick4,Brick3,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick5,Checkpoint,PickAndPlace.Top,steps);

% Adjust UR3, Adjust UR5
qMidwayUR3 = qHomeUR3;
qMidwayUR5 = qHomeUR5;
MoveIt.MoveTwoRobot(myUR3,qCurrUR3,qMidwayUR3,linearUR5,qCurrUR5,qMidwayUR5,steps);
qCurrUR3 = qMidwayUR3;
qCurrUR5 = qMidwayUR5;

% UR3 pick Brick 5, UR5 pick Brick 6
disp('UR3 pick 5, UR5 pick 6');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick5,linearUR5,qCurrUR5,Brick6,steps);

% UR3 place Brick 5 on top of Brick 1, UR5 place Brick 6 on Checkpoint
disp('UR3 place 5 on 1, UR5 place 6 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick5,Brick1,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick6,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 6, UR5 pick Brick 7
disp('UR3 pick 6, UR5 pick 7');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick6,linearUR5,qCurrUR5,Brick7,steps);

% UR3 place Brick 6 on top of Brick 2, UR5 place Brick 7 on Checkpoint
disp('UR3 place 6 on 2, UR5 place 7 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick6,Brick2,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick7,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 7, UR5 pick Brick 8
disp('UR3 pick 7, UR5 pick 8');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick7,linearUR5,qCurrUR5,Brick8,steps);

% UR3 place Brick 7 on top of Brick 4, UR5 place Brick 8 on Checkpoint
disp('UR3 place 7 on 4, UR5 place 8 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick7,Brick4,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick8,Checkpoint,PickAndPlace.Top,steps);

% UR3 pick Brick 8, UR5 pick Brick 9
disp('UR3 pick 8, UR5 pick 9');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPickBrick(myUR3,qCurrUR3,Brick8,linearUR5,qCurrUR5,Brick9,steps);

% UR3 place Brick 8 on top of Brick 5, UR5 place Brick 9 on Checkpoint
disp('UR3 place 8 on 5, UR5 place 9 at checkpoint');
[qCurrUR3,qCurrUR5] = PickAndPlace.TwoRobotPlaceBrick(myUR3,qCurrUR3,Brick8,Brick5,PickAndPlace.Top ...
    ,linearUR5,qCurrUR5,Brick9,Checkpoint,PickAndPlace.Top,steps);

% Adjust UR3, UR5 return init state
disp('UR5 return initial state');
qMidwayUR3 = qHomeUR3;
MoveIt.MoveTwoRobot(myUR3,qCurrUR3,qMidwayUR3,linearUR5,qCurrUR5,qInitUR5,steps);
qCurrUR3 = qMidwayUR3;

% UR3 pick Brick 9
disp('UR3 pick 9');
qCurrUR3 = PickAndPlace.UR3PickBrick(myUR3,qCurrUR3,Brick9,steps);

% UR3 place Brick 9 on Brick 6
disp('UR3 place 9 on 6');
qCurrUR3 = PickAndPlace.UR3PlaceBrickOnBrick(myUR3,qCurrUR3,Brick9,Brick6,steps);

% UR3 
disp('UR3 return initial state');
MoveIt.MoveOneRobot(myUR3,qCurrUR3,qInitUR3,steps);

disp('Done wall building');

%% Code for adjusting robot to home state
% qMidwayUR3 = qHomeUR3;
% qMidwayUR5 = qHomeUR5;
% MoveIt.MoveTwoRobotAndTwoBrick(myUR3,qCurrUR3,qMidwayUR3,linearUR5,qCurrUR5,qMidwayUR5,steps,Brick3,Brick4);
% qCurrUR3 = qMidwayUR3;
% qCurrUR5 = qMidwayUR5;

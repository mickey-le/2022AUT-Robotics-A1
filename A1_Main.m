clear all; clf; clc;

%% Initialise UR3 model and its base position 
myUR3 = UR3(0);
myUR3.model.base = transl(0.4, 0, 0);
myUR3.PlotAndColourRobot();

%% Initialise UR5 Linear model and its base position
linearUR5 = LinearUR5(0);
linearUR5.model.base = transl(-0.75, 0.35, -0.025) * trotx(pi/2) * troty(pi);
linearUR5.PlotAndColourRobot();

hold on;

%% Initialise the bricks and its position
Brick1 = Brick(transl(0, 0, 0.01) * rpy2tr(pi, 0, pi/2)); % Center
Brick2 = Brick(transl(0.2, 0, 0.01) * rpy2tr(pi, 0, pi/2)); % Right
Brick3 = Brick(transl(-0.2, 0, 0.01) * rpy2tr(pi, 0, pi/2)); % Left
Brick4 = Brick(transl(0, 0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Up
Brick5 = Brick(transl(0, -0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Down
Brick6 = Brick(transl(0.2, 0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Up-Right
Brick7 = Brick(transl(-0.2, 0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Up-Left
Brick8 = Brick(transl(-0.2, -0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Down-Left
Brick9 = Brick(transl(0.2, -0.4, 0.01) * rpy2tr(pi, 0, pi/2)); % Down-Right

Brick1.PlotBrickModel();
Brick2.PlotBrickModel();
Brick3.PlotBrickModel();
Brick4.PlotBrickModel();
Brick5.PlotBrickModel();
Brick6.PlotBrickModel();
Brick7.PlotBrickModel();
Brick8.PlotBrickModel();
Brick9.PlotBrickModel();

%% 
input('Press Enter to start');

% Steps number
steps = 100;

% Initial q and Ready q
% UR3:
qInitUR3 = myUR3.model.getpos();
qHomeUR3 = deg2rad([0 22 90 -22 -90 180]);

% UR5:
qInitUR5 = linearUR5.model.getpos();
qHomeUR5 = deg2rad([0 0 18 -90 -18 90 180]);

% Move robot to Home position
MoveIt.MoveTwoRobot(myUR3,qInitUR3,qHomeUR3,linearUR5,qInitUR5,qHomeUR5,steps);

% Set current robot q
qCurrUR3 = qHomeUR3;
qCurrUR5 = qHomeUR5;

%% Pick up Brick 6 and update current q
qCurrUR3 = PickAndPlace.OneRobotPickBrick(myUR3,qCurrUR3,Brick6,steps);

%% Place Brick 6 onto Brick 2 and update current q
qCurrUR3 = PickAndPlace.OneRobotPlaceBrick(myUR3,qCurrUR3,Brick6,Brick2,steps);

%% Pick up Brick 9 and update current q
qCurrUR3 = PickAndPlace.OneRobotPickBrick(myUR3,qCurrUR3,Brick9,steps);

%% Place Brick 9 onto Brick 6 and update current q
qCurrUR3 = PickAndPlace.OneRobotPlaceBrick(myUR3,qCurrUR3,Brick9,Brick6,steps);

%% Move Two Robot



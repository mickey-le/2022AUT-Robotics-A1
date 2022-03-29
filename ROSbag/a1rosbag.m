%% Initialise path and rosbag
clear all; clf; clc;
addpath '..'
addpath '../Original UR3'

myBag = rosbag('2018-03-20-18-34-46.bag');
selectBag = select(myBag,'Topic', '/joint_states');
msgs = readMessages(selectBag);

%% Create and plot UR3 object, move to home state
myUR3 = OrgUR3();
myUR3.PlotAndColourRobot();
qCurr = myUR3.model.getpos();
steps = 50;

qHome = deg2rad([0 -22 -90 22 90 180]);
MoveIt.MoveOneRobot(myUR3,qCurr,qHome,steps);
qCurr = qHome;

%% Run path from rosbag
for i=1:6003
    msgCell = msgs{i,1};
    qRobot = msgCell.Position';
    MoveIt.MoveOneRobot(myUR3,qCurr,qRobot,steps);
    qCurr = qRobot;
end



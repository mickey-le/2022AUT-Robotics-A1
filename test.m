% testBrick = Brick(transl(0,0,0) * rpy2tr(0,0,0));
clear all; clf; clc;
myUR3 = UR3();
myUR3.model.base = transl(0.4, 0, 0) * rpy2tr(pi/2,0,0);
myUR3.PlotAndColourRobot();
clear all; clf; clc;
myBag = rosbag('2018-03-20-18-34-46.bag');
selectBag = select(myBag,'Topic', '/joint_states');
msgs = readMessages(selectBag);


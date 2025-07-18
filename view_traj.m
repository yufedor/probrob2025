close all
clear
clc

#import calibration functions
source "frontSteeringCalibration.m"

#read the dataset file
[parameters, data] = parseDataset('dataset/dataset.txt');

#consider only the incremental information about the encoder to do the integration
traction_deltas = computeDeltaTicks(data.traction_ticks, 2^32);


#plot
figure;
plot(data.tracker_pose(:,1), data.tracker_pose(:,2), '.-');
axis equal;
title('Tracker Pose (Ground Truth)');
xlabel('x (m)');
ylabel('y (m)');

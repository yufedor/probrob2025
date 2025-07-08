close all
clear
clc

% Read the dataset file
fid = fopen('dataset.txt', 'r');
lines = {};
tline = fgetl(fid);
while ischar(tline)
    lines{end+1} = tline;
    tline = fgetl(fid);
end
fclose(fid);

% Initialize
x = [];
y = [];

% Process lines (skip first 9)
for i = 10:length(lines)
    line = strtrim(lines{i});
    tokens = ostrsplit(line, ':');
    tracker_pose = strtrim(tokens{end});
    xy = str2double(strsplit(tracker_pose));
    if numel(xy) >= 2
        x(end+1) = xy(1);
        y(end+1) = xy(2);
    end
end

% Plot
figure;
scatter(x, y, 'filled');
axis equal;
xlabel('x');
ylabel('y');
title('Tracker Pose');

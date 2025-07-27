close all
clear
clc

#import calibration functions
source 'frontSteeringCalibration.m';

#read the dataset file
[parameters, data] = parseDataset('dataset/dataset.txt');

#laser pose w.r.t. base_link
laser_offset = [1.5; 0; 0];

#handle encoder overflows
traction_deltas = computeDeltaTicks(data.traction_ticks, parameters.traction_wheel);
steer_deltas = computeDeltaTicks(data.steer_ticks, parameters.steering);
dt = diff(data.time);
dt(end+1) = dt(end);  #repeat last delta

#build measurements: [steering_ticks, traction_delta, dx, dy, dtheta]
n = length(traction_deltas);
Z = zeros(5, n-1);
for i = 2:n
  steer = steer_deltas(i);
  tr = traction_deltas(i);
  dx = data.tracker_pose(i,1) - data.tracker_pose(i-1,1);
  dy = data.tracker_pose(i,2) - data.tracker_pose(i-1,2);
  dtheta = data.tracker_pose(i,3) - data.tracker_pose(i-1,3);
  Z(:,i-1) = [steer; tr; dx; dy; dtheta];
end

#initial guess: [Ksteer, Ktraction, steer_offset, axis_length]
x = [parameters.Ksteer; parameters.Ktraction; parameters.steer_offset; parameters.axis_length];

#for odometry calibration convergence often occurs within 5–10 iterations
max_iters = 10;
tol = 1e-10;
chi_values = zeros(1, max_iters);
results_file = fopen('results/calibration_results.txt', 'w');
#plot
figure;
hold on;
grid on;
axis equal;
title('Trajectory Calibration Progress');
xlabel('x [m]');
ylabel('y [m]');

#plot ground truth
plot(data.tracker_pose(:,1), data.tracker_pose(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
for iter = 1:max_iters
  [x, chi] = oneRound(x, Z);
  chi_values(iter) = chi;
  fprintf('Iter %d:\n', iter);
  fprintf(' chi^2 = %.6f\n', chi);
  fprintf(' Ksteer       = %.6f \n', x(1));
  fprintf(' Ktraction    = %.6f \n', x(2));
  fprintf(' steer_offset = %.6f \n', x(3));
  fprintf(' axis_length  = %.6f \n', x(4));
  #save results to file
  fprintf(results_file, 'Iter %d:\n', iter);
  fprintf(results_file, ' chi^2 = %.6f\n', chi);
  fprintf(results_file, ' Ksteer       = %.6f \n', x(1));
  fprintf(results_file, ' Ktraction    = %.6f \n', x(2));
  fprintf(results_file, ' steer_offset = %.6f \n', x(3));
  fprintf(results_file, ' axis_length  = %.6f \n', x(4));
  fprintf(results_file, '\n');
  #simulate predicted trajectory using current estimate
  odom_pose = simulateOdometryTrajectory(data.steer_ticks, traction_deltas, x);
  plot(odom_pose(:,1), odom_pose(:,2), '-', 'DisplayName', sprintf('Est. Iter %d', iter));
  if chi < tol, break; end
end
fclose(results_file);

figure
plot(data.tracker_pose(:,1), data.tracker_pose(:,2), 'g-', 'LineWidth', 2); hold on;
xlabel('X [m]');
ylabel('Y [m]');
legend('Ground Truth (Tracker)');
title('Trajectory Comparison');
axis equal; grid on;

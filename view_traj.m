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

  # Handle angle wrapping for dtheta
  while dtheta > pi
    dtheta -= 2*pi;
  end
  while dtheta < -pi
    dtheta += 2*pi;
  end

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
fig1 = figure;
hold on;
grid on;
axis equal;
title('Trajectory Calibration Progress');
xlabel('x [m]');
ylabel('y [m]');

#plot ground truth
plot(data.tracker_pose(:,1), data.tracker_pose(:,2), 'g-', 'LineWidth', 1, 'DisplayName', 'Ground Truth');
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

  # Plot with different colors for better visualization
  colors = ['r', 'b', 'm', 'c', 'y', 'k'];
  color_idx = mod(iter-1, length(colors)) + 1;
  plot(odom_pose(:,1), odom_pose(:,2), '-', 'Color', colors(color_idx), ...
       'DisplayName', sprintf('Est. Iter %d (χ²=%.2e)', iter, chi));

  # Save intermediate trajectory plot every few iterations
  if mod(iter, 5) == 0 || iter == max_iters
    temp_fig = figure('Visible', 'off');
    plot(data.tracker_pose(:,1), data.tracker_pose(:,2), 'g-', 'LineWidth', 2); hold on;
    plot(odom_pose(:,1), odom_pose(:,2), 'r--', 'LineWidth', 2);
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('Ground Truth', sprintf('Iteration %d', iter), 'Location', 'best');
    title(sprintf('Trajectory at Iteration %d (χ²=%.2e)', iter, chi));
    axis equal; grid on;
    print(temp_fig, sprintf('results/trajectory_iter_%02d.png', iter), '-dpng', '-r300');
    close(temp_fig);
  end

  if chi < tol || (iter > 1 && abs(chi_values(iter-1) - chi) < 1e-12)
    break;
  end
end

# Save calibration progress plot
legend('show');
set(fig1, 'Position', [100, 100, 800, 600]);
saveas(fig1, 'results/calibration_progress.fig');

fclose(results_file);

# Final comparison plot
fig2 = figure;
plot(data.tracker_pose(:,1), data.tracker_pose(:,2), 'g-', 'LineWidth', 2); hold on;
odom_pose_final = simulateOdometryTrajectory(data.steer_ticks, traction_deltas, x);
plot(odom_pose_final(:,1), odom_pose_final(:,2), 'r--', 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
legend('Ground Truth (Tracker)', 'Calibrated Odometry', 'Location', 'best');
title('Final Trajectory Comparison');
axis equal; grid on;
set(fig2, 'Position', [200, 200, 800, 600]);

# Save final comparison plot
saveas(fig2, 'results/final_trajectory_comparison.fig');

# Plot chi-squared evolution
fig3 = figure;
valid_chi = chi_values(chi_values>0);
plot(1:length(valid_chi), valid_chi, 'bo-', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('Iteration');
ylabel('Chi-squared error');
title('Calibration Convergence');
grid on;
set(fig3, 'Position', [300, 300, 800, 600]);

# Save convergence plot
saveas(fig3, 'results/calibration_convergence.fig');

# Create a detailed results report
results_report_file = fopen('results/calibration_report.txt', 'w');
fprintf(results_report_file, '=== ODOMETRY CALIBRATION REPORT ===\n');
fprintf(results_report_file, 'Date: %s\n', datestr(now));
fprintf(results_report_file, 'Dataset: dataset/dataset.txt\n\n');

fprintf(results_report_file, '--- INITIAL PARAMETERS ---\n');
fprintf(results_report_file, 'Ksteer       = %.6f\n', parameters.Ksteer);
fprintf(results_report_file, 'Ktraction    = %.6f\n', parameters.Ktraction);
fprintf(results_report_file, 'steer_offset = %.6f\n', parameters.steer_offset);
fprintf(results_report_file, 'axis_length  = %.6f\n\n', parameters.axis_length);

fprintf(results_report_file, '--- FINAL CALIBRATED PARAMETERS ---\n');
fprintf(results_report_file, 'Ksteer       = %.6f\n', x(1));
fprintf(results_report_file, 'Ktraction    = %.6f\n', x(2));
fprintf(results_report_file, 'steer_offset = %.6f\n', x(3));
fprintf(results_report_file, 'axis_length  = %.6f\n\n', x(4));

fprintf(results_report_file, '--- CALIBRATION STATISTICS ---\n');
fprintf(results_report_file, 'Number of iterations: %d\n', iter);
fprintf(results_report_file, 'Final chi-squared error: %.6e\n', chi);
fprintf(results_report_file, 'Number of measurements: %d\n', size(Z,2));

# Calculate trajectory error statistics
trajectory_error_x = odom_pose_final(:,1) - data.tracker_pose(:,1);
trajectory_error_y = odom_pose_final(:,2) - data.tracker_pose(:,2);
trajectory_error_theta = odom_pose_final(:,3) - data.tracker_pose(:,3);

# Handle angle wrapping for theta error
for i = 1:length(trajectory_error_theta)
  while trajectory_error_theta(i) > pi
    trajectory_error_theta(i) -= 2*pi;
  end
  while trajectory_error_theta(i) < -pi
    trajectory_error_theta(i) += 2*pi;
  end
end

fprintf(results_report_file, '\n--- TRAJECTORY ERROR STATISTICS ---\n');
fprintf(results_report_file, 'X Error - Mean: %.4f m, Std: %.4f m, RMS: %.4f m\n', ...
        mean(trajectory_error_x), std(trajectory_error_x), sqrt(mean(trajectory_error_x.^2)));
fprintf(results_report_file, 'Y Error - Mean: %.4f m, Std: %.4f m, RMS: %.4f m\n', ...
        mean(trajectory_error_y), std(trajectory_error_y), sqrt(mean(trajectory_error_y.^2)));
fprintf(results_report_file, 'Theta Error - Mean: %.4f rad, Std: %.4f rad, RMS: %.4f rad\n', ...
        mean(trajectory_error_theta), std(trajectory_error_theta), sqrt(mean(trajectory_error_theta.^2)));

euclidean_error = sqrt(trajectory_error_x.^2 + trajectory_error_y.^2);
fprintf(results_report_file, 'Euclidean Error - Mean: %.4f m, Std: %.4f m, Max: %.4f m\n', ...
        mean(euclidean_error), std(euclidean_error), max(euclidean_error));

fclose(results_report_file);
fprintf('Detailed calibration report saved to: results/calibration_report.txt\n');

# Print final summary to console
fprintf('\n=== CALIBRATION COMPLETED ===\n');
fprintf('Final parameters saved to: results/calibration_results.txt\n');
fprintf('All plots saved to results/ directory\n');
fprintf('Mean trajectory error: %.4f m\n', mean(euclidean_error));
fprintf('Max trajectory error: %.4f m\n', max(euclidean_error));


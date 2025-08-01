1;

#computes odometry motion model given parameters x and encoder inputs
#x:	the actual calibration parameters
#ticks: the reading of the encoders
#delta: the estimated displacement in robot frame
function delta = h_odom(x, ticks)
  ksteer = x(1);
  ktraction = x(2);
  steer_offset = x(3);
  L = x(4);

  steer_tick = ticks(1);
  tr = ticks(2);

  alpha = ksteer * steer_tick + steer_offset;
  ds = ktraction * tr;
  dtheta = (ds / L) * tan(alpha);

  if abs(dtheta) < 1e-6
    dx = ds * cos(alpha);
    dy = ds * sin(alpha);
  else
    R = ds / dtheta;
    dx = R * sin(dtheta);
    dy = R * (1 - cos(dtheta));
  end

  delta = [dx; dy; dtheta];
end

#compute the error and the Jacobian for one measurement z
#does not depend on the state
#x:	the actual calibration parameters
#z:	the measurement vector
#e:	the error
#J:	the jacobian
function [e, J] = errorAndJacobian(x,z)
  ticks = z(1:2);
  meas = z(3:5);
  pred = h_odom(x,ticks);
  e = pred - meas;
  J = zeros(3,4);
  epsilon = 1e-3;
  inv_eps2 = 1/(2*epsilon);
  for i=1:4
    e_vec = zeros(4,1);
    e_vec(i) = epsilon;
    J(:,i) = inv_eps2 * (h_odom(x+e_vec, ticks) - h_odom(x-e_vec, ticks));
  end
end

#performs odometry parameter calibration using a Gauss-Newton least-squares loop
#accumulates total error and Jacobians over all measurements
#x: the actual calibration parameters
#z: the measurement matrix
#x_new: the estimate calibration parameters
#chi: sum of squared errors
function [x_new, chi] = oneRound(x, Z)
  H = zeros(4,4);
  b = zeros(4,1);
  nmeas = size(Z,2);
  chi = 0;
  for i = 1:nmeas
    [e, J] = errorAndJacobian(x, Z(:,i));
    H = H + J' * J;
    b = b + J' * e;
    chi = chi + e' * e;
  end
  dx = -H \ b;
  x_new = x + dx;
end

#simulate trajectory based on odometry and estimated parameters
#odom_pose: estimated pose
function odom_pose = simulateOdometryTrajectory(steer_ticks, traction_deltas, x)
  n = length(traction_deltas);
  odom_pose = zeros(n, 3); % [x, y, theta]
  for i = 2:n
    ticks = [steer_ticks(i); traction_deltas(i)];
    delta = h_odom(x, ticks);

    # Current pose
    x_curr = odom_pose(i-1,1);
    y_curr = odom_pose(i-1,2);
    theta_curr = odom_pose(i-1,3);

    # Transform delta from robot frame to global frame
    cos_theta = cos(theta_curr);
    sin_theta = sin(theta_curr);

    # Rotation matrix transformation
    dx_global = delta(1) * cos_theta - delta(2) * sin_theta;
    dy_global = delta(1) * sin_theta + delta(2) * cos_theta;
    dtheta_global = delta(3);

    # Update pose
    odom_pose(i,1) = x_curr + dx_global;
    odom_pose(i,2) = y_curr + dy_global;
    odom_pose(i,3) = theta_curr + dtheta_global;

    # Normalize angle
    while odom_pose(i,3) > pi
      odom_pose(i,3) -= 2*pi;
    end
    while odom_pose(i,3) < -pi
      odom_pose(i,3) += 2*pi;
    end
  end
end

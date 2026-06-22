clear
clc

% Load data
State = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\State_Data.dat");
measurements = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\IMU_Data.dat");

% Time
time = State(1, :);
dt = mean(diff(time));  % assume constant timestep

% Preallocate
N = length(time);
pos = zeros(3, N);
vel = zeros(3, N);

for i = 1:N-1
    % IMU acceleration in body frame
    acc_body = measurements(1:3, i);  % x_acc, y_acc, z_acc

    % Orientation from State: quaternion [w x y z]
    q = State(11:14, i)';  % quaternion must be row vector for quatrotate()

    % Rotate acc to ground frame using conjugate (MATLAB uses scalar-first quaternion format)
    acc_world = quatrotate(q, acc_body');  % returns row vector

    % Subtract gravity (in world frame)
    acc_world(3) = acc_world(3) - 9.807;

    % Integrate acceleration to velocity
    vel(:,i+1) = vel(:,i) + acc_world'*dt;

    % Integrate velocity to position
    pos(:,i+1) = pos(:,i) + vel(:,i)*dt + 0.5*acc_world'*dt^2;
end

% Plot results vs ground truth
figure;
titles = {'X Position', 'Y Position', 'Z Position'};
for j = 1:3
    subplot(3,1,j);
    plot(time, State(j+1,:), 'k--', 'DisplayName','True');
    hold on;
    plot(time, pos(j,:), 'b', 'DisplayName','Estimated');
    ylabel(titles{j});
    legend;
end
xlabel('Time (s)');
sgtitle('Position Estimation using IMU and True Orientation');

figure;
titles = {'X Velocity', 'Y Velocity', 'Z Velocity'};
for j = 1:3
    subplot(3,1,j);
    plot(time, State(j+4,:), 'k--', 'DisplayName','True');
    hold on;
    plot(time, vel(j,:), 'b', 'DisplayName','Estimated');
    ylabel(titles{j});
    legend;
end
xlabel('Time (s)');
sgtitle('Velocity Estimation using IMU and True Orientation');
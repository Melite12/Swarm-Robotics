% Manash Dugar
% Inputing data from the Rollbot's first simulation.
% Calculating values which the IMU would give in this motion 

clear
clc

% Importing data 
lin_data = readmatrix('C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\Mathematica - Sphere\linear_data2.txt');
ang_data = readmatrix('C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\Mathematica - Sphere\angular_data.txt');
inp_data = readmatrix('C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\Mathematica - Sphere\input_data2.txt');

% Extracting Data
time = lin_data(:, 1);
dt = 0.1;

x = lin_data(:, 2); % x-y position, ground frame
y = lin_data(:, 3); 
x_vel = lin_data(:, 4); % x-y vel
y_vel = lin_data(:, 5); 
x_acc = lin_data(:, 6); % x-y acc
y_acc = lin_data(:, 7);  

a = ang_data(:, 2); % ZYZ Euler Angles
b = ang_data(:, 3); 
g = ang_data(:, 4); 
a_vel = ang_data(:, 5);  % Derivatives
b_vel = ang_data(:, 6);  
g_vel = ang_data(:, 7);  

theta = inp_data(:, 2); % Position of Internal Mass 
theta_vel = inp_data (:, 3); % Velocity
theta_acc = inp_data(:,4); % Acceleration

R = 0.12;
r = 0.093;
phi = pi/4;


for (ii = 1:size(a,1)) % calculating angular velocity of axes (IMU DATA)

    % The order of euler angles is Z, Y, Z'  
    % Transforming to body frame 
    angular_imu = [-a_vel(ii)*sin(b(ii))*cos(g(ii)) + b_vel(ii)*sin(g(ii));
                    a_vel(ii)*sin(b(ii))*sin(g(ii)) + b_vel(ii)*cos(g(ii));
                    a_vel(ii)*cos(b(ii))            + g_vel(ii)];

    %summing the components to get IMU data
    x_axis_vel_imu(ii) = angular_imu(1);
    y_axis_vel_imu(ii) = angular_imu(2);
    z_axis_vel_imu(ii) = angular_imu(3);

end


% Converting between 
eul = [a b g];
quaternion = eul2quat(eul,"ZYZ");

euler = quat2eul(quaternion,'ZYZ');
euler(:,1);


for (ii = 1:size(a,1)) % rotating acceleration data to body frame and adding gravity (IMU Data)
    T = eul2rotm([a(ii), b(ii), g(ii)], "ZYZ");

    mat = [x_acc(ii);
           y_acc(ii);
           -9.807];

    acc_total = transpose(T) * mat;

    x_acc_imu(ii) = acc_total(1);
    y_acc_imu(ii) = acc_total(2);
    z_acc_imu(ii) = acc_total(3);
end

%Normalising angular data within -pi -> pi
a_fixed = wrapTo2Pi(a);
b_fixed = wrapTo2Pi(b);
g_fixed = wrapTo2Pi(g);

z = R * ones(1501,1); % data of the global z-axis
z_vel = zeros(1501,1);
z_acc = zeros(1501,1);

State_Data=[time,x,y,z,x_vel,y_vel,z_vel,x_acc,y_acc,z_acc,quaternion,x_axis_vel_imu', y_axis_vel_imu', z_axis_vel_imu',theta,theta_vel]';
writematrix(State_Data,'State_Data.dat','Delimiter',';');
type State_Data.dat;

IMU_Data = [x_acc_imu; y_acc_imu; z_acc_imu; x_axis_vel_imu; y_axis_vel_imu; z_axis_vel_imu; theta_vel'];
writematrix(IMU_Data,'IMU_Data.dat','Delimiter',';');
type IMU_Data.dat;

%Full Simulation 

for ii = (510 : size(a,1))
    [X,Y,Z] = sphere(10); % Main Rollbot 
    X = X * R + x(ii); 
    Y = Y * R + y(ii);
    Z = Z * R + R;

    surf(X,Y,Z, 'FaceColor',[1,1,1])
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    axis equal
    axis vis3d % all to show a nicer view (can't see internal axis though)
    view(3)
    xlim([-1 0.5])
    ylim([-1.2 0.5])
    zlim([0 0.25])
    alpha(0.3)

    hold on

    center = [x(ii), y(ii), R];
    % rot = eul2rotm(euler(ii,:), "ZYZ"); % Axes - Euler
    % x_axis_end = center' + R*(rot * [1; 0; 0]);
    % y_axis_end = center' + R*(rot * [0; 1; 0]);
    % z_axis_end = center' + R*(rot * [0; 0; 1]);

    rot = quat2rotm(quaternion(ii,:)); % Axes - Quaternion
    q = quaternion(ii,:);
    x_axis_end = center + R*(quatrotate(quatconj(q),[1,0,0]));
    y_axis_end = center + R*(quatrotate(quatconj(q),[0,1,0]));
    z_axis_end = center + R*(quatrotate(quatconj(q),[0,0,1]));

    plot3([center(1), x_axis_end(1)], [center(2), x_axis_end(2)], [center(3), x_axis_end(3)], 'r-', 'LineWidth', 2)
    plot3([center(1), y_axis_end(1)], [center(2), y_axis_end(2)], [center(3), y_axis_end(3)], 'g-', 'LineWidth', 2)
    plot3([center(1), z_axis_end(1)], [center(2), z_axis_end(2)], [center(3), z_axis_end(3)], 'b-', 'LineWidth', 2)

    [X2,Y2,Z2] = sphere; % Internal Mass
    internal_mass_end = center' + (rot * (r* [sin(phi)*cos(theta(ii));  sin(phi)*sin(theta(ii));  -cos(phi)]));
    X2 = X2 * 0.01 + internal_mass_end(1);
    Y2 = Y2 * 0.01 + internal_mass_end(2);
    Z2 = Z2 * 0.01 + internal_mass_end(3);
    surf(X2,Y2,Z2, 'FaceColor',[0.8,0.8,0])

    pause(0.001)
    hold off
end
%}

% Plotting IMU DATA
%{
figure(1)
subplot(3,1,1);
plot(time,x_acc_imu);
xlabel("Time (s)");
ylabel("Acceleration (m/s^2)");
title("IMU X acc");

subplot(3,1,2);
plot(time,y_acc_imu);
xlabel("Time (s)");
ylabel("Acceleration (m/s^2)");
title("IMU Y acc");

subplot(3,1,3);
plot(time,z_acc_imu);
xlabel("Time (s)");
ylabel("Acceleration (m/s^2)");
title("IMU Z acc");

figure(2);
subplot(3,1,1);
plot(time,[x_axis_vel_imu]);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("X Axis");

subplot(3,1,2);
plot(time,[y_axis_vel_imu]);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Y Axis");

subplot(3,1,3);
plot(time,[z_axis_vel_imu]);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Z Axis");

figure(3)
plot(time, theta_vel);
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Pendulum velocity");
%}

% PLOTTING DATA 
%{
figure(1); % x + x_acc
subplot(2,1,1);
plot(time, x);
xlabel("Time (s)");
ylabel("x-pos Global Frame (m)");
title("Rollbot X Position");

subplot(2,1,2);
plot(time, x_acc);
xlabel("Time (s)");
ylabel("x-acc Global Frame (m/s^2)");
title("Rollbot X Acceleration");

figure(2); % y + y_acc

subplot(2,1,1);  
plot(time, y);
xlabel("Time (s)");
ylabel("y-pos Global Frame (m)");
title("Rollbot Y Position");

subplot(2,1,2);
plot(time, y_acc);
xlabel("Time (s)");
ylabel("y-acc Global Frame (m/s^2)");
title("Rollbot Y Acceleration");

figure(3); % angular stuff

subplot(3,2,1);
plot(time, a);
xlabel("Time (s)");
ylabel("α (rad)");
title("Rollbot Roll Angle α(t)");

subplot(3,2,2);
plot(time, a_vel);
xlabel("Time (s)");
ylabel("dα/dt (rad/s)");
title("Rollbot Roll Rate α'(t)");

subplot(3,2,3);
plot(time, b);
xlabel("Time (s)");
ylabel("β (rad)");
title("Rollbot Pitch Angle β(t)");

subplot(3,2,4);
plot(time, b_vel);
xlabel("Time (s)");
ylabel("dβ/dt (rad/s)");
title("Rollbot Pitch Rate β'(t)");

subplot(3,2,5);
plot(time, g);
xlabel("Time (s)");
ylabel("γ (rad)");
title("Rollbot Yaw Angle γ(t)");

subplot(3,2,6);
plot(time, g_vel);
xlabel("Time (s)");
ylabel("dγ/dt (rad/s)");
title("Rollbot Yaw Rate γ'(t)");

figure(4);

subplot(3,1,1);
plot(time, theta);
xlabel("Time (s)");
ylabel("Pendulum pos: Body Frame (rad)");
title("Rollbot Pendulum Position");

subplot(3,1,2);
plot(time, theta_vel);
xlabel("Time (s)");
ylabel("Pendulum velocity: Body Frame (rad/s)");
title("Rollbot Pendulum Velocity");

subplot(3,1,3);
plot(time, theta_acc);
xlabel("Time (s)");
ylabel("Pendulum acceleration: Body Frame (rad/s^2)");
title("Rollbot Pendulum Acceleration");
    
%}

%PLOTTING AXES
%{
% --- Create figure and axes ---
f = figure('Name', '3D Axis Viewer');
ax = axes(f);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
view(ax, 3);

% --- Initial frame ---
t = 1;
scale = 0.3;
hx = quiver3(ax, 0, 0, 0, x_axis(t,1), x_axis(t,2), x_axis(t,3), scale, 'r', 'LineWidth', 2);
hy = quiver3(ax, 0, 0, 0, y_axis(t,1), y_axis(t,2), y_axis(t,3), scale, 'g', 'LineWidth', 2);
hz = quiver3(ax, 0, 0, 0, z_axis(t,1), z_axis(t,2), z_axis(t,3), scale, 'b', 'LineWidth', 2);
title(ax, ['Frame: ', num2str(t)]);

% --- Store arrows and data in UserData ---
f.UserData.hx = hx;
f.UserData.hy = hy;
f.UserData.hz = hz;
f.UserData.x_axis = x_axis;
f.UserData.y_axis = y_axis;
f.UserData.z_axis = z_axis;
f.UserData.ax = ax;

% --- Slider for stepping through frames ---
uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', 1501, 'Value', 1, ...
    'SliderStep', [1/(1501-1), 1/(1501-1)], ...
    'Position', [150, 20, 300, 20], ...
    'Callback', @(src, ~) updateAxes(round(src.Value)));

% --- Function to update all 3 arrows ---
function updateAxes(t)
    fig = gcf;
    hx = fig.UserData.hx;
    hy = fig.UserData.hy;
    hz = fig.UserData.hz;
    x_axis = fig.UserData.x_axis;
    y_axis = fig.UserData.y_axis;
    z_axis = fig.UserData.z_axis;
    ax = fig.UserData.ax;

    set(hx, 'UData', x_axis(t,1), 'VData', x_axis(t,2), 'WData', x_axis(t,3));
    set(hy, 'UData', y_axis(t,1), 'VData', y_axis(t,2), 'WData', y_axis(t,3));
    set(hz, 'UData', z_axis(t,1), 'VData', z_axis(t,2), 'WData', z_axis(t,3));
    title(ax, ['Frame: ', num2str(t)]);
end
%}

% Manual euler to quaternion
%{
for i = 10:10
    qa = [cos(a(i)/2), 0, 0, sin(a(i)/2)];
    qb = [cos(b(i)/2), 0, sin(b(i)/2), 0];
    qg = [cos(g(i)/2), 0, 0, sin(g(i)/2)];

    quaternion(i,:) = quatmultiply(quatmultiply(qa,qb),qg);

end
%}

%Plotting normalised Euler Angles
%{
figure(1) % They look weird because the number wraps back around every rotation 
subplot(3,1,1)
plot(time, a_fixed)
xlabel("Time (s)")
ylabel("alpha euler angle (rad)")

subplot(3,1,2)
plot(time, b_fixed)
xlabel("Time (s)")
ylabel("beta euler angle (rad)")

subplot(3,1,3)
plot(time, g_fixed)
xlabel("Time (s)")
ylabel("gamma euler angle (rad)")
%}

% Trying to get angular velocity in ground frame: use rotation matrix
% instead of quaternion if you come back to it
%{
for i = 1:size(quaternion,1)
    ground_data =   quatrotate(quaternion(i,:), [x_axis_vel_imu(i) 0 0]) + quatrotate(quaternion(i,:), [0 y_axis_vel_imu(i) 0]) + quatrotate(quaternion(i,:), [0 0 z_axis_vel_imu(i)]);
    Wx(i) = ground_data(1);
    Wy(i) = ground_data(2);
    Wz(i) = ground_data(3);
end

figure(1);
subplot(3,1,1);
plot(time, Wx);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Ground Frame X-axis Angular Velocity");

figure(1);
subplot(3,1,2);
plot(time, Wy);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Ground Frame Y-axis Angular Velocity");

figure(1);
subplot(3,1,3);
plot(time, Wx);
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Ground Frame Z-axis Angular Velocity");
%}


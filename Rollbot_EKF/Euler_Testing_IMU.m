% Manash Dugar
% Using Extended Kalman filter to estimate state vector of Rollbot
clear
clc

%  No Noise State Data: Rollbot state vector in Ground frame and
%  quaternion. Wx, Wy, Wz are in body frame 
% Format : (time,x,y,z,x',y',z',x'',y'',z'',w,a,b,g,Wx,Wy,Wz,θ,θ')
% Size   : (19 x 1501)
State = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\State_Data.dat");
time = State(1,:);
quaternion = State(11:14,:);

%  No Noise Measurements: Contains the IMU measurements we expect
% Format : (x_acc, y_acc, z_acc, Wx, Wy, Wz, theta_vel)
% Size   : (7 x 1501)
measurements = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\IMU_Data.dat");
theta_vel = measurements(7,:);

%Angular Data 2
ang_data = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\Rollbot_EKF\angular_data2.txt")';
time2 = ang_data(1,:);
a = ang_data(2,:);
b = ang_data(3,:);
g = ang_data(4,:);
a_vel = ang_data(5,:);
b_vel = ang_data(6,:);
g_vel = ang_data(7,:);



for ii = 1:size(a,2) % calculating angular velocity of axes (IMU DATA)

    % The order of euler angles is Z, Y, Z'  
    % Transforming to body frame 
    angular_imu(:,ii) = [-a_vel(ii)*sin(b(ii))*cos(g(ii)) + b_vel(ii)*sin(g(ii));
                          a_vel(ii)*sin(b(ii))*sin(g(ii)) + b_vel(ii)*cos(g(ii));
                          a_vel(ii)*cos(b(ii))            + g_vel(ii)];

end

% 1 - Orientation using IMU angular velocity 

pred_q = [1 0 0 0]';

% dt = 0.005;
% for i = 1:length(time2) 
%     H = quatmultiply(pred_q', [0 angular_imu(1:3,i)'])'; %Which way should it multiply. Video: qw * qi.
%     state_q(:,i) = pred_q + (H * (dt/2));
%     state_q(:,i) = state_q(:,i) / norm(state_q(:,i));
% 
%     pred_q = state_q(:,i);
% end

state_q(:,1) = [1 0 0.4 0]';
dt = 0.1;
for i = 1:length(State) 
    u = measurements(4:6,i);
    Wx = u(1);
    Wy = u(2);
    Wz = u(3);

    % H = quatmultiply(pred_q', [0 Wx Wy Wz])'; %Which way should it multiply. Video: qw * qi.
    % state_q(:,i) = pred_q + (H * (dt/2));
    % 
    % state_q(1,i+1) = state_q(1,i) + (-a*Wx-b*Wy-g*Wz) * (dt/2);
    % state_q(2,i+1) = state_q(2,i) + (w*Wx + g*Wy - b*Wz) * (dt/2);
    % state_q(3,i+1) = state_q(3,i) + (w*Wy + a*Wz - g*Wx) * (dt/2);
    % state_q(4,i+1) = state_q(4,i) + (w*Wz + b*Wx - a*Wy) * (dt/2);

    mat = [2/dt -Wx -Wy -Wz;
           Wx 2/dt Wz -Wy;
           Wy -Wz 2/dt Wx;
           Wz Wy -Wx 2/dt];
    state_q(:,i+1) = dt/2 * mat * state_q(:,i);

    state_q(:,i+1) = state_q(:,i+1) / norm(state_q(:,i+1));
end
state_q(:,1502) = [];

figure(1); % Quaternion
subplot(4,2,1);
plot(time, state_q(1,:));
xlabel("Time (s)");ylabel("w");title("w");

subplot(4,2,2);
plot(time, quaternion(1,:));
xlabel("Time (s)");ylabel("w");title("w - STATE");

subplot(4,2,3);
plot(time, state_q(2,:));
xlabel("Time (s)");ylabel("a");title("a");

subplot(4,2,4);
plot(time, quaternion(2,:));
xlabel("Time (s)");ylabel("a");title("a - STATE");

subplot(4,2,5);
plot(time, state_q(3,:));
xlabel("Time (s)");ylabel("b");title("b");

subplot(4,2,6);
plot(time, quaternion(3,:));
xlabel("Time (s)");ylabel("b");title("b - STATE");

subplot(4,2,7);
plot(time, state_q(4,:));
xlabel("Time (s)");ylabel("g");title("g");

subplot(4,2,8);
plot(time, quaternion(4,:));
xlabel("Time (s)");ylabel("g");title("g - STATE");

% figure(2)
% subplot(3,1,1)
% plot(time2,angular_imu(1,:))
% 
% subplot(3,1,2)
% plot(time2,angular_imu(2,:))
% 
% subplot(3,1,3)
% plot(time2,angular_imu(3,:))
%}

% 2 - Rotating acceleration 
% q * body frame * q^ = ground frame
%{

for i = 1:length(ang_data) %measurements
    %ground_acc(:,i) = quatmultiply(quatmultiply(quaternion(:,i)',[0 measurements(1:3,i)']), [quaternion(1,i)', -quaternion(2:4,i)']);

    eul = [a(i) b(i) g(i)];
    rotm = eul2rotm(eul,"ZYZ");
    ground_acc(:,i) = rotm * angular_imu(1:3,i);

end

%for quaternion, ground acc has 4 components

figure(2); 
subplot(3,2,1);
plot(time2, ground_acc(1,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("X-acc - MINE?");

subplot(3,2,2);
plot(time, State(8,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("X-acc - STATE");

subplot(3,2,3);
plot(time2, ground_acc(2,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("Y-acc - MINE?");

subplot(3,2,4);
plot(time, State(9,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("Y-acc - STATE");

subplot(3,2,5);
plot(time2, ground_acc(3,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("Z-acc - MINE?");

subplot(3,2,6);
plot(time, State(10,:));
xlabel("Time (s)");ylabel("acceleration (m/s)");title("Z-acc - STATE");
%}

% 3 - Chat
%{
dt = 0.001;
N = length(angular_imu);
orientation = zeros(4, N);
orientation(:,1) = [1; 0; 0.4; 0]; % initial quaternion

for i = 2:N
    omega = angular_imu(:, i-1);     % [Wx, Wy, Wz]
    theta = norm(omega) * dt;

    if theta < 1e-8
        delta_q = [1, 0, 0, 0];     % No rotation
    else
        axis = omega / norm(omega);
        delta_q = [cos(theta/2), sin(theta/2)*axis'];
    end

    orientation(:,i) = quatmultiply(orientation(:,i-1)', delta_q)';
    orientation(:,i) = orientation(:,i) / norm(orientation(:,i));
end

figure(1); % Quaternion
subplot(4,2,1);
plot(time2, orientation(1,:));
xlabel("Time (s)");ylabel("w");title("w");

subplot(4,2,2);
plot(time, quaternion(1,:));
xlabel("Time (s)");ylabel("w");title("w - STATE");

subplot(4,2,3);
plot(time2, orientation(2,:));
xlabel("Time (s)");ylabel("a");title("a");

subplot(4,2,4);
plot(time, quaternion(2,:));
xlabel("Time (s)");ylabel("a");title("a - STATE");

subplot(4,2,5);
plot(time2, orientation(3,:));
xlabel("Time (s)");ylabel("b");title("b");

subplot(4,2,6);
plot(time, quaternion(3,:));
xlabel("Time (s)");ylabel("b");title("b - STATE");

subplot(4,2,7);
plot(time2, orientation(4,:));
xlabel("Time (s)");ylabel("g");title("g");

subplot(4,2,8);
plot(time, quaternion(4,:));
xlabel("Time (s)");ylabel("g");title("g - STATE");

%}
% Manash Dugar
% Using Extended Kalman filter to estimate state vector of Rollbot
clear
clc

%  No Noise State Data: Rollbot state vector in Ground frame and
%  quaternion. Wx, Wy, Wz are in body fram 
% Format : (time,x,y,z,x',y',z',x'',y'',z'',w,a,b,g,Wx,Wy,Wz,θ,θ')
% Size   : (19 x 1501)
State = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Extracurricular\Robotics\Code\Rollbot_EKF\State_Data.dat");

%  No Noise Measurements: Contains the IMU measurements we expect
% Format : (x_acc, y_acc, z_acc, Wx, Wy, Wz, theta, theta_vel)
% Size   : (8 x 1501)
measurements = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Extracurricular\Robotics\Code\Rollbot_EKF\IMU_Data.dat");

% Attempt 2: Back to basics
%{
syms x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel dt real
vars_hor = [x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel];
vars_vert = [x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel]';
vars_H = [x_acc, y_acc w a b g Wx Wy Wz theta_vel]';

dt = 0.1;

F = [x + x_vel*dt + 0.5*x_acc*dt^2,
     y + y_vel*dt + 0.5*y_acc*dt^2,
     z + z_vel*dt + 0.5*z_acc*dt^2,
     x_vel + x_acc*dt,
     y_vel + y_acc*dt,
     z_vel + z_acc*dt,
     x_acc,
     y_acc,
     z_acc,
     -a*Wx - b*Wy - g*Wz * dt/2,
     w*Wx + g*Wy - b*Wz * dt/2,
     w*Wy + a*Wz - g*Wx * dt/2,
     w*Wz + b*Wz - a*Wy * dt/2,
     Wx,
     Wy,
     Wz,
     theta + theta_vel*dt
     theta_vel]';

H = squatmultiply(squatmultiply([w a b g], [0 x_acc y_acc -9.807]),[w -a -b -g]);
H = [H(2:4) Wx Wy Wz theta_vel];


state_x(:,1) = zeros(18,1);

for i = 1:length(measurements)
    z = measurements(:,i);
    
    acc(:,i) = quatmultiply(quatmultiply(state_x(10:13,i)',[0 z(1:3,1)']), [state_x(10,i)', -state_x(11:13,i)']);
    acc(3,i) = acc(3,i) - 9.807;

    state_x(7,i+1) = acc(1,i);
    state_x(8,i+1) = acc(2,i);
    state_x(9,i+1) = acc(3,i);

    state_x(4,i+1) = state_x(4,i) + state_x(7,i+1)*dt;
    state_x(5,i+1) = state_x(5,i) + state_x(8,i+1)*dt;
    state_x(6,i+1) = state_x(6,i) + state_x(9,i+1)*dt;

    state_x(1,i+1) = state_x(1,i) + dt*state_x(4,i+1) + 0.5*dt^2*state_x(7,i+1);
    state_x(2,i+1) = state_x(2,i) + dt*state_x(5,i+1) + 0.5*dt^2*state_x(8,i+1);
    state_x(3,i+1) = state_x(3,i) + dt*state_x(6,i+1) + 0.5*dt^2*state_x(9,i+1);

    state_x(10:13,i+1) = State(11:14,i);

end

start = 7;

figure(1); % x_pos
subplot(2,1,1);
plot(State(1,1:300), State(start+1,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - STATE");

subplot(2,1,2);
plot(State(1,1:300), state_x(start,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - KALMAN");

figure(2); % x_pos
subplot(2,1,1);
plot(State(1,1:300), State(start+2,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - STATE");

subplot(2,1,2);
plot(State(1,1:300), state_x(start+1,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - KALMAN");

figure(3); % x_pos
subplot(2,1,1);
plot(State(1,1:300), State(start+3,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - STATE");

subplot(2,1,2);
plot(State(1,1:300), state_x(start+2,1:300));
xlabel("Time (s)");
ylabel("");
title("Rollbot - KALMAN");

%}

% Attempt 1

% Initialisation
syms x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel dt real
vars_hor = [x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel];
vars_vert = [x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel]';
vars_H = [x_acc, y_acc w a b g Wx Wy Wz theta_vel]';

    % Constants
    dt = 0.1; %UNSURE
    acc_err = 0.3;
    gyro_err = 3.1;
    
    % Storage Initialisation
    state_x = zeros(18, 1501);
    pred_x = zeros(18,1502);
    
    % Matrices
    F = [x + x_vel*dt + 0.5*x_acc*dt^2,
         y + y_vel*dt + 0.5*y_acc*dt^2,
         z + z_vel*dt + 0.5*z_acc*dt^2,
         x_vel + x_acc*dt,
         y_vel + y_acc*dt,
         z_vel + z_acc*dt,
         x_acc,
         y_acc,
         z_acc,
         (-a*Wx - b*Wy - g*Wz) * dt/2,
         (w*Wx + g*Wy - b*Wz) * dt/2,
         (w*Wy + a*Wz - g*Wx) * dt/2,
         (w*Wz + b*Wz - a*Wy) * dt/2,
         Wx,
         Wy,
         Wz,
         theta + theta_vel*dt
         theta_vel];
    
    Jf = jacobian(F, vars_hor);
    
    % First row of H is zero for the w part of the quaternion produced
    % during rotation
    %H = quatmultiply(quatmultiply([w a b g], [0 x_acc y_acc -9.807]), [w -a -b -g]);
    H = squatmultiply(squatmultiply([w a b g], [0 x_acc y_acc -9.807]),[w -a -b -g]);
    H = [H(2:4) Wx Wy Wz theta_vel];

    Jh = jacobian(H,vars_hor);

    % Wrong because the variance order is different
    % Q_acc_block = acc_err^2 * [dt^4/4 dt^3/2 dt^2/2;
    %                              dt^3/2 dt^2   dt;
    %                              dt^2/2 dt     1];
    % Q = blkdiag(Q_acc_block, Q_acc_block, Q_acc_block); %UNFINISHED
    Q = acc_err^2 * eye(18);

    R_accel = 0.01*acc_err^2 * eye(3); 
    R_gyro  = 0.01*gyro_err^2 * eye(4);  
    R = blkdiag(R_accel, R_gyro);

% Predicition
init_x = zeros(18,1);
init_P = 500*eye(18);

pred_x(:,1) = double(subs(F, vars_vert, init_x));
dF = double(subs(Jf, vars_vert, init_x));
pred_P = dF * init_P * transpose(dF) + Q;

%Loop
time_len = 100;
for i = 1:time_len  %length(measurements)
    % disp(pred_x(:,i))
    
    % Measurement
    z  = measurements(:,i);
    Hx = double(subs(H, vars_vert, pred_x(:,i)))';
    dH = double(subs(Jh, vars_vert, pred_x(:,i)));
    % disp(Hx)
    % disp(dH)

    % Update
    K = pred_P*transpose(dH) * inv(dH*pred_P*transpose(dH) + R);
    state_x(:,i) =  pred_x(:,i) + K*(z - Hx);
    %state_x(10:13,i) = State(11:14,i);
    state_P = (eye(18) - K*dH) * pred_P * transpose(eye(18) - K*dH) + K*R*transpose(K);
    
    % disp(K)
    % disp(state_x(:,i))

    % Predict
    pred_x(:,i+1) = double(subs(F, vars_vert, state_x(:,i)));
    dF = double(subs(Jf, vars_vert, state_x(:,i)));
    pred_P = dF * state_P * transpose(dF) + Q;

end

%plotting x,y and z together
start = 16;

figure(1); 
subplot(2,1,1);
plot(State(1,1:time_len), State(start+1,1:time_len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot X - STATE");

subplot(2,1,2);
plot(State(1,1:time_len), state_x(start,1:time_len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot X - KALMAN");

figure(2);
subplot(2,1,1);
plot(State(1,1:time_len), State(start+2,1:time_len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Y - STATE");

subplot(2,1,2);
plot(State(1,1:time_len), state_x(start+1,1:time_len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Y - KALMAN");

figure(3); 
subplot(2,1,1);
plot(State(1,1:time_len), State(start+3,1:time_len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Z - STATE");

subplot(2,1,2);
plot(State(1,1:time_len), state_x(start+2,1:time_len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Z - KALMAN");

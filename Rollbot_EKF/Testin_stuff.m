%Testing
clear
clc

len =  150; % length(measurements);

%  No Noise State Data: Rollbot state vector in Ground frame and
%  quaternion. Wx, Wy, Wz are in body frame 
% Format : (time,x,y,z,x',y',z',x'',y'',z'',w,a,b,g,Wx,Wy,Wz,θ,θ')
% Size   : (19 x 1501)
State = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\State_Data.dat");
quat = State(11:14,1:len);
acc = State(8:10,1:len);

%  No Noise Measurements: Contains the IMU measurements we expect
% Format : (x_acc, y_acc, z_acc, Wx, Wy, Wz, theta_vel)
% Size   : (7 x 1501)
measurements = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\IMU_Data.dat");

syms x y z x_vel y_vel z_vel x_acc y_acc z_acc w a b g Wx Wy Wz theta theta_vel dt real


% Constants
    dt = 0.1; %UNSURE
    acc_err = 0.3 ;  
    gyro_err = 3.1 ;

% EKF ORIENTATION - DONE... YAYY

vars = [w a b g Wx Wy Wz];

F = [w + (-a*Wx - b*Wy - g*Wz) * (dt/2),
     a + (w*Wx - g*Wy + b*Wz) * (dt/2),
     b + (w*Wy - a*Wz + g*Wx) * (dt/2),
     g + (w*Wz - b*Wx + a*Wy) * (dt/2),
     Wx,
     Wy,
     Wz];
Jf = jacobian(F, vars);

H = [zeros(3,4), eye(3)];

Q = 0.3*eye(7);  %acc_err^2 * eye(7);
R = 0.01*eye(3); %gyro_err^2* eye(3);

init_x = [1 0 0.383 0 0 0 0]';
init_P = 100*eye(7);

pred_x(:,1) = double(subs(F, vars', init_x));
dF = double(subs(Jf, vars', init_x));
pred_P = dF * init_P * transpose(dF) + Q;

for i = 1:len
    
    z = measurements(4:6,i);
    Hx = H * pred_x(:,i);
    
    K = pred_P * transpose(H) * inv(H * pred_P * transpose(H) + R);

    state_x(:,i) = pred_x(:,i) + K * (z - Hx);
    q_state = state_x(1:4,i);
    state_x(1:4,i) = q_state / norm(q_state);
    state_P = (eye(7) - K*H) * pred_P * transpose(eye(7) - K*H) + K*R*transpose(K);

    pred_x(:,i+1) = double(subs(F,vars',state_x(:,i)));
    dF = double(subs(Jf, vars', state_x(:,i)));
    pred_P = dF * state_P * transpose(dF) + Q;
end
%}


%PLOTTING

start_kal = 1;
start = 10;

figure(1); 
subplot(3,2,1);
plot(State(1,1:len), State(start+1,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot a - STATE");

subplot(3,2,2);
plot(State(1,1:len), state_x(start_kal,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot a - KALMAN");

subplot(3,2,3);
plot(State(1,1:len), State(start+2,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot b - STATE");

subplot(3,2,4);
plot(State(1,1:len), state_x(start_kal+1,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot b - KALMAN");

subplot(3,2,5);
plot(State(1,1:len), State(start+3,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot g - STATE");

subplot(3,2,6);
plot(State(1,1:len), state_x(start_kal+2,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot g - KALMAN");

figure(2); 
subplot(2,1,1);
plot(State(1,1:len), State(start+4,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot X - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+3,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot X - KALMAN");

figure(3);
subplot(2,1,1);
plot(State(1,1:len), State(start+5,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Y - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+4,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Y - KALMAN");

figure(4); 
subplot(2,1,1);
plot(State(1,1:len), State(start+6,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Z - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+5,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Z - KALMAN");

%}




% Rotating acceleration and integrating to ensure understanding

% First add acceleration to the F, H and matrix in size and equations
% PLEASE CHECK AND ENSURE THE ROTATION EQUATION IS RIGHT AND WORKING
%My measurement will have more, ensure you're changing everything
%accordingly
% Make a copy and try it. don't change the code above 
% 1: Code out how I'm going to plot it and plot the right results
% 2: Write on paper and test components (rotating, matrices ...)
% 3: REWRITE THE CODE (don't just copy shit). 


for i = 1:len
    % Rotate Frame
    % q = quaternion(state_x(1,i),state_x(2,i),state_x(3,i),state_x(4,i));
    % ground_acc2(:,i) = rotateframe(q, measurements(1:3,i)');
    % ground_acc2(3,i) = ground_acc2(3,i) + 9.807;
    
    % Quatmultiply - makes the indexing so annoying
    % ground_acc2(:,i) = quatmultiply(quatmultiply([state_x(1:4,i)'], [0 measurements(1:3,i)']), [state_x(1,i) -state_x(2:4,i)']);
    % ground_acc2(4,i) = ground_acc2(4,i) + 9.807;

    % quatrotate
    ground_acc2(:,i) = quatrotate([state_x(1,i) -state_x(2:4,i)'], measurements(1:3,i)');
    ground_acc2(3,i) = ground_acc2(3,i) + 9.807;
end


% CODE
%{
vars = [x_acc y_acc z_acc w a b g Wx Wy Wz];

F = [x_acc,
     y_acc,
     z_acc,
     w + (-a*Wx - b*Wy - g*Wz) * (dt/2),
     a + (w*Wx - g*Wy + b*Wz) * (dt/2),
     b + (w*Wy - a*Wz + g*Wx) * (dt/2),
     g + (w*Wz - b*Wx + a*Wy) * (dt/2),
     Wx,
     Wy,
     Wz];
Jf = jacobian(F, vars);

H = [eye(3) zeros(3,7);
    zeros(3,7) eye(3)];

Q = 0.3*eye(10);  %acc_err^2 * eye(7);
R = 0.01*eye(6); %gyro_err^2* eye(3);

init_x = [0 0 0 1 0 0.383 0 0 0 0]';
init_P = 100*eye(10);

pred_x(:,1) = double(subs(F, vars', init_x));
dF = double(subs(Jf, vars', init_x));
pred_P = dF * init_P * transpose(dF) + Q;

for i = 1:len
    
    % Measurements
    z = measurements(1:6,i);
    if i == 1
        q = [init_x(4) -init_x(5:7)'];
    else
        q = [state_x(4,i-1), -state_x(5:7,i-1)'];
    end
    z(1:3) = quatrotate(q, z(1:3)');
    z(3) = z(3) + 9.807;
    ground_acc(:,i) = z(1:3);
    
    Hx = H * pred_x(:,i);
    K = pred_P * transpose(H) * inv(H * pred_P * transpose(H) + R);

    state_x(:,i) = pred_x(:,i) + K * (z - Hx);
    q_state = state_x(4:7,i);
    state_x(4:7,i) = q_state / norm(q_state);
    state_P = (eye(10) - K*H) * pred_P * transpose(eye(10) - K*H) + K*R*transpose(K);

    pred_x(:,i+1) = double(subs(F,vars',state_x(:,i)));
    dF = double(subs(Jf, vars', state_x(:,i)));
    pred_P = dF * state_P * transpose(dF) + Q;
end
%}


Quat_data=[state_x(1:4,:)];
writematrix(Quat_data,'Quat_data.dat','Delimiter',';');
type Quat_data.dat;

% PLOTTING
%{
start = 7;

figure(1); 
subplot(3,2,1);
plot(State(1,1:len), State(start+1,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - STATE");

subplot(3,2,2);
plot(State(1,1:len), ground_acc(1,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - KALMAN");

subplot(3,2,3);
plot(State(1,1:len), State(start+2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - STATE");

subplot(3,2,4);
plot(State(1,1:len), ground_acc(2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - KALMAN");

subplot(3,2,5);
plot(State(1,1:len), State(start+3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - STATE");
 
subplot(3,2,6);
plot(State(1,1:len), ground_acc(3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - KALMAN");


start_kal = 5;
start = 11;

figure(2); 
subplot(2,1,1);
plot(State(1,1:len), State(start+1,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot a - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot a - KALMAN");

figure(3);
subplot(2,1,1);
plot(State(1,1:len), State(start+2,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot b - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+1,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot b - KALMAN");

figure(4); 
subplot(2,1,1);
plot(State(1,1:len), State(start+3,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot g - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+2,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot g - KALMAN");

figure(5); 
subplot(2,1,1);
plot(State(1,1:len), State(start+4,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot X - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+3,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot X - KALMAN");

figure(6);
subplot(2,1,1);
plot(State(1,1:len), State(start+5,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Y - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+4,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Y - KALMAN");

figure(7); 
subplot(2,1,1);
plot(State(1,1:len), State(start+6,1:len));
xlabel("Time (s)");
ylabel("Angular velocity (rad/s)");
title("Rollbot Z - STATE");

subplot(2,1,2);
plot(State(1,1:len), state_x(start_kal+5,1:len));
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");
title("Rollbot Z - KALMAN");
%}

figure(5)
start = 7;

subplot(3,2,1);
plot(State(1,1:len), State(start+1,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - STATE");

subplot(3,2,2);
plot(State(1,1:len), ground_acc2(1,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - KALMAN");

subplot(3,2,3);
plot(State(1,1:len), State(start+2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - STATE");

subplot(3,2,4);
plot(State(1,1:len), ground_acc2(2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - KALMAN");

subplot(3,2,5);
plot(State(1,1:len), State(start+3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - STATE");
 
subplot(3,2,6);
plot(State(1,1:len), ground_acc2(3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - KALMAN");

% Rotating acceleration data 

State = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\State_Data.dat");
time = State(1,:);
quat = State(11:14,:);
acc = State(8:10,:);

measurements = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\IMU_Data.dat");
len = 1000; %length(measurements);

Quat_data = readmatrix("C:\Users\manas\OneDrive - Northwestern University\Northwestern\Robotics\Rollbot_EKF\Quat_data.dat");

% Plot 1 - check quat data 
num = 2;
figure(1)
subplot(2,1,1);
plot(time,quat(num,:));

subplot(2,1,2);
plot(time,Quat_data(num,:));

% Rotate acceleration - with my quaternion and measurement

for i = 1:len
    % Rotate Frame
    % q = quaternion(Quat_data(1,i),Quat_data(2,i),Quat_data(3,i),Quat_data(4,i));
    % ground_acc2(:,i) = rotateframe(q, measurements(1:3,i)');
    % ground_acc2(3,i) = ground_acc2(3,i) + 9.807;
    
    % Quatmultiply 
    ground_acc(:,i) = quatmultiply(quatmultiply([Quat_data(1:4,i)'], [0 measurements(1:3,i)']), [Quat_data(1,i) -Quat_data(2:4,i)']);
    ground_acc(4,i) = ground_acc(4,i) + 9.807;

    % quatrotate
    % ground_acc(:,i) = quatrotate([Quat_data(1,i) -Quat_data(2:4,i)'], measurements(1:3,i)');
    % ground_acc(3,i) = ground_acc(3,i) + 9.807;
end

figure(2)
start = 7;

subplot(3,2,1);
plot(State(1,1:len), State(start+1,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - STATE");

subplot(3,2,2);
plot(State(1,1:len), ground_acc(2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot X - KALMAN");

subplot(3,2,3);
plot(State(1,1:len), State(start+2,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - STATE");

subplot(3,2,4);
plot(State(1,1:len), ground_acc(3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Y - KALMAN");

subplot(3,2,5);
plot(State(1,1:len), State(start+3,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - STATE");
 
subplot(3,2,6);
plot(State(1,1:len), ground_acc(4,1:len));
xlabel("Time (s)");
ylabel("Acceleration (ms^-2)");
title("Rollbot Z - KALMAN");


%Kalman 0.0000    1.1748   -0.8868   -9.7019

%State 0.0000    0.3253    0.1410   -9.8065

function compare_quat(i, Quat_data, quat, measurements)
    kalman =  quatmultiply(quatmultiply([Quat_data(1:4,i)'], [0 measurements(1:3,i)']), [Quat_data(1,i) -Quat_data(2:4,i)']);
    stt =  quatmultiply(quatmultiply([quat(1:4,i)'], [0 measurements(1:3,i)']), [quat(1,i) -quat(2:4,i)']);

    fprintf("Kalman : %g \n", kalman');
    fprintf("Quaternion : %g \n", Quat_data(1:4,i)');

    fprintf("State : %g \n", stt');
    fprintf("Quaternion: %g \n", quat(1:4,i)');

end

compare_quat(693, Quat_data, quat, measurements);
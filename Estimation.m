close all
clear all

load('distances.mat', 'D');
load('speeds.mat', 'Sp');
for veh = 1:12
    
%Initialization
Nsamples=120*10;
dt = .1;
t=0:dt:dt*Nsamples;
Vtrue = 10;

Xinitial = 0;
%Xtrue = Xinitial + Vtrue * t
Xtrue = D(veh,:);

Xk_prev = [0; 
    1*Vtrue];

Xk=[];

% Phi represents the dynamics of the system.
Phi = [1 dt;
       0  1];

% The error matrix 
sigma_model = 1;

P = [sigma_model^2             0;
                 0 sigma_model^2];
             
% Q is the process noise covariance.
Q = [0.5 0;
     0 0.5];
 
% M is the measurement matrix. X is measured 
M = [1 0];

% R is the measurement noise covariance. 
sigma_meas = .1; 
R = sigma_meas^2;


% Buffers for later display
Xk_buffer = zeros(2,Nsamples+1);
Xk_buffer(:,1) = Xk_prev;
Z_buffer = zeros(1,Nsamples+1);

% Kalman iteration 
for k=1:Nsamples
    
    % Z is the measurement vector. 
    Z = Xtrue(k+1)+sigma_meas*randn;
    Z_buffer(k+1) = Z;
    
    % Kalman iteration
    P1 = Phi*P*Phi' + Q;
    S = M*P1*M' + R;
    
    % K is Kalman gain.
    K = P1*M'*inv(S);
    P = P1 - K*M*P1;
    
    Xk = Phi*Xk_prev + K*(Z-M*Phi*Xk_prev);
    Xk_buffer(:,k+1) = Xk;
    
    % For the next iteration
    Xk_prev = Xk; 
end

%Save the driving distances and speeds for plotting
Xk_d(veh,:) = Xk_buffer(1,:);
Xk_sum(veh,:) = Xk_buffer(2,:);
end

%Calculate the queue lengths for each iterations. 
%If the speed is lower than 2 the vehicle is assumed
%to be in a queue.
for col = 250:1201
    queueLengthEst = 0;
    queueLengthSim = 0;
    for row = 1:12
       if(Xk_sum(row,col) < 2)
          queueLengthEst = queueLengthEst + 1;
       end
       if(Sp(row,col) < 2)
          queueLengthSim = queueLengthSim + 1; 
       end
    end
   queueLengths(1,col) = queueLengthEst; 
   queueLengths(2,col) = queueLengthSim;
end

%Distance plot
figure;
plot(t,Xk_d(1,:),'r');
hold on;
for d = 2:12
    plot(t,Xk_d(d,:),'r');
end
plot(t,Xtrue,'b');
title('Driven distances');
xlabel('Time (s)');
ylabel('Distance (m)');

%Queue length plot
figure;
plot(t,queueLengths(1,:),'b');
hold on;
plot(t,queueLengths(2,:),'r');
title('Queue Lengths');
xlabel('Time (s)');
ylabel('Queue Length');
legend('Simulated queue lengths','Kalman estimated queue lengths');

%dxkbuffer = size(Xk_buffer)
%figure;
%plot(t,Xtrue,'g');
%hold on;
%plot(t,Z_buffer,'c');
%plot(t,Xk_buffer(1,:),'m');
%title('Position estimation results');
%xlabel('Time (s)');
%ylabel('Position (m)');
%legend('True position','Kalman estimated displacement');

%%%%%%Velocity analysis
%figure;
%plot(t,ones(size(t))*Vtrue,'m');
%plot(t, Sp(1,:), 'm');
%hold on;
%plot(t,Xk_buffer(2,:),'k');
%title('Velocity estimation results');
%xlabel('Time (s)');
%ylabel('Velocity (m/s)');
%legend('True velocity','Estimated velocity by Kalman filter');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


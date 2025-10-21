clear all; close all;
L = 2.7; % L = wheelbase (m)
track = 1.572; % track (m)


%% off-tracking distance 
steering_angle = [2, 5, 10, 15, 20, 30]; % deg
steering_angle_rad = deg2rad(steering_angle);
R = L ./ steering_angle_rad; % Truning Radius (m)
Delta = L^2./(2*R); % Delta = off-track (m)
figure(1);clf; 
plot(steering_angle, Delta,'-ro');
xlabel('steering angle (deg)');
ylabel('off-tracking distance \Delta (m)');
grid on; grid minor; box on;
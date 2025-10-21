clear all; close all;
L = 2.7; % L = wheelbase (m) for a bicycle model

%% steering angle = sine wave, 0.05 Hz, Amplitude = 5, 25
dt = 0.01; % time step size (sec)
t = 0:dt:20-dt; % time step vector (sec)
T = 20; % sine wave period (T = 20 sec)
f = 1/T; % frequency = 1/T
sta_amp = 30; % Steering Angle Amplitude = 30 (deg)
sta = sta_amp * cosd(linspace(0,360,length(t))); % steering angle step vector (deg)


%% initial state
figure(1); clf; hold on; box on;
xlabel('x (m)'); ylabel('y (m)');
v_x= 0; % vehicle initial speed to x-dir (0 m/sec)
v_y= 0; % vehicle initial speed to y-dir (0 m/sec)
theta = 0; % vehicle inital yaw angle (0 rad)
cmd_sta = deg2rad(0); % initial command of steering angle (0 rad)
v = 1; % vehicle speed to forward-dir = 1 m/s (v = sqrt(v_x^2 + v_y^2))

fd = [0, 0]; % Coord of initial front wheel center = (0, 0) 
fdx = cos(theta)*fd(1) - sin(theta)*fd(2);
fdy = sin(theta)*fd(1) + cos(theta)*fd(2);
rd = [-L, 0]; % Coord of initial rear wheel center = (0, 0) 
rdx = cos(theta)*rd(1) - sin(theta)*rd(2);
rdy = sin(theta)*rd(1) + cos(theta)*rd(2);
[updatedFx, updatedFy, updatedRx, updatedRy, updatedVx, updatedVy] = ...
veh_obj(theta, cmd_sta, fdx, fdy, rdx, rdy, v_x, v_y, L);


vehicle = fill(updatedVx,updatedVy,'w','EraseMode','normal');
Ffw = fill(updatedFx,updatedFy,'r','EraseMode','normal'); % front wheel fill object
Frw = fill(updatedRx,updatedRy,'b','EraseMode','normal'); % rear wheel fill object
pd = plot([fdx rdx],[fdy rdy],'k','LineWidth',2); % driveshaft line object
track_fx = zeros(1,2); % front wheel center (x,y) position 
track_rx = zeros(1,2); % rear wheel center (x,y) position 
% front wheel track line object
Ltrack_fw = plot(track_fx(:,1),track_fx(:,2),'r','LineWidth',2); 
% rear wheel track line object
Ltrack_rw = plot(track_rx(:,1),track_rx(:,2),'b:','LineWidth',2); 
plot(0,0,'rx'); % plot front wheel initial (x,y) position (Origin of the coord)

%% simulation 
for i = 1:1:length(t)
    cmd_sta = deg2rad(sta(i)) - theta;
    v_x = v_x + v * cos( theta + cmd_sta ) * dt;
    v_y = v_y + v * sin( theta + cmd_sta ) * dt;
    theta = theta + v * tan(cmd_sta)/L * dt;
    fdx = cos(theta)*fd(1) - sin(theta)*fd(2);
    fdy = sin(theta)*fd(1) + cos(theta)*fd(2);
    rdx = cos(theta)*rd(1) - sin(theta)*rd(2);
    rdy = sin(theta)*rd(1) + cos(theta)*rd(2);
    [updatedFx, updatedFy, updatedRx, updatedRy, updatedVx, updatedVy] ...
    = veh_obj(theta, cmd_sta, fdx, fdy, rdx, rdy, v_x, v_y, L);
    
    
    set(vehicle, 'Xdata', updatedVx, 'Ydata', updatedVy); 
    set(Ffw, 'Xdata', updatedFx,'Ydata', updatedFy); 
    set(Frw, 'Xdata', updatedRx,'Ydata', updatedRy);
    log_fw(i,:) = [fdx+v_x fdy+v_y];
    log_rw(i,:) = [rdx+v_x rdy+v_y];
    set(pd, 'Xdata', [fdx+v_x rdx+v_x],'Ydata',[fdy+v_y rdy+v_y]); % Update driveshaft
    set(Ltrack_fw, 'Xdata', log_fw(:,1),'Ydata',log_fw(:,2)); % Update track of fw
    set(Ltrack_rw, 'Xdata', log_rw(:,1),'Ydata',log_rw(:,2)); % Update track of rw
    title(strcat('steering angle : ',num2str(rad2deg(cmd_sta)),'(deg)' ));
    axis equal; grid on;
    axis([v_x-11 v_x+9 -3.5 3.5])
    drawnow limitrate
    F(i) = getframe(gcf);
end

% Redraw whole simulation
axis([-5 25 -3.5 3.5])
title('result')
drawnow limitrate
F(i+1) = getframe(gcf);
%% save as vedio
v = VideoWriter('sim_off_track_sine.avi');
v.FrameRate = 100;
open(v);
for t = 1 : length(F)
writeVideo(v,F(t));
end
for t2 = 1:1:100 
writeVideo(v,F(length(F))); % 마지막 frame 유지
end
close(v);

function [updatedFx, updatedFy, updatedRx, updatedRy, updatedVx, updatedVy] = ...
veh_obj(theta, cmd_sta, fdx, fdy, rdx, rdy, v_x, v_y,L)
Loa = 4.57; % Overall length (4.57 m)
Woa = 1.8; % Overall width (1.8 m)
Lwh = 0.7; % wheel length (0.7 m)
Wwh = 0.42; % wheel width (0.42 m)
shape_x = [-Lwh/2 Lwh/2 Lwh/2 -Lwh/2]; 
shape_y = [Wwh/2 Wwh/2 -Wwh/2 -Wwh/2]; 
shape_vx = [-L-(Loa-L)/2 (Loa-L)/2 (Loa-L)/2 -L-(Loa-L)/2];
shape_vy = [Woa/2 Woa/2 -Woa/2 -Woa/2];

updatedFx = [shape_x(1) * cos(theta+cmd_sta) - shape_y(1) * sin(theta+cmd_sta) + fdx + v_x
shape_x(2) * cos(theta+cmd_sta) - shape_y(2) * sin(theta+cmd_sta) + fdx + v_x
shape_x(3) * cos(theta+cmd_sta) - shape_y(3) * sin(theta+cmd_sta) + fdx + v_x
shape_x(4) * cos(theta+cmd_sta) - shape_y(4) * sin(theta+cmd_sta) + fdx + v_x];
updatedFy = [shape_x(1) * sin(theta+cmd_sta) + shape_y(1) * cos(theta+cmd_sta) + fdy + v_y
shape_x(2) * sin(theta+cmd_sta) + shape_y(2) * cos(theta+cmd_sta) + fdy + v_y
shape_x(3) * sin(theta+cmd_sta) + shape_y(3) * cos(theta+cmd_sta) + fdy + v_y
shape_x(4) * sin(theta+cmd_sta) + shape_y(4) * cos(theta+cmd_sta) + fdy + v_y]; 

updatedRx = [shape_x(1) * cos(theta) - shape_y(1) * sin(theta) + rdx + v_x
shape_x(2) * cos(theta) - shape_y(2) * sin(theta) + rdx + v_x
shape_x(3) * cos(theta) - shape_y(3) * sin(theta) + rdx + v_x
shape_x(4) * cos(theta) - shape_y(4) * sin(theta) + rdx + v_x];
updatedRy = [shape_x(1) * sin(theta) + shape_y(1) * cos(theta) + rdy + v_y
shape_x(2) * sin(theta) + shape_y(2) * cos(theta) + rdy + v_y
shape_x(3) * sin(theta) + shape_y(3) * cos(theta) + rdy + v_y
shape_x(4) * sin(theta) + shape_y(4) * cos(theta) + rdy + v_y];


updatedVx = [shape_vx(1) * cos(theta) - shape_vy(1) * sin(theta) + v_x
shape_vx(2) * cos(theta) - shape_vy(2) * sin(theta) + v_x
shape_vx(3) * cos(theta) - shape_vy(3) * sin(theta) + v_x
shape_vx(4) * cos(theta) - shape_vy(4) * sin(theta) + v_x];
updatedVy = [shape_vx(1) * sin(theta) + shape_vy(1) * cos(theta) + v_y
shape_vx(2) * sin(theta) + shape_vy(2) * cos(theta) + v_y
shape_vx(3) * sin(theta) + shape_vy(3) * cos(theta) + v_y
shape_vx(4) * sin(theta) + shape_vy(4) * cos(theta) + v_y];
end

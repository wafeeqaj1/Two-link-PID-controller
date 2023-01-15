% This program simulates a double link manipulator with a torque at the
% pivot.

clear all; close all; clc;

% various physical parameters, packed into a single variable 'param'
param.m1 = 0.2; param.I1 = 1/12; param.L1 = 0.25; param.g = 9.81; 
param.m2 = 0.2; param.I2 = 1/12; param.L2 = 0.25; 

L1 = param.L1; L2 = param.L2;

%% Creating work Space
figure(1);
% Creating bounded circles for the graph input space
annulus = nsidedpoly(1000, 'Center', [0 0], 'Radius', L1 + L2);
hold on;
plot(annulus,'FaceColor', 'w')
plot(0,0,'ko'); % pivot
hold on;

%grpahing the two links on initial position
X1 = L1*cos(-pi/4);
X2 = X1 + L2*cos(-pi/4);
Y1 = L1*sin(-pi/4);
Y2 = Y1+ L2*sin(-pi/4);

hOA = plot([0 X1],[0 Y1],'b'); set(hOA,'linewidth',2); axis equal;
hold on;
hOB = plot([X1, X2],[Y1, Y2],'r'); set(hOB,'linewidth',2); axis equal;
xlim([-1.5*(L1+L2) 1.5*(L1+L2)]);
ylim([-1.5*(L1+L2) 1.5*(L1+L2)]);


%% user input
R = L1+L2;
title(['Press 1 points within the circle']);

% Grabbing a click input
[xpoint,ypoint] = ginput(1);

%checks the user did not select from outside the circle
while((xpoint>R)||(xpoint <-1*R) || (ypoint>R) || (ypoint <-1*R))
    title('Invalid point press again');
    [xpoint,ypoint] = ginput(1);
end 
hold on;

plot(xpoint,ypoint,'.k');     %plots the point
title('DONE');
hold on;
%% Calculating joint variable
r = sqrt(xpoint^2 + ypoint^2);
% Calculating theta 1
param.theta1desired = atan2(ypoint,xpoint) - acos((L2^2-L1^2-r^2)/(-2*L1*r));

% Calculating theta 2
param.theta2desired= pi - acos((r^2-L1^2-L2^2)/(-2*L1*L2));


%% ode function

% time duration of integration
tmax = 10; Numtimelist = 500;
timespan = linspace(0,tmax,Numtimelist)';

% initial conditions
theta1_0 = -pi/4; dtheta1_0 = 0;    ddtheta1_0 = 0;
theta2_0 = -pi/4; dtheta2_0 = 0;    ddtheta2_0 =0;
statevar0 = [theta1_0; dtheta1_0; theta2_0; dtheta2_0; ddtheta1_0; ddtheta2_0];

% options that determine accuracy of integration
options = odeset('reltol',1e-9,'abstol',1e-9);

[tlist, statevarlist] = ode45(@doublelinkodefile_PID,timespan,statevar0,options,param);
theta1list = statevarlist(:,1);
dtheta1list = statevarlist(:,2);
theta2list = statevarlist(:,3);
dtheta2list = statevarlist(:,4);


%% animation
for i = 1:1:Numtimelist
    xA = L1*cos(theta1list(i));
    yA = L1*sin(theta1list(i));
    xB = xA + L2*cos(theta1list(i)+theta2list(i));
    yB = yA + L2*sin(theta1list(i)+theta2list(i));
    set(hOA,'xdata',[0 xA],'ydata',[0 yA],'LineWidth',2);
    set(hOB,'xdata',[xA xB],'ydata',[yA yB],'LineWidth',2);
    pause(0.01);
    
end
title('Done');

%% response plot
figure(21); %plot for theta1
plot(tlist,theta1list, 'linewidth', 2); ylabel('\theta_1'); xlabel('t'); hold on;
figure(22); %plot for theta2
plot(tlist,theta2list, 'linewidth', 2); ylabel('\theta_2');  xlabel('t');hold on;
figure(23); %plot for dtheta1
plot(tlist,dtheta1list, 'linewidth', 2); ylabel('$\dot{\theta_1}$', 'Interpreter','latex'); xlabel('t'); hold on;
figure(24); %plot for dtheta2
plot(tlist,dtheta2list,'linewidth', 2); ylabel('$\dot{\theta_2}$', 'Interpreter','latex');  xlabel('t');
pause(3);

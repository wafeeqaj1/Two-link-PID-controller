function dstatevar = doublelinkodefile_PID(t,statevar,param)
%DOUBLELINKODEFILE with PID control

% unpacking the statevar vector
theta1 = statevar(1);
dtheta1 = statevar(2);
theta2 = statevar(3);
dtheta2 = statevar(4);
error1 = statevar(5);
error2 = statevar(6);

% unpacking the param variable for the physical parameters
g = param.g; m1 = param.m1; I1 = param.I1; m2 = param.m2; I2 = param.I2; 
L1 = param.L1; L2 = param.L2; theta1desired = param.theta1desired; 
theta2desired = param.theta2desired;

r1 = L1; r2 = L2;

%% Tune the controller
omega = 30;
zeta = 1;
kp=m1*omega^2;
kv =2*m1*omega*zeta;
ki = 0.1;
tau1 = -kp*(theta1-theta1desired)-kv*dtheta1 - ki*error1;
tau2 = -kp*(theta2-theta2desired)-kv*dtheta2 - ki*error2;

%% EOM Formulas
E = zeros(2,1);

E(1) = L1*m2*r2*sin(theta2)*dtheta1^2 - tau2 + g*m2*r2*cos(theta1 + theta2);
E(2) = -L1*m2*r2*sin(theta2)*dtheta2^2-2*L1*dtheta1*m2*r2*sin(theta2)*dtheta2 - tau1 + g*m2*r2*cos(theta1 + theta2) + L1*g*m2*cos(theta1) + g*m1*r1*cos(theta1);

%Mass Matrix
M = zeros(2,2);
M(1,1) = -m2*r2^2-L1*m2*cos(theta2)*r2-I2;
M(1,2) = -m2*r2^2-I2;
M(2,1) = -m2*L1^2-2*m2*cos(theta2)*L1*r2-m1*r1^2-m2*r2^2-I1-I2;
M(2,2) = -m2*r2^2-L1*m2*cos(theta2)*r2-I2;
C = M\E; 

ddtheta1 = C(1);
ddtheta2 = C(2);

derror1 = theta1 - theta1desired;
derror2 = theta2 - theta2desired;

dstatevar = [dtheta1; ddtheta1; dtheta2; ddtheta2; derror1;derror2];

end
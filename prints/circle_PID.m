%% draw_circle_PID.m
% PID-controlled circle trajectory for a 3-DOF Cartesian robot in CoppeliaSim

clc; clear; close all;

%% -----------------------------
% Connect to CoppeliaSim
% -----------------------------
client = RemoteAPIClient();
sim = client.getObject('sim');

disp('Connected to CoppeliaSim (ZeroMQ).');

%% -----------------------------
% Get joint handles
% -----------------------------
hX = sim.getObject('/Prismatic_joint_x');
hY = sim.getObject('/Prismatic_joint_y');
hZ = sim.getObject('/Prismatic_joint_z');

%% -----------------------------
% PID gains (adjust if needed)
% -----------------------------
Kp = 10;
Ki = 1;
Kd = 2;

% Integrators and last error
ix = 0; iy = 0; iz = 0;
ex_last = 0; ey_last = 0; ez_last = 0;

%% -----------------------------
% Trajectory params: CIRCLE
% -----------------------------
dt = 0.02;
T  = 12;
time = 0:dt:T;

r  = 0.10;      % radius
cx = 0.20;     % center X
cy = 0.20;     % center Y
z  = 0.10;     % drawing height

traj = [
    cx + r*cos(2*pi*(0:length(time)-1)/length(time))' , ...
    cy + r*sin(2*pi*(0:length(time)-1)/length(time))' , ...
    z * ones(length(time),1)
];

%% -----------------------------
% Start simulation
% -----------------------------
sim.startSimulation();
disp('Simulation started.');

pause(0.5); % let things stabilize

%% -----------------------------
% MAIN PID LOOP
% -----------------------------
errors = zeros(length(time),3); % for plotting later

for k = 1:length(time)

    % Desired positions
    x_des = traj(k,1);
    y_des = traj(k,2);
    z_des = traj(k,3);

    % Actual positions (feedback)
    x_act = sim.getJointPosition(hX);
    y_act = sim.getJointPosition(hY);
    z_act = sim.getJointPosition(hZ);

    % Errors
    ex = x_des - x_act;
    ey = y_des - y_act;
    ez = z_des - z_act;

    % Integrals
    ix = ix + ex*dt;
    iy = iy + ey*dt;
    iz = iz + ez*dt;

    % Derivatives
    dex = (ex - ex_last)/dt;
    dey = (ey - ey_last)/dt;
    dez = (ez - ez_last)/dt;

    ex_last = ex; 
    ey_last = ey; 
    ez_last = ez;

    % PID outputs (velocities)
    uX = Kp*ex + Ki*ix + Kd*dex;
    uY = Kp*ey + Ki*iy + Kd*dey;
    uZ = Kp*ez + Ki*iz + Kd*dez;

    % Limit velocity to prevent jitter
    uX = max(min(uX, 0.5), -0.5);
    uY = max(min(uY, 0.5), -0.5);
    uZ = max(min(uZ, 0.5), -0.5);

    % Send velocity commands
    sim.setJointTargetVelocity(hX, uX);
    sim.setJointTargetVelocity(hY, uY);
    sim.setJointTargetVelocity(hZ, uZ);

    % Save errors
    errors(k,:) = [ex ey ez];

    pause(dt);
end

%% -----------------------------
% Stop simulation
% -----------------------------
sim.stopSimulation();
disp('Simulation stopped.');

%% -----------------------------
% Plot tracking error
% -----------------------------
figure;
plot(time, errors(:,1), 'r', 'LineWidth',1.2); hold on;
plot(time, errors(:,2), 'g', 'LineWidth',1.2);
plot(time, errors(:,3), 'b', 'LineWidth',1.2);
xlabel('Time (s)');
ylabel('Error (m)');
title('PID Tracking Error (X,Y,Z)');
legend('X error','Y error','Z error');
grid on;

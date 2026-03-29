%% FINAL WORKING: CoppeliaSim 3-DOF Cartesian Robot Control (ZeroMQ API)
% Uses your exact object names: /Prismatic_joint_x, /Prismatic_joint_y, /Prismatic_joint_z

clc; clear; close all;

% -----------------------------
% Connect to CoppeliaSim
% -----------------------------
client = RemoteAPIClient();     % requires modern API folder added to MATLAB path
sim = client.getObject('sim');

disp('Connected to CoppeliaSim using ZeroMQ Remote API.');

% -----------------------------
% Get object handles
% -----------------------------
hX  = sim.getObject('/Prismatic_joint_x');
hY  = sim.getObject('/Prismatic_joint_y');
hZ  = sim.getObject('/Prismatic_joint_z');

% If you have a pen tip, uncomment and edit the name:
% hPen = sim.getObject('/pen_tip');

disp('Handles acquired successfully.');

% -----------------------------
% Start simulation
% -----------------------------
sim.startSimulation();
disp('Simulation started.');

% -----------------------------
% Generate a square trajectory
% -----------------------------
dt = 0.02;
T  = 12;
time = 0:dt:T;

P1 = [0.10 0.10 0.10];
P2 = [0.30 0.10 0.10];
P3 = [0.30 0.30 0.10];
P4 = [0.10 0.30 0.10];

Nedge = floor(length(time)/4);
traj = [ ...
    [linspace(P1(1),P2(1),Nedge)' linspace(P1(2),P2(2),Nedge)' linspace(P1(3),P2(3),Nedge)']; ...
    [linspace(P2(1),P3(1),Nedge)' linspace(P2(2),P3(2),Nedge)' linspace(P2(3),P3(3),Nedge)']; ...
    [linspace(P3(1),P4(1),Nedge)' linspace(P3(2),P4(2),Nedge)' linspace(P3(3),P4(3),Nedge)']; ...
    [linspace(P4(1),P1(1),Nedge)' linspace(P4(2),P1(2),Nedge)' linspace(P4(3),P1(3),Nedge)']; ...
];

if size(traj,1) < length(time)
    traj(end+1:length(time),:) = repmat(traj(end,:), length(time)-size(traj,1), 1);
else
    traj = traj(1:length(time),:);
end

% -----------------------------
% Stream to CoppeliaSim
% -----------------------------
disp('Sending trajectory to robot...');

for k = 1:length(time)
    px = traj(k,1);
    py = traj(k,2);
    pz = traj(k,3);

    sim.setJointTargetPosition(hX, px);
    sim.setJointTargetPosition(hY, py);
    sim.setJointTargetPosition(hZ, pz);

    pause(dt);
end

disp('Trajectory completed.');

% -----------------------------
% Stop simulation
% -----------------------------
sim.stopSimulation();
disp('Simulation stopped.');

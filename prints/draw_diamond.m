%% draw_diamond.m
% Draws a diamond shape with pen-up/down travel moves

clc; clear; close all;

%% -----------------------------
% Connect to CoppeliaSim
% -----------------------------
client = RemoteAPIClient();
sim = client.getObject('sim');

disp('Connected to CoppeliaSim.');

%% -----------------------------
% Get prismatic joints
% -----------------------------
safeRetract = [0.35, 0.05];   % far safe spot
Zfar = 0.30;                  % far high retract
hX = sim.getObject('/Prismatic_joint_x');
hY = sim.getObject('/Prismatic_joint_y');
hZ = sim.getObject('/Prismatic_joint_z');

%% -----------------------------
% Start simulation
% -----------------------------
sim.startSimulation();
disp('Simulation started.');

%% -----------------------------
% Motion Parameters
% -----------------------------
Zdown = 0.10;      % pen touches surface
Zup   = 0.25;      % pen lifted
dt    = 0.02;
steps = 40;

home  = [0.10, 0.10];  % home location (bottom-left)

cx = 0.20;             % center of pattern
cy = 0.20;
r  = 0.10;

% Diamond corners
P1 = [cx,     cy + r];   % Top
P2 = [cx + r, cy    ];   % Right
P3 = [cx,     cy - r];   % Bottom
P4 = [cx - r, cy    ];   % Left

%% -----------------------------
% Smooth movement helper
% -----------------------------
function smoothMove(sim, hX, hY, hZ, Pstart, Pend, Z, steps, dt)
    xs = linspace(Pstart(1), Pend(1), steps);
    ys = linspace(Pstart(2), Pend(2), steps);
    for i = 1:steps
        sim.setJointTargetPosition(hX, xs(i));
        sim.setJointTargetPosition(hY, ys(i));
        sim.setJointTargetPosition(hZ, Z);
        pause(dt);
    end
end

%% -----------------------------
% COMPLETE MOTION SEQUENCE
% -----------------------------

%% 1. Pen Up
sim.setJointTargetPosition(hZ, Zup);
pause(0.5);

%% 2. Go to home
smoothMove(sim, hX, hY, hZ, [0,0], home, Zup, steps, dt);

%% 3. Move to pattern start (P1) with pen up
smoothMove(sim, hX, hY, hZ, home, P1, Zup, steps, dt);

%% 4. Pen Down
sim.setJointTargetPosition(hZ, Zdown);
pause(0.3);

%% 5. Draw the diamond
smoothMove(sim, hX, hY, hZ, P1, P2, Zdown, steps, dt);
smoothMove(sim, hX, hY, hZ, P2, P3, Zdown, steps, dt);
smoothMove(sim, hX, hY, hZ, P3, P4, Zdown, steps, dt);
smoothMove(sim, hX, hY, hZ, P4, P1, Zdown, steps, dt);

%% 6. Pen Up (basic lift)
sim.setJointTargetPosition(hZ, Zup);
pause(0.2);

%% 7. Retract to FAR SIDE (big lift + travel)
% Lift high to avoid obstacles
sim.setJointTargetPosition(hZ, Zfar);
pause(0.3);

% Move far away to the "other side"
smoothMove(sim, hX, hY, hZ, P1, safeRetract, Zfar, steps, dt);

%% 8. Return home from the far side
smoothMove(sim, hX, hY, hZ, safeRetract, home, Zfar, steps, dt);

%% 9. Lower back to normal Z-up
sim.setJointTargetPosition(hZ, Zup);
pause(0.2);

disp('Diamond drawing complete.');

%% -----------------------------
% Stop simulation
% -----------------------------
sim.stopSimulation();
disp('Simulation stopped.');

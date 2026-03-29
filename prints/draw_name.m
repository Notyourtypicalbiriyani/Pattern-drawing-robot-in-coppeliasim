%% draw_name.m
% Writes a NAME using pen-up/down motion on a Cartesian (PPP) robot

clc; clear; close all;

%% -----------------------------
% Connect to CoppeliaSim
% -----------------------------
client = RemoteAPIClient();
sim = client.getObject('sim');
disp('Connected to CoppeliaSim.');

%% -----------------------------
% Get joint handles
% -----------------------------
hX = sim.getObject('/Prismatic_joint_x');
hY = sim.getObject('/Prismatic_joint_y');
hZ = sim.getObject('/Prismatic_joint_z');

%% -----------------------------
% Simulation start
% -----------------------------
sim.startSimulation();
disp('Simulation started.');

%% Pen heights
Zdown = 0.10;   % pen touching
Zup   = 0.20;   % pen lifted

%% Letter spacing
letterSpace = 0.15;
baseX = 0.10;
baseY = 0.10;

%% -----------------------------
% WRITE LETTER A
% -----------------------------
x0 = baseX;
y0 = baseY;

moveRobot(sim, hX, hY, hZ, x0, y0, Zup);

moveRobot(sim, hX, hY, hZ, x0, y0, Zdown);
moveRobot(sim, hX, hY, hZ, x0+0.05, y0+0.15, Zdown);
moveRobot(sim, hX, hY, hZ, x0+0.10, y0, Zdown);

moveRobot(sim, hX, hY, hZ, x0+0.025, y0+0.075, Zdown);
moveRobot(sim, hX, hY, hZ, x0+0.075, y0+0.075, Zdown);

moveRobot(sim, hX, hY, hZ, x0+0.10, y0, Zup);

%% -----------------------------
% WRITE LETTER D
% -----------------------------
x0 = baseX + letterSpace;

moveRobot(sim, hX, hY, hZ, x0, y0, Zup);
moveRobot(sim, hX, hY, hZ, x0, y0, Zdown);
moveRobot(sim, hX, hY, hZ, x0, y0+0.15, Zdown);

steps = linspace(0, pi, 40);
for t = steps
    moveRobot(sim, hX, hY, hZ, x0+0.07*cos(t), y0+0.075+0.075*sin(t), Zdown);
end

moveRobot(sim, hX, hY, hZ, x0+0.07, y0+0.075, Zup);

%% -----------------------------
% WRITE LETTER I
% -----------------------------
x0 = baseX + 2*letterSpace;

moveRobot(sim, hX, hY, hZ, x0, y0, Zup);
moveRobot(sim, hX, hY, hZ, x0, y0, Zdown);
moveRobot(sim, hX, hY, hZ, x0, y0+0.15, Zdown);
moveRobot(sim, hX, hY, hZ, x0, y0+0.15, Zup);

%% -----------------------------
% WRITE LETTER L
% -----------------------------
x0 = baseX + 3*letterSpace;

moveRobot(sim, hX, hY, hZ, x0, y0+0.15, Zup);
moveRobot(sim, hX, hY, hZ, x0, y0+0.15, Zdown);
moveRobot(sim, hX, hY, hZ, x0, y0, Zdown);
moveRobot(sim, hX, hY, hZ, x0+0.10, y0, Zdown);
moveRobot(sim, hX, hY, hZ, x0+0.10, y0, Zup);

%% -----------------------------
% Finish
% -----------------------------
disp('NAME DRAWING COMPLETE.');
sim.stopSimulation();
disp('Simulation stopped.');

%% -----------------------------
% Helper function
% -----------------------------
function moveRobot(sim, hX, hY, hZ, x, y, z)
    sim.setJointTargetPosition(hX, x);
    sim.setJointTargetPosition(hY, y);
    sim.setJointTargetPosition(hZ, z);
    pause(0.02);
end

function smoothMove(sim, hX, hY, hZ, x1, y1, z1, x2, y2, z2, steps)

    xs = linspace(x1, x2, steps);
    ys = linspace(y1, y2, steps);
    zs = linspace(z1, z2, steps);

    for i = 1:steps
        sim.setJointTargetPosition(hX, xs(i));
        sim.setJointTargetPosition(hY, ys(i));
        sim.setJointTargetPosition(hZ, zs(i));
        pause(0.02);   % slow enough to see clearly
    end
end

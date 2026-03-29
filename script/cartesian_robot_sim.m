%% FINAL ENHANCED 3-DOF CARTESIAN ROBOT SCRIPT
% ZeroMQ API – Safe Trajectory, Pen Control, Inverted Bot Support
clc; clear; close all;

%% ---------- USER CONFIG ----------
jointNames = {'/Prismatic_joint_x','/Prismatic_joint_y','/Prismatic_joint_z'}; 
penHandleName = '/pen_tip';                % optional
usePenHandle  = false;                      % false = use signals only
penDownSignalName = 'penDown';
inkFlowSignalName = 'inkFlow';

invertBot = false;                          % true = flip Z
axisMap = [1 2 3];                          % reorder if needed

dt = 0.02;                                  % control timestep (s)
T_total = 12;                               % total time (s)

maxSpeed = 0.15;                            % m/s
maxStep = maxSpeed * dt;                    % per-step displacement limit

jointLimits = [0.0 0.5;                     % X limit
               0.0 0.5;                     % Y limit
               0.05 0.3];                   % Z limit

enablePlots = true;

%% ---------- CONNECT TO SIM ----------
client = RemoteAPIClient();
sim = client.getObject('sim');

disp('Connected to CoppeliaSim (ZeroMQ).');

% Joint handles
nq = numel(jointNames);
hJ = zeros(1,nq);
for i=1:nq
    hJ(i) = sim.getObject(jointNames{i});
end

%% Pen handle check
if usePenHandle
    try
        hPen = sim.getObject(penHandleName);
        disp(['Pen handle found: ' penHandleName]);
    catch
        warning('Pen object not found. Using signal-based pen control.');
        usePenHandle = false;
    end
end

%% ---------- START SIMULATION ----------
sim.startSimulation();
disp('Simulation started.');

%% ---------- TRAJECTORY GENERATION ----------
tvec = 0:dt:T_total;
N = length(tvec);

% Square corners (XY) with fixed Z
P1 = [0.10 0.10 0.10];
P2 = [0.30 0.10 0.10];
P3 = [0.30 0.30 0.10];
P4 = [0.10 0.30 0.10];

edgePts = ceil(N/4);

makeEdge = @(A,B,M) [linspace(A(1),B(1),M)' linspace(A(2),B(2),M)' linspace(A(3),B(3),M)'];

traj = [ makeEdge(P1,P2,edgePts);
         makeEdge(P2,P3,edgePts);
         makeEdge(P3,P4,edgePts);
         makeEdge(P4,P1,edgePts) ];

% Resize safely
if size(traj,1) < N
    traj(end+1:N,:) = repmat(traj(end,:), N - size(traj,1), 1);
else
    traj = traj(1:N,:);
end

% Add brief corner Z-lift
liftHeight = 0.02;
for e = 1:4
    idxStart = (e-1)*edgePts + round(0.9*edgePts);
    idxEnd   = min(e*edgePts, size(traj,1));
    if idxStart < idxEnd
        traj(idxStart:idxEnd,3) = traj(idxStart:idxEnd,3) + liftHeight;
    end
end

% Invert Z if needed
if invertBot
    traj(:,3) = -traj(:,3);
end

% Axis remap
traj = traj(:, axisMap);

% Joint limits clamp
for i=1:nq
    traj(:,i) = max(traj(:,i), jointLimits(i,1));
    traj(:,i) = min(traj(:,i), jointLimits(i,2));
end

%% ---------- PEN CONTROL FUNCTION ----------
function setPen(sim, useHandle, penDownSignalName, inkFlowSignalName, down, flow)
    sim.setIntegerSignal(penDownSignalName, int32(down));
    sim.setFloatSignal(inkFlowSignalName, double(flow));
end

%% ---------- EXECUTION ----------
disp('Executing trajectory...');

log_t = []; log_q = []; log_qdes = []; log_delta = [];

prevCmd = zeros(1,nq);
lastPenState = -1;

try
    % Initial pen up
    setPen(sim,usePenHandle,penDownSignalName,inkFlowSignalName,0,0);

    for k = 1:N
        t = tvec(k);
        qdes = traj(k,:);

        % Read current joint positions
        qcur = zeros(1,nq);
        for i=1:nq
            qcur(i) = sim.getJointPosition(hJ(i));
        end

        % Compute limited step
        delta = qdes - qcur;
        delta = max(-maxStep, min(maxStep, delta));
        cmd = qcur + delta;

        % Safety clamp
        for i=1:nq
            cmd(i) = max(jointLimits(i,1), min(jointLimits(i,2), cmd(i)));
        end

        % Send commands
        for i=1:nq
            sim.setJointTargetPosition(hJ(i), cmd(i));
        end

        % PEN LOGIC: pen down when Z is within drawing height
        zpos = qdes(3);
        drawHeight = 0.11;
        penDown = double(zpos <= drawHeight);
        flow = 0.6 * penDown; % constant 60% flow when drawing

        if penDown ~= lastPenState
            setPen(sim,usePenHandle,penDownSignalName,inkFlowSignalName,penDown,flow);
            lastPenState = penDown;
        end

        % Logging
        log_t(end+1) = t;
        log_q(end+1,:) = qcur;
        log_qdes(end+1,:) = qdes;
        log_delta(end+1,:) = delta;

        pause(dt);
    end

    % Final pen up
    setPen(sim,usePenHandle,penDownSignalName,inkFlowSignalName,0,0);

catch ME
    warning('Error during execution: %s', ME.message);
    setPen(sim,usePenHandle,penDownSignalName,inkFlowSignalName,0,0);
end

disp('Trajectory complete.');

%% ---------- STOP SIM ----------
sim.stopSimulation();
disp('Simulation stopped.');

%% ---------- PLOTS ----------
if enablePlots
    figure('Name','Joint Tracking');
    for i=1:nq
        subplot(nq,1,i);
        plot(log_t, log_q(:,i),'b'); hold on;
        plot(log_t, log_qdes(:,i),'r--');
        xlabel('Time'); ylabel(['Joint ' num2str(i)]);
        legend('Actual','Desired');
    end

    figure('Name','Command Delta');
    plot(log_t, vecnorm(log_delta,2,2));
    xlabel('Time'); ylabel('Step magnitude');
end

disp('Done.');

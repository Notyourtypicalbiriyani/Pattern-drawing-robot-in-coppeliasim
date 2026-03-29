% cartesian_3dof_draw.m
% Minimal 3-DOF Cartesian robot simulation for pattern drawing (circle).
% Save as cartesian_3dof_draw.m and run in MATLAB.
clear; close all; clc;

%% Parameters (tweak if needed)
m = [1.0, 1.0, 0.5];   % masses for X, Y, Z axes (kg)
b = [5.0, 5.0, 2.0];   % damping coefficients (N s/m)
dt = 0.005;            % simulation timestep (s)
Tsim = 8;              % total sim time (s)
time = 0:dt:Tsim;

% PD gains (start here; tune if needed)
Kp = [120, 120, 80];
Kd = [20, 20, 10];
Ki = [0,0,0];          % set nonzero if you want integral action (small)

% Trajectory choice: 'circle' or 'letterA'
traj_type = 'letterA';

%% Reference trajectory (x_ref(t), y_ref(t), z_ref(t))
xc = 0; yc = 0; zc = 0.2;   % center for circle
r = 0.08;                   % radius (meters)
omega = 2*pi / Tsim * 2;    % rotate a few times across sim

x_ref = zeros(3, numel(time));
switch traj_type
    case 'circle'
        x_ref(1,:) = xc + r * cos(omega * time);         % X
        x_ref(2,:) = yc + r * sin(omega * time);         % Y
        x_ref(3,:) = zc + 0.02 * sin(2*omega * time);    % small Z wiggle
    case 'letterA'
        % Simple piecewise letter-like shape (quick approx)
        % here we create 3 line segments; this is illustrative
        % Key points of the "A" (in meters)
        P1 = [-0.05; 0.00; 0.15];   % bottom-left
        P2 = [ 0.00; 0.12; 0.15];   % top-middle
        P3 = [ 0.05; 0.00; 0.15];   % bottom-right
        P4 = [-0.02; 0.06; 0.15];   % crossbar left
        P5 = [ 0.02; 0.06; 0.15];   % crossbar right

        % Split time for strokes
        N = numel(time);
        N1 = floor(N*0.33);   % stroke 1
        N2 = floor(N*0.33);   % stroke 2
        N3 = N - N1 - N2;     % stroke 3

        % Stroke 1: P1 -> P2
        S1 = [linspace(P1(1),P2(1),N1);
           linspace(P1(2),P2(2),N1);
           linspace(P1(3),P2(3),N1)];

        % Stroke 2: P2 -> P3
        S2 = [linspace(P2(1),P3(1),N2);
              linspace(P2(2),P3(2),N2);
              linspace(P2(3),P3(3),N2)];

        % Stroke 3: P4 -> P5 (crossbar)
        S3 = [linspace(P4(1),P5(1),N3);
              linspace(P4(2),P5(2),N3);
              linspace(P4(3),P5(3),N3)];
    
        x_ref = [S1, S2, S3];
    otherwise
        error('Unknown trajectory type');
end

%% State initialization
x = zeros(3,1);    % positions
xd = zeros(3,1);   % velocities
xi = zeros(3,1);   % integrator states if used

% logs
Xlog = zeros(3, numel(time));
Ulog = zeros(3, numel(time));

%% Simple Euler integration loop
for k = 1:numel(time)
    % reference at current time
    xr = x_ref(:,k);
    % derivative of reference (finite diff)
    if k == 1
        xrd = zeros(3,1);
    else
        xrd = (x_ref(:,k) - x_ref(:,k-1)) / dt;
    end
    
    % control law per axis (PID -> here PD + optional I)
    e = xr - x;
    ed = xrd - xd;
    xi = xi + e * dt;
    u = Kp'.*e + Kd'.*ed + Ki'.*xi;   % control force/effort for each axis
    
    % dynamics: m * xdd + b * xd = u  => xdd = (u - b*xd)/m
    xdd = (u - b'.*xd) ./ m';
    
    % integrate Euler
    xd = xd + xdd * dt;
    x = x + xd * dt;
    
    % log
    Xlog(:,k) = x;
    Ulog(:,k) = u;
end

%% Compute tracking error metrics
err = x_ref - Xlog;
rmse = sqrt(mean(err.^2, 2));
fprintf('RMSE X: %.4f m, Y: %.4f m, Z: %.4f m\n', rmse(1), rmse(2), rmse(3));

%% Plots (positions vs ref)
figure('Units','normalized','Position',[0.1 0.15 0.8 0.7]);
subplot(3,2,1);
plot(time, x_ref(1,:), 'k--', 'LineWidth', 1.2); hold on;
plot(time, Xlog(1,:), 'LineWidth', 1.2);
title('X position'); xlabel('t (s)'); ylabel('m'); legend('ref','actual');

subplot(3,2,3);
plot(time, x_ref(2,:), 'k--','LineWidth',1.2); hold on;
plot(time, Xlog(2,:),'LineWidth',1.2);
title('Y position'); xlabel('t (s)'); ylabel('m'); legend('ref','actual');

subplot(3,2,5);
plot(time, x_ref(3,:), 'k--','LineWidth',1.2); hold on;
plot(time, Xlog(3,:),'LineWidth',1.2);
title('Z position'); xlabel('t (s)'); ylabel('m'); legend('ref','actual');

% XY drawing (path)
subplot(1,2,2);
plot(x_ref(1,:), x_ref(2,:), 'k--','LineWidth',1.2); hold on;
plot(Xlog(1,:), Xlog(2,:), 'LineWidth', 1.4);
axis equal; grid on;
title('XY drawing path'); xlabel('X (m)'); ylabel('Y (m)');
legend('ref','actual','Location','Best');

% Save figure to PNG for PPT
outimg = 'results_cartesian_path.png';
saveas(gcf, outimg);
fprintf('Saved figure: %s\n', outimg);

%% Optional simple animation: draw pen path
figure('Name','Animation','Units','normalized','Position',[0.2 0.2 0.45 0.5]);
axis equal; grid on;
xlim([min([x_ref(1,:), Xlog(1,:)]) - 0.05, max([x_ref(1,:), Xlog(1,:)]) + 0.05]);
ylim([min([x_ref(2,:), Xlog(2,:)]) - 0.05, max([x_ref(2,:), Xlog(2,:)]) + 0.05]);
hold on;
plot(x_ref(1,:), x_ref(2,:), 'k:','LineWidth',1);
h_ref = plot(x_ref(1,1), x_ref(2,1), 'ko','MarkerFaceColor','k');
h_actual = plot(Xlog(1,1), Xlog(2,1), 'ro','MarkerFaceColor','r');
for k = 1:5:numel(time)
    set(h_ref,'XData',x_ref(1,k),'YData',x_ref(2,k));
    set(h_actual,'XData',Xlog(1,k),'YData',Xlog(2,k));
    drawnow;
end

%% Save simulation data (optional)
save('sim_cartesian_3dof.mat','time','x_ref','Xlog','Ulog','rmse');
fprintf('Saved data sim_cartesian_3dof.mat\n');

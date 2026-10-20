%% ME 4879 - Part 5: Singularity-Safe Wall-Painting Trajectory + Masked IK
% Fixes:
%  (1) mask is 6-element [x y z rx ry rz], not scalar [1] -> position-only IK
%  (2) wall pulled in / Y-sweep shrunk so max required reach stays < 0.400 m

clear; clc; close all;

%% --- Robot (same as workspace script) ---
L(1) = Link('d', 0.150, 'a', 0,     'alpha', pi/2, 'standard');
L(2) = Link('d', 0,     'a', 0.200, 'alpha', 0,    'standard');
L(3) = Link('d', 0,     'a', 0.200, 'alpha', 0,    'standard');
L(4) = Link('d', 0.100, 'a', 0,     'alpha', 0,    'standard');
robot = SerialLink(L, 'name', 'PaintBot4R');

%% --- SAFE trajectory parameters ---
% Original (singular): X = 0.25, Y in [-0.25, 0.25], Z in [0.20, 0.35]
%   -> worst-case reach at (Y=+/-0.25, Z=0.35): sqrt(0.25^2+0.25^2+0.20^2)
%      = 0.406 m > a2+a3 = 0.400 m  -> boundary singularity (theta3 -> 0)
%
% Fix applied here: pull wall to X = 0.20 m AND shrink Y-sweep to +/-0.18 m
X_wall   = 0.20;
Y_amp    = 0.18;
Z_low    = 0.20;
Z_high   = 0.35;
d1       = 0.150;
reach_max = 0.200 + 0.200;   % a2 + a3

% Worst-case check (corner point) before running anything
worst_case_reach = sqrt(X_wall^2 + Y_amp^2 + (Z_high - d1)^2);
fprintf('Worst-case required reach: %.4f m  (limit: %.4f m, margin: %.4f m)\n', ...
        worst_case_reach, reach_max, reach_max - worst_case_reach);
assert(worst_case_reach < reach_max, 'Trajectory still exceeds a2+a3 - shrink further.');

%% --- Build the trajectory (sinusoidal Y sweep, stepped Z) ---
t = linspace(0, 10, 200)';
Y = Y_amp * sin(2*pi*0.2*t);              % smooth sinusoidal sweep
Z = Z_low + (Z_high - Z_low) * (t / t(end));  % linear step/ramp 0.20 -> 0.35
X = X_wall * ones(size(t));

%% --- Masked inverse kinematics (position-only, mask fixes the overconstraint) ---
mask = [1 1 1 0 0 0];      % track x,y,z ; ignore roll,pitch,yaw (4-DoF can't set them)
q = zeros(numel(t), 4);
q_guess = zeros(1,4);       % warm-start IK from previous solution each step

for k = 1:numel(t)
    T_target = transl(X(k), Y(k), Z(k));   % orientation left as identity - masked out anyway
    q(k,:) = robot.ikine(T_target, 'q0', q_guess, 'mask', mask);
    q_guess = q(k,:);       % warm start next solve
end

%% --- Verify: forward kinematics of the solved trajectory ---
xyz_check = zeros(numel(t), 3);
for k = 1:numel(t)
    Tk = robot.fkine(q(k,:));
    xyz_check(k,:) = Tk.t';
end
pos_error = vecnorm(xyz_check - [X Y Z], 2, 2);
fprintf('Max position tracking error: %.6f m\n', max(pos_error));

%% --- Plot commanded vs. achieved path ---
figure('Color','w');
plot3(X, Y, Z, 'b-', 'LineWidth', 1.5); hold on;
plot3(xyz_check(:,1), xyz_check(:,2), xyz_check(:,3), 'r--', 'LineWidth', 1.2);
legend('Commanded wall path', 'Achieved (fkine of IK solution)');
axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Singularity-Safe Trajectory Tracking Validation');
view(135, 20);
%% ME 4879: Automated Singularity-Aware Workspace Visualization (Task 4)
% Programmatic Monte-Carlo Sweep identifying and mapping Singularity Zones
clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  TASK 4: AUTOMATED WORKSPACE SINGULARITY ZONE MAPPING          \n');
fprintf('=================================================================\n\n');

%% Step 1: Verify and Load the Robot Model
if ~exist('DOF4_Arm', 'var')
    fprintf('Loading "DOF4_Arm" using robot_tree.m...\n');
    try
        robot_tree; % Run your tree loader script
    catch
        error('Could not load robot_tree.m. Please make sure you are in the correct MATLAB folder.');
    end
end

% Set the end-effector body name (matching your Simulink Get Transform block)
endEffectorName = 'Body5'; 

%% Step 2: Dynamically Extract Joint Limits
jointLimits = [];
configTemplate = homeConfiguration(DOF4_Arm); % Get configuration struct template

activeJointIdx = 1;
for i = 1:length(DOF4_Arm.BodyNames)
    body = getBody(DOF4_Arm, DOF4_Arm.BodyNames{i});
    joint = body.Joint;
    
    if ~strcmp(joint.Type, 'fixed')
        limits = joint.PositionLimits;
        if isempty(limits) || any(isinf(limits))
            limits = [-pi, pi]; 
        end
        jointLimits(activeJointIdx, :) = limits;
        activeJointIdx = activeJointIdx + 1;
    end
end
numActiveJoints = activeJointIdx - 1;

%% Step 3: Run the Monte-Carlo Sampling with Jacobian Computation
numSamples = 20000; % High-density sampling
fprintf('Generating %d configurations and analyzing their Jacobians...\n', numSamples);

x_pts = zeros(numSamples, 1);
y_pts = zeros(numSamples, 1);
z_pts = zeros(numSamples, 1);
w_pts = zeros(numSamples, 1); % Manipulability index array

h_wait = waitbar(0, 'Computing Jacobians & Mapping Singularities...');

for i = 1:numSamples
    % Generate random joint values
    q_random = zeros(numActiveJoints, 1);
    for j = 1:numActiveJoints
        min_limit = jointLimits(j, 1);
        max_limit = jointLimits(j, 2);
        q_random(j) = min_limit + (max_limit - min_limit) * rand();
    end
    
    % Map to configuration structure
    config = configTemplate;
    for j = 1:numActiveJoints
        config(j).JointPosition = q_random(j);
    end
    
    % Forward Kinematics Transform
    tform = getTransform(DOF4_Arm, config, endEffectorName, 'Base');
    x_pts(i) = tform(1, 4);
    y_pts(i) = tform(2, 4);
    z_pts(i) = tform(3, 4);
    
    % Extract the 3x4 positional Jacobian from the robot tree
    % (jacobian at the end-effector with respect to base)
    J_full = geometricJacobian(DOF4_Arm, config, endEffectorName);
    J_pos = J_full(4:6, :); % Isolation of linear velocity (3x4)
    
    % Calculate manipulability index: w = sqrt(det(Jp * Jp'))
    w_pts(i) = sqrt(abs(det(J_pos * J_pos')));
    
    if mod(i, 2000) == 0
        waitbar(i/numSamples, h_wait);
    end
end
close(h_wait);
fprintf('Kinematic sweep and singularity mapping complete!\n');

%% Step 4: Separate Safe vs. Singular Zones
% We use a threshold of w < 0.001 to classify critical singularity zones
% (Due to the link lengths of 0.20m, the max possible w is ~0.025)
critical_threshold = 0.001; 
singular_idx = w_pts < critical_threshold;
safe_idx = w_pts >= critical_threshold;

fprintf('\n=== STATISTICAL ANALYSIS OF WORKSPACE ===\n');
fprintf('Safe Points (Dexterous Workspace): %d (%.2f%%)\n', sum(safe_idx), sum(safe_idx)/numSamples*100);
fprintf('Singular Points (w < %.4f):    %d (%.2f%%)\n', critical_threshold, sum(singular_idx), sum(singular_idx)/numSamples*100);

%% Step 5: Plotting the 3D and 2D Projections
fig = figure('Name', '4-DoF Manipulator Workspace Singularity Map', 'Position', [50, 50, 1200, 900], 'Color', 'w');

% Plot 1: 3D Visualization of Singularity Zones
subplot(2, 2, 1);
scatter3(x_pts(safe_idx), y_pts(safe_idx), z_pts(safe_idx), 2, 'b', 'filled', 'MarkerFaceAlpha', 0.05);
hold on;
scatter3(x_pts(singular_idx), y_pts(singular_idx), z_pts(singular_idx), 4, 'r', 'filled', 'MarkerFaceAlpha', 0.25);
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'LineWidth', 3);
grid on;
axis equal;
xlabel('X (meters)', 'FontWeight', 'bold');
ylabel('Y (meters)', 'FontWeight', 'bold');
zlabel('Z (meters)', 'FontWeight', 'bold');
title('3D Workspace: Dexterous (Blue) vs Singular (Red)', 'FontSize', 11);
legend('Dexterous (w \geq 0.001)', 'Singular (w < 0.001)', 'Base Origin', 'Location', 'best');
view(45, 20);

% Plot 2: X-Y Top-Down View
subplot(2, 2, 2);
scatter(x_pts(safe_idx), y_pts(safe_idx), 2, 'b', 'filled', 'MarkerFaceAlpha', 0.05);
hold on;
scatter(x_pts(singular_idx), y_pts(singular_idx), 4, 'r', 'filled', 'MarkerFaceAlpha', 0.25);
plot(0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('X (meters)', 'FontWeight', 'bold');
ylabel('Y (meters)', 'FontWeight', 'bold');
title('X-Y Top-Down: Central Shoulder Column Singularity', 'FontSize', 11);

% Plot 3: X-Z Side View
subplot(2, 2, 3);
scatter(x_pts(safe_idx), z_pts(safe_idx), 2, 'b', 'filled', 'MarkerFaceAlpha', 0.05);
hold on;
scatter(x_pts(singular_idx), z_pts(singular_idx), 4, 'r', 'filled', 'MarkerFaceAlpha', 0.25);
plot(0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('X (meters)', 'FontWeight', 'bold');
ylabel('Z (meters)', 'FontWeight', 'bold');
title('X-Z Side Profile: Horizontal and Outer Edge Boundaries', 'FontSize', 11);

% Plot 4: Y-Z Front View
subplot(2, 2, 4);
scatter(y_pts(safe_idx), z_pts(safe_idx), 2, 'b', 'filled', 'MarkerFaceAlpha', 0.05);
hold on;
scatter(y_pts(singular_idx), z_pts(singular_idx), 4, 'r', 'filled', 'MarkerFaceAlpha', 0.25);
plot(0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('Y (meters)', 'FontWeight', 'bold');
ylabel('Z (meters)', 'FontWeight', 'bold');
title('Y-Z Front Profile: Radial Boundary Singularity Shell', 'FontSize', 11);

sgtitle('4-DoF Manipulator Workspace Singularity Map', 'FontSize', 14, 'FontWeight', 'bold');

%% Step 6: Identify and Report Key Singular Positions
% Find specific coordinates representing unique physical states
[~, center_idx] = min(x_pts.^2 + y_pts.^2); % Near base axis (Shoulder Singularity)
[~, max_reach_idx] = max(sqrt(x_pts.^2 + y_pts.^2 + (z_pts-0.150).^2)); % Boundary reach

% Extract some singular coordinates for the report table
singular_points = find(singular_idx);
table_rows = min(5, length(singular_points));

fprintf('\n=================================================================\n');
fprintf('     SAMPLE PHYSICAL COORDINATES OF SINGULAR CONFIGURATIONS      \n');
fprintf('=================================================================\n');
fprintf('%-10s | %-10s | %-10s | %-15s | %-20s\n', 'X (m)', 'Y (m)', 'Z (m)', 'w (Metric)', 'Singularity Type');
fprintf('%s\n', repmat('-', 1, 75));

% Shoulder Singularity Sample
fprintf('%-10.3f | %-10.3f | %-10.3f | %-15.6f | %-20s\n', ...
    x_pts(center_idx), y_pts(center_idx), z_pts(center_idx), w_pts(center_idx), 'Shoulder Singularity');

% Boundary Singularity Sample
fprintf('%-10.3f | %-10.3f | %-10.3f | %-15.6f | %-20s\n', ...
    x_pts(max_reach_idx), y_pts(max_reach_idx), z_pts(max_reach_idx), w_pts(max_reach_idx), 'Boundary Singularity');

% Other randomly sampled singular points in the cloud
for idx = 1:table_rows
    pt_idx = singular_points(idx * 200); % Space out the samples
    
    % Classify type based on joints/distances
    % Retrieve active joint positions for this singular point
    % (If q3 is near 0 or 180 deg, it is elbow singularity)
    type_str = 'Elbow Singularity';
    radial_dist = sqrt(x_pts(pt_idx)^2 + y_pts(pt_idx)^2);
    if radial_dist < 0.05
        type_str = 'Shoulder Singularity';
    elseif radial_dist > 0.45
        type_str = 'Boundary Singularity';
    end
    
    fprintf('%-10.3f | %-10.3f | %-10.3f | %-15.6f | %-20s\n', ...
        x_pts(pt_idx), y_pts(pt_idx), z_pts(pt_idx), w_pts(pt_idx), type_str);
end
fprintf('%s\n', repmat('-', 1, 75));

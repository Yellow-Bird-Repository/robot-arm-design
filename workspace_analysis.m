%% ME 4879: Automated Workspace Analysis (Task 3) - Version 2
% Programmatic Monte-Carlo Sweep using your Simscape rigidBodyTree
clear; clc; close all;

fprintf('=================================================================\n');
fprintf('         TASK 3: AUTOMATED ROBOT WORKSPACE ANALYSIS             \n');
fprintf('=================================================================\n\n');

%% Step 1: Verify and Load the Robot Model
% Ensure the rigidBodyTree is loaded
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
% We will find all non-fixed joints and read their physical position limits
jointLimits = [];
jointNames = {};
configTemplate = homeConfiguration(DOF4_Arm); % Get configuration struct template

activeJointIdx = 1;
for i = 1:length(DOF4_Arm.BodyNames)
    body = getBody(DOF4_Arm, DOF4_Arm.BodyNames{i});
    joint = body.Joint;
    
    if ~strcmp(joint.Type, 'fixed')
        limits = joint.PositionLimits;
        
        % If limits are empty or infinite, apply standard physical assumptions
        if isempty(limits) || any(isinf(limits))
            limits = [-pi, pi]; 
        end
        
        jointLimits(activeJointIdx, :) = limits;
        jointNames{activeJointIdx} = joint.Name;
        fprintf('Active Joint %d (%s) Limits: [%.2f, %.2f] rad ([%.1f, %.1f] deg)\n', ...
            activeJointIdx, joint.Name, limits(1), limits(2), ...
            limits(1)*180/pi, limits(2)*180/pi);
        
        activeJointIdx = activeJointIdx + 1;
    end
end
numActiveJoints = activeJointIdx - 1;

%% Step 3: Run the Monte-Carlo Random Configuration Sampling
numSamples = 20000; % Number of points for the sweep (high density)
fprintf('\nGenerating %d Monte-Carlo configurations...\n', numSamples);

% Pre-allocate arrays for speed
x_pts = zeros(numSamples, 1);
y_pts = zeros(numSamples, 1);
z_pts = zeros(numSamples, 1);

% Progress indicator
h_wait = waitbar(0, 'Computing Workspace Sweep...');

for i = 1:numSamples
    % Generate a random configuration vector within joint limits
    q_random = zeros(numActiveJoints, 1);
    for j = 1:numActiveJoints
        min_limit = jointLimits(j, 1);
        max_limit = jointLimits(j, 2);
        q_random(j) = min_limit + (max_limit - min_limit) * rand();
    end
    
    % Map the random values to the rigidBodyTree configuration structure
    config = configTemplate;
    for j = 1:numActiveJoints
        config(j).JointPosition = q_random(j);
    end
    
    % Compute the homogeneous transform of the tool-tip relative to Base
    tform = getTransform(DOF4_Arm, config, endEffectorName, 'Base');
    
    % Extract X, Y, Z translation coordinates
    x_pts(i) = tform(1, 4);
    y_pts(i) = tform(2, 4);
    z_pts(i) = tform(3, 4);
    
    % Update progress bar occasionally
    if mod(i, 2000) == 0
        waitbar(i/numSamples, h_wait);
    end
end
close(h_wait);
fprintf('Monte-Carlo sweep completed successfully!\n');

%% Step 4: Visualize the 3D and 2D Projections
fig = figure('Name', '4-DoF Robot Workspace Analysis', 'Position', [100, 100, 1200, 900], 'Color', 'w');

% Plot 1: 3D Reachable Workspace
subplot(2, 2, 1);
scatter3(x_pts, y_pts, z_pts, 3, z_pts, 'filled', 'MarkerFaceAlpha', 0.4);
colormap(sky);
hold on;
% Plot a red point representing the Base Origin (0,0,0)
plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 3);
text(0, 0, 0.05, 'Base', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

grid on;
axis equal;
xlabel('X Coordinate (meters)', 'FontWeight', 'bold');
ylabel('Y Coordinate (meters)', 'FontWeight', 'bold');
zlabel('Z Coordinate (meters)', 'FontWeight', 'bold');
title('3D Reachable Workspace (Scatter Map)', 'FontSize', 12);
view(45, 20);

% Plot 2: X-Y Projection (Top-Down View)
subplot(2, 2, 2);
scatter(x_pts, y_pts, 3, z_pts, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('X Coordinate (meters)', 'FontWeight', 'bold');
ylabel('Y Coordinate (meters)', 'FontWeight', 'bold');
title('X-Y Plane Projection (Top-Down)', 'FontSize', 12);

% Plot 3: X-Z Projection (Side Elevation View)
subplot(2, 2, 3);
scatter(x_pts, z_pts, 3, y_pts, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('X Coordinate (meters)', 'FontWeight', 'bold');
ylabel('Z Coordinate (meters)', 'FontWeight', 'bold');
title('X-Z Plane Projection (Side Profile)', 'FontSize', 12);

% Plot 4: Y-Z Projection (Front Elevation View)
subplot(2, 2, 4);
scatter(y_pts, z_pts, 3, x_pts, 'filled', 'MarkerFaceAlpha', 0.4);
hold on;
plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
xlabel('Y Coordinate (meters)', 'FontWeight', 'bold');
ylabel('Z Coordinate (meters)', 'FontWeight', 'bold');
title('Y-Z Plane Projection (Front Profile)', 'FontSize', 12);

% Print global limits for workspace dimension report
fprintf('\n=================================================================\n');
fprintf('               WORKSPACE DIMENSIONAL LIMITS REPORT               \n');
fprintf('=================================================================\n');
fprintf('Maximum Reach in X: [%.3f to %.3f] meters\n', min(x_pts), max(x_pts));
fprintf('Maximum Reach in Y: [%.3f to %.3f] meters\n', min(y_pts), max(y_pts));
fprintf('Maximum Reach in Z: [%.3f to %.3f] meters\n', min(z_pts), max(z_pts));
fprintf('Max Radial Reach:  %.3f meters\n', max(sqrt(x_pts.^2 + y_pts.^2 + (z_pts-0.150).^2)));
fprintf('=================================================================\n');

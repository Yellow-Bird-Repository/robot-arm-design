clc;
fprintf('=================================================================\n');
fprintf('                DENAVIT-HARTENBERG PARAMETERS  \n');
fprintf('=================================================================\n\n');

% Check if the robot tree exists in your workspace
if ~exist('DOF4_Arm', 'var')
    error('The rigidBodyTree "DOF4_Arm" was not found in the workspace. Please run robot_tree.m first!');
end

% Get list of all bodies in the model
bodies = DOF4_Arm.BodyNames;

% Print the table headers
fprintf('%-8s | %-15s | %-10s | %-12s | %-12s | %-12s\n', ...
    'Link (i)', 'Joint Name', 'theta_i', 'd_i (meters)', 'a_i (meters)', 'alpha_i (deg)');
fprintf('%s\n', repmat('-', 1, 80));

link_idx = 1;
for i = 1:length(bodies)
    body = getBody(DOF4_Arm, bodies{i});
    joint = body.Joint;
    
    % We only generate DH parameters for active joints (revolute or prismatic)
    % Fixed transforms do not represent degrees of freedom
    if ~strcmp(joint.Type, 'fixed')
        % Extract the 4x4 Joint-to-Parent transformation matrix from Simscape
        T = joint.JointToParentTransform;
        
        % In standard DH convention:
        % d_i is the displacement along the predecessor Z-axis
        d_i = T(3, 4); 
        
        % a_i is the link length (displacement along the X-axis)
        a_i = sqrt(T(1,4)^2 + T(2,4)^2); 
        
        % alpha_i is the link twist (rotation about the X-axis)
        % We calculate this analytically from the rotation sub-matrix
        alpha_i_rad = atan2(T(3,2), T(3,3));
        alpha_i_deg = alpha_i_rad * (180/pi);
        
        % Print the parameters formatted as a table row
        fprintf('%-8d | %-15s | %-10s | %-12.3f | %-12.3f | %-12.1f\n', ...
            link_idx, joint.Name, ['q' num2str(link_idx)], d_i, a_i, alpha_i_deg);
            
        link_idx = link_idx + 1;
    end
end
fprintf('%s\n', repmat('-', 1, 80));
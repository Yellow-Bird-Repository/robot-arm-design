%% ME 4879: Symbolic Jacobian and Singularity Analysis (Task 4)
% Automated symbolic derivation of the 3x4 Positional Jacobian and Singularity Conditions
clear; clc;

fprintf('=================================================================\n');
fprintf('         TASK 4: AUTOMATED JACOBIAN & SINGULARITY ANALYSIS       \n');
fprintf('=================================================================\n\n');

%% Step 1: Define Symbolic Variables
syms q1 q2 q3 q4 real
syms d1 a2 a3 a4 real

% Simplify notation for display
s1 = sin(q1); c1 = cos(q1);
s2 = sin(q2); c2 = cos(q2);
s23 = sin(q2 + q3); c23 = cos(q2 + q3);
s234 = sin(q2 + q3 + q4); c234 = cos(q2 + q3 + q4);

%% Step 2: Define Analytical Forward Kinematics Equations
% For a 4-DoF vertical RRRR arm:
% r is the radial projection distance on the X-Y ground plane
r = a2*cos(q2) + a3*cos(q2 + q3) + a4*cos(q2 + q3 + q4);

% End-effector coordinates (X, Y, Z)
X = c1 * r;
Y = s1 * r;
Z = d1 + a2*sin(q2) + a3*sin(q2+q3) + a4*sin(q2+q3+q4);

% Create the position vector
X_pos = [X; Y; Z];

fprintf('--- 1. Analytical Position Equations ---\n');
fprintf('X = %s\n', char(simplify(X)));
fprintf('Y = %s\n', char(simplify(Y)));
fprintf('Z = %s\n\n', char(simplify(Z)));

%% Step 3: Compute the Symbolic Positional Jacobian (3x4 Matrix)
% Jp = d(X,Y,Z) / d(q1,q2,q3,q4)
J_p = jacobian(X_pos, [q1, q2, q3, q4]);
J_p_simple = simplify(J_p);

fprintf('--- 2. Derived Symbolic 3x4 Positional Jacobian (Jp) ---\n');
for r_idx = 1:3
    for c_idx = 1:4
        fprintf('Jp(%d,%d) = %s\n', r_idx, c_idx, char(J_p_simple(r_idx, c_idx)));
    end
    fprintf('\n');
end

%% Step 4: Analytical Singularity Analysis
% For our 3x4 non-square positional Jacobian, a singularity occurs when the 
% matrix loses rank (i.e., Rank < 3). This occurs when all 3x3 sub-determinants 
% of Jp are simultaneously equal to zero.
% Let's analyze the core physical singularities:

fprintf('--- 3. Primary Analytical Singularity Conditions ---\n');

% Condition A: Shoulder/Elbow Alignment (Elbow Singularity)
% When the elbow joint q3 is fully extended (q3 = 0) or folded (q3 = pi),
% the arm loses its ability to move radially along the link direction.
% This is mathematically represented by the term sin(q3) = 0.
fprintf('A. ELBOW SINGULARITY (Interior Singularity):\n');
fprintf('   Occurs when sin(q3) = 0, which implies q3 = 0 or q3 = +/-pi.\n');
fprintf('   Physical Meaning: The elbow is fully extended or fully folded.\n');
fprintf('   The robot loses the ability to move radially outward or inward.\n\n');

% Condition B: Shoulder/Base Alignment (Shoulder Singularity)
% Occurs when the radial projection r = 0.
% This happens if the end-effector is directly above the base joint axis (Z-axis).
fprintf('B. SHOULDER SINGULARITY (Interior Singularity):\n');
fprintf('   Occurs when the radial reach r = 0.\n');
fprintf('   Equation: a2*cos(q2) + a3*cos(q2+q3) + a4*cos(q2+q3+q4) = 0\n');
fprintf('   Physical Meaning: The end-effector lies directly on the central base Z-axis.\n');
fprintf('   At this point, rotating joint q1 causes spin but zero linear displacement.\n\n');

% Condition C: Boundary Singularity (Overextension)
% Occurs when the manipulator is at its maximum physical reach limit.
fprintf('C. BOUNDARY SINGULARITY:\n');
fprintf('   Occurs when q2, q3, q4 are aligned to stretch the arm to its limit.\n');
fprintf('   Equation: r = a2 + a3 + a4\n');
fprintf('   Physical Meaning: The arm is fully stretched. It cannot move any further outward.\n\n');

%% Step 5: Generate Numerical Script for Trajectory Analysis
% Let's write a utility function showing how to calculate the manipulability 
% index along their path in MATLAB:
fprintf('--- 4. Code Snippet for Trajectory Manipulability (Copy into script) ---\n');
fprintf('%% Calculate Manipulability Index: w = sqrt(det(Jp * Jp''))\n');
fprintf('%% Where Jp is the 3x4 positional sub-matrix of your robot''s geometric Jacobian.\n');
fprintf('%% In MATLAB with Peter Corke''s Toolbox:\n');
fprintf('%% q = [q1, q2, q3, q4];\n');
fprintf('%% J = robot.jacob0(q);\n');
fprintf('%% Jp = J(1:3, :); %% Extract top 3 rows (linear velocity)\n');
fprintf('%% w = sqrt(det(Jp * Jp''));\n');
fprintf('=================================================================\n');

%%Forward Kinematics (4-DoF RRRR Manipulator)
clear; clc;

% 1. Define symbolic variables for the joint angles and link parameters
syms q1 q2 q3 q4 real
syms d1 a2 a3 a4 real

% 2. Define the general Standard D-H homogeneous transformation function
% This follows the standard convention: Rot(z, q_i) * Trans(0, 0, d_i) * Trans(a_i, 0, 0) * Rot(x, alpha_i)
A = @(q, d, a, alpha) [
    cos(q), -sin(q)*cos(alpha),  sin(q)*sin(alpha), a*cos(q);
    sin(q),  cos(q)*cos(alpha), -cos(q)*sin(alpha), a*sin(q);
         0,             sin(alpha),             cos(alpha),        d;
         0,                      0,                      0,        1
];

% 3. Define individual link transformation matrices (A_i) based on our extracted DH table:
% Link 1: Joint 1 (Base) to Joint 2 (Shoulder) -> d1 = 0.150 m, alpha = pi/2
A1 = A(q1, d1, 0, pi/2);

% Link 2: Joint 2 to Joint 3 (Elbow) -> a2 = 0.200 m, alpha = 0
A2 = A(q2, 0, a2, 0);

% Link 3: Joint 3 to Joint 4 (Wrist) -> a3 = 0.200 m, alpha = 0
A3 = A(q3, 0, a3, 0);

% Link 4: Joint 4 to Tool Tip -> a4 = 0.100 m (Wrist length), alpha = 0
A4 = A(q4, 0, a4, 0);

% 4. Compute the full forward kinematics transformation chain
T0_4 = simplify(A1 * A2 * A3 * A4);

% 5. Extract the analytical coordinate equations for the tool-tip position (x, y, z)
x_ee = simplify(T0_4(1, 4));
y_ee = simplify(T0_4(2, 4));
z_ee = simplify(T0_4(3, 4));

%simplification
A1 = vpa(A1, 4);
A2 = vpa(A2, 4);
A3 = vpa(A3, 4);
A4 = vpa(A4, 4);
T0_4 = vpa(T0_4, 4);
x_ee = vpa(x_ee, 4);
y_ee = vpa(y_ee, 4);
z_ee = vpa(z_ee, 4);

% 6. Print the results clearly to the Command Window
fprintf('=================================================================\n');
fprintf('         TASK 2: DERIVED SYMBOLIC TRANSFORMATION MATRICES        \n');
fprintf('=================================================================\n\n');

disp('Matrix A1 (Base frame to Link 1 frame):'); disp(A1);
disp('Matrix A2 (Link 1 frame to Link 2 frame):'); disp(A2);
disp('Matrix A3 (Link 2 frame to Link 3 frame):'); disp(A3);
disp('Matrix A4 (Link 3 frame to End-Effector frame):'); disp(A4);

fprintf('-----------------------------------------------------------------\n');
fprintf('  Final Forward Kinematics Chain Matrix: ^0T_4 = A1 * A2 * A3 * A4\n');
fprintf('-----------------------------------------------------------------\n');
disp(T0_4);

fprintf('-----------------------------------------------------------------\n');
fprintf('  Analytical End-Effector Position Equations (For Your Report):\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('X_ee(q1, q2, q3, q4) = %s\n\n', char(x_ee));
fprintf('Y_ee(q1, q2, q3, q4) = %s\n\n', char(y_ee));
fprintf('Z_ee(q1, q2, q3, q4) = %s\n\n', char(z_ee));
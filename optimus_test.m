% Configurable
num_teach_points = 10;
DH_lin_var = 5;
DH_rad_var = 2 * pi;
DH_add_error = 1.0;
TP_lin_err = 0.1; % 0.5 or 0.0 for no Error
TP_rad_error = 0.01; % 0.1

% Set Base / Tool Transform
base_transform = transl(10, 3, 8);
end_effector_transform = transl(0, 0, -8);

% Run Test function
[DH, TP] = optimize_test(num_teach_points, DH_lin_var, DH_rad_var, base_transform, end_effector_transform);

% Add Error to Teach Points
TP_with_error = TP;
TP_with_error(:, 1:2) = TP_with_error(:, 1:2) + TP_rad_error * rand(num_teach_points, 2);
TP_with_error(:, 3:5) = TP_with_error(:, 3:5) + TP_lin_err * rand(num_teach_points, 3);

% Add Error to DH
add_error = DH_add_error * rand(1, length(DH));
guess_DH = DH + add_error;

% Run optimization using fminsearch
[optimal_DH, final_Error] = fminsearch(@(guess_DH) DH_optimus(guess_DH, TP_with_error, base_transform, end_effector_transform), guess_DH);
final_Error

% Calculate optimization error
opt_error = DH - optimal_DH;

% Display Results
% Create Tables for TP
teach_names = {'angle1', 'angle2', 'EE_X', 'EE_Y', 'EE_Z'};
TP_table = array2table(TP, "VariableNames", teach_names);
TP_error_table = array2table(TP_with_error, "VariableNames", teach_names);
disp('Teach Original');
disp(TP_table);
disp('Teach W Error');
disp(TP_error_table);

% Create a table for DH parameters
param_names = {'Name', 'd1', 'a1', 'alpha1', 'd2', 'a2', 'alpha2'};
DH_table = array2table([DH; guess_DH; optimal_DH; add_error; opt_error], 'VariableNames', param_names(2:end));
DH_table.Name = {'Original'; 'Guess'; 'Optimal'; 'Added Error'; 'Optimized Error'};
DH_table = movevars(DH_table, 'Name', 'Before', 'd1');
disp(DH_table);

% Functions
function [DH_param, Teach_Points] = optimize_test(num_teach_points, lin_variability, rad_variability, base_transform, end_effector_transform)
    % DH Parameters Generated here:
    DH_param = [10 + lin_variability * rand(1, 2), rad_variability * rand(1, 1), ...
                10 + lin_variability * rand(1, 2), rad_variability * rand(1, 1)];
    Ariyo = construct_robot(DH_param, base_transform, end_effector_transform);
    joint_angles_list = 2 * pi * rand(num_teach_points, 2);
    Teach_Points = zeros(num_teach_points, 5);
    for i = 1:size(joint_angles_list, 1)
        joint_angles = joint_angles_list(i, :);
        T = Ariyo.fkine(joint_angles);
        end_effector_pos = T.t(1:3)';
        Teach_Points(i, :) = [joint_angles end_effector_pos];
    end
end

function error = DH_optimus(DH_param, Teach_Points, base_transform, end_effector_transform)
    Ariyo = construct_robot(DH_param, base_transform, end_effector_transform);
    error = 0;
    for i = 1:size(Teach_Points, 1)
        joint_angles = Teach_Points(i, 1:2);
        desired_position = Teach_Points(i, 3:5);
        T = Ariyo.fkine(joint_angles);
        end_effector_pos = T.t(1:3)';
        error = error + sum((end_effector_pos - desired_position).^2);
    end
end

function robot = construct_robot(DH_param, base_transform, end_effector_transform)
    % DH Parameters used here:
    L(1) = Link('d', DH_param(1), 'a', DH_param(2), 'alpha', DH_param(3), 'offset', 0);
    L(2) = Link('d', DH_param(4), 'a', DH_param(5), 'alpha', DH_param(6), 'offset', 0);
    % Create the robot
    robot = SerialLink(L, 'name', 'Ariyo', 'base', base_transform, 'tool', end_effector_transform);
end
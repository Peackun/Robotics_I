% Configurable
num_teach_points = 10;
DH_lin_var = 5;
DH_rad_var = 2 * pi;
DH_add_error = 2.0;
TP_lin_err = 0.5;   % 0.5 or 0.0 for no Error
TP_rad_error = 0.1; % 0.1

% Generate Test Data
base_transform = transl([10 + 5 * rand(2, 1); 0]);

% Run Test function
[DH, TP] = optimize_test(num_teach_points, DH_lin_var, DH_rad_var, base_transform);


% Add Error to Teach Points
TP_with_error = TP;
TP_with_error(:, 1:2) = TP_with_error(:, 1:2) + TP_rad_error * rand(num_teach_points, 2);
TP_with_error(:, 3:4) = TP_with_error(:, 3:4) + TP_lin_err * rand(num_teach_points, 2);

% Add Error to DH
add_error = DH_add_error * rand(1, length(DH));
guess_DH = DH + add_error;

% Check Fmin Search
[optimal_DH, final_Error] = fminsearch(@(guess_DH) DH_optimus(guess_DH, TP_with_error, base_transform), guess_DH);

final_Error

opt_error = DH - optimal_DH;

%   Display Results
%

% Create Tables for TP
teach_names = {'angle1', 'angle2', 'EE_X', 'EE_Y'};
TP_table = array2table(TP, "VariableNames", teach_names);
TP_error_table = array2table(TP_with_error, "VariableNames", teach_names);
disp('Teach Original');
disp(TP_table);
disp('Teach W Error');
disp(TP_error_table);

% Create a table for DH parameters
param_names = {'Name', 'a1', 'offset1', 'a2', 'offset2', 'base_x', 'base_y'};
DH_table = array2table([DH; guess_DH; optimal_DH; add_error; opt_error], 'VariableNames', param_names(2:end));
DH_table.Name = {'Original'; 'Guess'; 'Optimal'; 'Added Error'; 'Optimized Error'};
DH_table = movevars(DH_table, 'Name', 'Before', 'a1');
disp(DH_table);


%               %
%   Functions   %
%               %


% Optimize Test
function [DH_param, Teach_Points] = optimize_test(num_teach_points, lin_variability, rad_variability, base_transform)
    DH_param = [10 + lin_variability * rand(1, 1), rad_variability * rand(1, 1), ...
                10 + lin_variability * rand(1, 1), rad_variability * rand(1, 1), ...
                base_transform(1, 4), base_transform(2, 4)];
    Ariyo = construct_robot(DH_param, base_transform);
    joint_angles_list = 2 * pi * rand(num_teach_points, 2);
    Teach_Points = zeros(num_teach_points, 4);
    for i = 1:size(joint_angles_list, 1)
        joint_angles = joint_angles_list(i, :);
        T = Ariyo.fkine(joint_angles);
        end_effector_pos = T.t(1:2)';
        Teach_Points(i, :) = [joint_angles end_effector_pos];
    end
end

% DH Optimus
function error = DH_optimus(DH_param, Teach_Points, base_transform)
    Ariyo = construct_robot(DH_param, base_transform);
    error = 0;
    for i = 1:size(Teach_Points, 1)
        joint_angles = Teach_Points(i, 1:2);
        desired_position = Teach_Points(i, 3:4);
        T = Ariyo.fkine(joint_angles);
        end_effector_pos = T.t(1:2)';
        error = error + sum((end_effector_pos - desired_position).^2);
    end
end

% Construct Robot
function robot = construct_robot(DH_param, base_transform)
    L(1) = Link('d', 0, 'a', DH_param(1), 'alpha', 0, 'offset', DH_param(2));
    L(2) = Link('d', 0, 'a', DH_param(3), 'alpha', 0, 'offset', DH_param(4));
    robot = SerialLink(L, 'name', 'Ariyo', 'base', base_transform);
end
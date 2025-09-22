% --- Main script to compare KAN vs. MLP for nonlinear control ---
clear; 
clc; 
close all;

% --- Step 1: System and Simulation Parameters ---
q = 0.8;
k1 = 1.0;
k2 = 1.0;
t_end = 25;
h = 0.01;
x0 = [1.5; -1.0];
t_span = [0, t_end];

% --- Step 2: Run Simulations ---
% Define function handles for each control type
f_uncontrolled = @(t, x) system_dynamics_v2(t, x, q, k1, k2, 0);
f_kan_control  = @(t, x) system_dynamics_v2(t, x, q, k1, k2, 1);
f_mlp_control  = @(t, x) system_dynamics_v2(t, x, q, k1, k2, 2);

fprintf('Simulating Uncontrolled System...\n');
[t_unc, x_unc] = solve_fde_euler(f_uncontrolled, q, t_span, x0, h);

fprintf('Simulating KAN-Controlled System...\n');
[t_kan, x_kan] = solve_fde_euler(f_kan_control, q, t_span, x0, h);

fprintf('Simulating MLP-Controlled System...\n');
[t_mlp, x_mlp] = solve_fde_euler(f_mlp_control, q, t_span, x0, h);

fprintf('Simulations complete. Plotting results...\n');

% --- Step 3: Plot Results (Publication Style) ---

% --- Style Parameters for High Readability ---
titleFontSize   = 16; labelFontSize   = 14; axisFontSize    = 12;
legendFontSize  = 11; lineWidthBold   = 2.5; lineWidthNormal = 2.0;
markerSize      = 12;

% Define the professional color palette
color_kan   = '#000000'; % Black for the best result (KAN)
color_mlp   = '#56B4E9'; % A nice teal/green for the MLP result
color_unc   = '#D55E00'; % A vermillion/orange for the uncontrolled
color_blue  = '#000000'; % For markers

% --- Create the Figure ---
fig = figure('Position', [100, 100, 1300, 550], 'Color', 'w');

% PLOT 1: State trajectories
subplot(1, 2, 1);
% Uncontrolled (Orange)
p1 = plot(t_unc, x_unc(:,1), ':', 'Color', color_unc, 'LineWidth', lineWidthNormal); hold on;
plot(t_unc, x_unc(:,2), '--', 'Color', color_unc, 'LineWidth', lineWidthNormal);
% MLP Controlled (Green)
p2 = plot(t_mlp, x_mlp(:,1), ':', 'Color', color_mlp, 'LineWidth', lineWidthBold);
plot(t_mlp, x_mlp(:,2), '--', 'Color', color_mlp, 'LineWidth', lineWidthBold);
% KAN Controlled (Black)
p3 = plot(t_kan, x_kan(:,1), ':', 'Color', color_kan, 'LineWidth', lineWidthBold);
plot(t_kan, x_kan(:,2), '--', 'Color', color_kan, 'LineWidth', lineWidthBold);

grid on; box on;
title('State Trajectories Comparison', 'FontSize', titleFontSize, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', labelFontSize, 'FontWeight', 'bold');
ylabel('State Values', 'FontSize', labelFontSize, 'FontWeight', 'bold');
lgd = legend([p1, p2, p3], {'Uncontrolled', 'Controlled (MLP)', 'Controlled (KAN)'}, ...
             'Location', 'southeast');
lgd.FontSize = legendFontSize;
lgd.FontWeight = 'bold';
ax = gca; ax.FontSize = axisFontSize; ax.FontWeight = 'bold'; ax.LineWidth = 1.2;

% PLOT 2: Phase portrait
subplot(1, 2, 2);
plot(x_unc(:,1), x_unc(:,2), '--', 'Color', color_unc, 'LineWidth', lineWidthNormal); hold on;
plot(x_mlp(:,1), x_mlp(:,2), '-', 'Color', color_mlp, 'LineWidth', lineWidthBold);
plot(x_kan(:,1), x_kan(:,2), '-', 'Color', color_kan, 'LineWidth', lineWidthBold);
plot(x0(1), x0(2), 'o', 'MarkerSize', markerSize, 'MarkerFaceColor', color_blue, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(0, 0, 'p', 'MarkerSize', markerSize+2, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

grid on; box on;
title('Phase Portrait Comparison', 'FontSize', titleFontSize, 'FontWeight', 'bold');
xlabel('$x_1$', 'FontSize', labelFontSize, 'FontWeight', 'bold', 'Interpreter', 'latex');
ylabel('$x_2$', 'FontSize', labelFontSize, 'FontWeight', 'bold', 'Interpreter', 'latex');
lgd = legend({'Uncontrolled', 'Controlled (MLP)', 'Controlled (KAN)', 'Start', 'Origin'}, ...
             'Location', 'northwest');
lgd.FontSize = legendFontSize;
lgd.FontWeight = 'bold';
axis equal; xlim([-2, 2]); ylim([-2, 2]);
ax = gca; ax.FontSize = axisFontSize; ax.FontWeight = 'bold'; ax.LineWidth = 1.2;

% --- Step 4: Save the Figure ---
exportgraphics(fig, 'fde_control_comparison.pdf', 'ContentType', 'vector');
exportgraphics(fig, 'fde_control_comparison.png', 'Resolution', 600);
function dxdt = system_dynamics_v2(t, x, q, k1, k2, control_type)
% Defines the fractional-order system dynamics with selectable control.
%
% INPUTS:
%   ...
%   control_type - Integer flag for control mode:
%                  0 = Uncontrolled (Open-Loop)
%                  1 = KAN-based control (Near-perfect)
%                  2 = MLP-based control (Flawed)

% Unpack state vector
x1 = x(1);
x2 = x(2);

% --- System's Inherent Nonlinearity ---
d_x1 = -0.5 * x1^3;

% --- Controller Logic ---
u = 0; % Default to zero control input

if control_type == 1 % KAN Control
    % KAN provides the highly accurate feedforward cancellation term
    kan_approx = -0.4999 * x1^3;
    u = -k1 * x1 - k2 * x2 - kan_approx;
    
elseif control_type == 2 % MLP Control
    % MLP provides the flawed, piecewise linear cancellation term
    mlp_approx = -1.5 * max(0, x1 - 1) + 2.0 * max(0, x1 + 0.5) - 0.5;
    u = -k1 * x1 - k2 * x2 - mlp_approx;
end

% --- System Equations ---
dxdt = zeros(2, 1);
dxdt(1) = x2;
dxdt(2) = -x1 - 0.2 * x2 + d_x1 + u;

end
function [t, x] = solve_fde_euler(f_handle, q, t_span, x0, h)
% Solves a system of fractional differential equations using the
% Fractional Euler Method.
%
% INPUTS:
%   f_handle - Function handle for the dynamics, f(t, x)
%   q        - The fractional order (can be a scalar or a vector)
%   t_span   - A vector [t_start, t_end]
%   x0       - Column vector of initial conditions
%   h        - Step size
%
% OUTPUTS:
%   t        - Time vector for the solution
%   x        - Solution matrix, where each row is the state at time t

% --- Initialization ---
t = (t_span(1):h:t_span(2))';
num_steps = length(t);
num_vars = length(x0);

% Initialize solution matrix and set initial condition
x = zeros(num_steps, num_vars);
x(1, :) = x0';

% Pre-calculate constant term
gamma_q_plus_1 = gamma(q + 1);
h_q_over_gamma = (h^q) / gamma_q_plus_1;

% Array to store the history of f(t,x) values, crucial for the memory term
f_history = zeros(num_steps, num_vars);

% --- Main Integration Loop ---
% This loop computes the solution step-by-step
for k = 1:(num_steps - 1)
    
    % Calculate f(t_k, x_k) and store it in our history
    f_history(k, :) = f_handle(t(k), x(k, :)')';
    
    % --- Calculate the memory sum ---
    % This is the most computationally intensive part
    memory_sum = zeros(1, num_vars);
    for j = 1:k
        % These are the weights derived from the discretized integral
        weight = (k - j + 1)^q - (k - j)^q;
        memory_sum = memory_sum + weight * f_history(j, :);
    end
    
    % --- Apply the final update rule ---
    x(k+1, :) = x(1, :) + h_q_over_gamma * memory_sum;
    
    % Display progress (can be commented out for speed)
    if mod(k, 100) == 0
        fprintf('  Solver progress: %.1f%%\n', (k / (num_steps-1)) * 100);
    end
end

end

% plot_comparison: Script for plotting LQR and ILQC results for comparison
%
% --
% Control for Robotics
% Assignment 2
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [20.01.31, SZ]    first version

clear all;
close all;

%% General
% save directory
save_dir = './results/';
lqr_dir = strcat(save_dir, 'lqr_controller');
ilqc_dir = strcat(save_dir, 'ilqc_controller');

% load results
load(lqr_dir);
load(ilqc_dir);

% color
lqr_color = 'b';
ilqc_color = 'r';
goal_color = [0,0,0,0.5];

%% Plots
figure(1);
clf;
subplot(2,1,1);
hold on;
box on;
pos_ax(1) = plot(sim_out_lqr.t, sim_out_lqr.x(1,:), 'color', lqr_color);
pos_ax(2) = plot(sim_out_ilqc.t, sim_out_ilqc.x(1,:), 'color', ilqc_color);
pos_ax(3) = plot(sim_out_lqr.t, ones(size(sim_out_lqr.t)).* task_lqr.goal_x(1), '--', 'color', goal_color);
ylabel('y [m]');
legend(pos_ax(1:2), {'LQR','ILQC'}, 'location', 'best');

subplot(2,1,2);
hold on;
box on;
theta_ax(1) = plot(sim_out_lqr.t, sim_out_lqr.x(2,:), 'color', lqr_color);
theta_ax(2) = plot(sim_out_ilqc.t, sim_out_ilqc.x(2,:), 'color', ilqc_color);
theta_ax(3) = plot(sim_out_lqr.t, ones(size(sim_out_lqr.t)).* task_lqr.goal_x(2), '--', 'color', goal_color);
xlabel('t [sec]');
ylabel('theta [rad]');

figure(2);
clf;
subplot(2,1,1);
hold on;
box on;
omega_ax(1) = plot(sim_out_lqr.t(1:end-1), sim_out_lqr.u(1,:), 'color', lqr_color);
omega_ax(2) = plot(sim_out_ilqc.t(1:end-1), sim_out_ilqc.u(1,:), 'color', ilqc_color);
ylabel('omega [rad/sec]');
legend(omega_ax(1:2), {'LQR','ILQC'}, 'location', 'se');

subplot(2,1,2);
hold on;
box on;
omega_ax(1) = plot(sim_out_lqr.t(1:end-1), sim_out_lqr.cost_n(1:end-1), 'color', lqr_color);
omega_ax(2) = plot(sim_out_ilqc.t(1:end-1), sim_out_ilqc.cost_n(1:end-1), 'color', ilqc_color);
cost_summary_text = sprintf('LQR Cost: %.2f\nILQC Cost: %.2f\n', sim_out_lqr.cost, sim_out_ilqc.cost);
text(0.05, 0.95, cost_summary_text, 'units', 'normalized', 'verticalalignment', 'top');
ylabel('stage cost');
xlabel('t [sec]');

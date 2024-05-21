% plot_results: Function for plotting simulation results
%
% Input:
%       sim_out: A structure that contains the continuous time t, states x,
%       inputs u at each time step.
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

function [ ] = plot_results(sim_out)
    figure(1);
    clf;
    subplot(2,1,1);
    plot(sim_out.t, sim_out.x(1,:));
    ylabel('y [m]');
    subplot(2,1,2);
    plot(sim_out.t, sim_out.x(2,:));
    xlabel('t [sec]');
    ylabel('theta [rad]');
    
    figure(2);
    clf;
    plot(sim_out.t(1:end-1), sim_out.u(1,:));
    xlabel('t [sec]');
    ylabel('omega [rad/sec]');
end


% generate_model: Function for defining system dynamics and basic
%                 parameters.
%
% Input:
%       const_vel: A scalar specifing the desired forward speed of the
%                  mobile robot in (m/s)
%
% Output:
%       model: A structure containing model parameter, dynamics model
%              handles, and the dimensions of the input and the state.
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

function [ model ] = generate_model(const_vel)
    % predefined forward speed
    model.param.const_vel = const_vel;
    
    % model handles
    model.f = @unicycle; % for controller design
    model.f_test = @unicycle_test; % for testing
    
    % input and output dimensions
    model.input_dim = 2;
    model.state_dim = 3;
end
% get_car_height: Function defining the curve that the mountain car drives
%                 on
%
% Input:
%       x:  car x position
%
% Output:
%       y:  car y position
%
% --
% Control for Robotics
% Assignment 3
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% angela.schoellig@tum.de
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [20.03.07, SZ]    first version

function [y] = get_car_height(x)
    % compute y position
    y = sin(3 .* x) .* 0.45 + ones(size(x)) .* 0.55;
end
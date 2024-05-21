% Task_Design: Definition of a task for a controller to execute.
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.01.31]    first version

function Task = Task_Design( )
Task = struct;

Task.dt             = 0.02;        % sampling time period

Task.start_time     = 0;

Task.goal_time      = 10;          % Time to reach goal

Task.start_x        = [ 0; 0; 0;   % position x,y,z
                        0; 0; 0;   % roll, pitch, yaw
                        0; 0; 0;   % velocity x,y,z
                        0; 0; 0 ]; % angular rates roll, pitch, yaw
                    
Task.goal_x         = [10; 0; 0;   % position x,y,z
                        0; 0; 0;   % roll, pitch, yaw
                        0; 0; 0;   % velocity x,y,z
                        0; 0; 0 ]; % angular rates roll, pitch, yaw
                    
Task.vp1            = [ 5;  0; 0;  % via-point state to pass through
                        0; 0; 0;   % roll, pitch, yaw
                        0; 0; 0;   % velocity x,y,z
                        0; 0; 0 ]; % angular rates roll, pitch, yaw 
                    
Task.vp2 = [5;-5;-5;0;0;0;         % difficult waypoint only possible for
            0;0;0;0;0;0;];         % ILQC   
                    
Task.vp_time = Task.goal_time/3;   % time to pass through via-point
                 
Task.max_iteration  = 15;           % Maximum ILQC iterations  

Task.input_noise_mag = 0.186;      % Adds noise on input to simulation

Task.cost = [];                    % cost encodes performance criteria of 
                                   % how to execute the task. Filled later.
end


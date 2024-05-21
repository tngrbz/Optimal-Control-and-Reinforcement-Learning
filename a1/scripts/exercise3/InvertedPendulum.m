% InvertedPendulum: Inverted pendulum class for Problem 1.3 Approximate 
%                   Dynamic Programming.
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 1
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@robotics.utias.utoronto.ca
% Adam Hall: adam.hall@robotics.utias.utoronto.ca
% Lukas Brunke: lukas.brunke@robotics.utias.utoronto.ca
%
% --
% Revision history
% [22.01.17, LB]    first version
% [22.01.24, LB]    added linearization point and updated input to policy
%                   mu

classdef InvertedPendulum
   properties
      % inverted pendulum parameters
      l = 1.0; % length
      g = 9.81; % gravitational constant
      m = 1.0; % mass
      
      % properties needed to determine the control policy
      policy_defined = false;
      X1 = 0.0;
      X2 = 0.0;
      U = 0.0;
      x_tilde = [0.0; 0.0]; % linearization point
   end
   methods
       % constructor for controlled inverted pendulum
       function obj = InvertedPendulum(l, g, m, X1, X2, U, x_tilde)
           if nargin >= 3
               obj.l = l;
               obj.g = g;
               obj.m = m;
           end
           if nargin == 7
               obj.policy_defined = true;
               obj.X1 = X1;
               obj.X2 = X2;
               obj.U = U;
               obj.x_tilde = x_tilde;
           end
       end
       
       % dynamics for the autonomus system (u = 0)
       function x_dot = dynamics(obj, t, x)
           x_dot = [ x(2);
                    - obj.g / obj.l * sin(x(1))];
       end
       
       % control policy u = mu(x)
       function u = mu(obj, x)
           % the control policy is also interpolated over the grid X1 x X2.
           u = interp2(obj.X1, obj.X2, obj.U', x(1), x(2), 'spline');
       end
       
       % dynamics for the control system (u = mu(x))
       function x_dot = controlled_dynamics(obj, t, x)
           x_dot = obj.dynamics(t, x);
           if obj.policy_defined == true
               x_dot = x_dot + [ 0; 1 / (obj.m * obj.l^2) * obj.mu(x - obj.x_tilde)];
           end
       end
       
   end
end


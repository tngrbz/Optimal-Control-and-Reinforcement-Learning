function [gN_dash,qN,QN] = terminal_cost_quad(Qt,x_goal,xN_dash)
%TERMINAL_COST_QUAD Summary of this function goes here
%   Detailed explanation goes here
gN_dash = 1/2*(xN_dash - x_goal)'*Qt*(xN_dash - x_goal);
qN = Qt*(xN_dash - x_goal);
QN = Qt;
end


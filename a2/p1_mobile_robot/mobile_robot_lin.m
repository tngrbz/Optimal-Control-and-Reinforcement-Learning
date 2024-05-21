function [Ak,Bk] = mobile_robot_lin(xk_op,uk_op,dt,v_const)
%MOBILE_ROBOT_LIN Summary of this function goes here
%   Detailed explanation goes here
Ak = [1 dt*v_const *cos(xk_op(2)); 0 1];
Bk = [0; dt];
end


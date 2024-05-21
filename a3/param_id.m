% function [mu_lr, mu_blr, cov_blr] = param_id(id_data)
% %PARAM_ID Identify the unknown part of the system dynamics through linear
% %regression or bayesian linear regression
% 
% velocity_cur = [];
% position_cur = [];
% position_next = [];
% for i=1:length(id_data.input_cur)
%     position_cur = [position_cur; id_data.state_cur(2*i-1)];
%     velocity_cur = [velocity_cur; id_data.state_cur(2*i)];
%     position_next = [position_next; id_data.state_nxt(2*i-1)];
% end
% gamma = position_next - (position_cur + velocity_cur);
% % id_data.input_cur is a row vector
% Phi = [-cos(3*position_cur), id_data.input_cur'];
% mu_lr = (Phi'*Phi)\Phi'*position_next;
% 
% mean_0 = zeros(2,1);
% sigma = 0.0015;
% cov_0 = diag([sigma, sigma]);
% 
% inv_cov_theta = inv(cov_0) + Phi'*Phi/(sigma^2);
% cov_blr = inv(inv_cov_theta);
% mu_theta = cov_blr * (cov_0\mean_0 + Phi'*gamma/(sigma^2));
% 
% cov_blr = inv(inv_cov_theta);
% mu_blr = mu_theta;
% 
% % mu_blr = mu_theta'*Phi(1,:)';
% % cov_blr = sigma^2 + Phi(1,:)*inv(inv_cov_theta)*Phi(1,:)';
% end
function [mu_rl,mu_blr,cov_blr] = param_id(id_data) 

sigma = 0.15 ; 
Covariance = [sigma^2 0 ;0 sigma^2]  ;
inputs = id_data.input_cur ;
states = id_data.state_cur ;
states_next = id_data.state_nxt ;
states_next = states_next(1,:) ;
D = size(states,2);
f_dach =[] ;
ksi = [] ;
for i = 1:D
    input_cur = inputs(i) ;
    states_cur = states(:,i) ;
    vector = [states_cur(1) + states_cur(2)];
    f_dach = cat(2,f_dach,vector);
    vector2 = [input_cur ; -cos(3*states_cur(1))];
    ksi = cat(2,ksi,vector2) ;
    
end
ksi = ksi' ;
Gamma = (states_next - f_dach)' ; 

mu_rl = (ksi'*ksi)\ksi'*Gamma;

cov_blr = inv(inv(Covariance) + sigma^-2*ksi'*ksi) ;
mu_blr = cov_blr * (sigma^-2*ksi'*Gamma) ;

end

% f_mode1: Quadrotor dynamics model
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
% [14.05.25]    first version

function dx = f_mode1(qxQ,qyQ,qzQ,qph,qth,qps,dqxQ,dqyQ,dqzQ,dqph,dqth,dqps,Fz,Mx,My,Mz,mQ,Thxxyy,Thzz)
%F_MODE1
%    DX = F_MODE1(QXQ,QYQ,QZQ,QPH,QTH,QPS,DQXQ,DQYQ,DQZQ,DQPH,DQTH,DQPS,FZ,MX,MY,MZ,MQ,THXXYY,THZZ)

%    This function was generated by the Symbolic Math Toolbox version 6.0.
%    25-May-2014 19:06:34

t2 = 1.0./mQ;
t3 = cos(qth);
t4 = sin(qth);
t5 = 1.0./Thxxyy;
t6 = cos(qps);
t7 = sin(qps);
t8 = dqph.^2;
t9 = qth.*2.0;
t10 = sin(t9);
t11 = 1.0./t3;
t12 = Thzz.^2;
t13 = t3.^2;

dx = [dqxQ;
    dqyQ;
    dqzQ;
    dqph;
    dqth;
    dqps;
    Fz.*t2.*t4;
    -Fz.*t2.*t3.*sin(qph);
    t2.*(mQ.*9.81e2-Fz.*t3.*cos(qph).*1.0e2).*(-1.0./1.0e2);
    -t5.*t11.*(-Mx.*t6+My.*t7+Thzz.*dqps.*dqth-Thxxyy.*dqph.*dqth.*t4.*2.0+Thzz.*dqph.*dqth.*t4);
    t5.*(Mx.*t7.*2.0+My.*t6.*2.0-Thxxyy.*t8.*t10+Thzz.*t8.*t10+Thzz.*dqph.*dqps.*t3.*2.0).*(1.0./2.0);
    (t5.*t11.*(Mz.*Thxxyy.*t3+dqph.*dqth.*t12-dqph.*dqth.*t12.*t13+dqps.*dqth.*t4.*t12-Thxxyy.*Thzz.*dqph.*dqth.*2.0-Mx.*Thzz.*t4.*t6+My.*Thzz.*t4.*t7+Thxxyy.*Thzz.*dqph.*dqth.*t13))./Thzz];

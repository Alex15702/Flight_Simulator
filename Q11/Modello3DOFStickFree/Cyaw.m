function [C_yaw] = Cyaw(alpha_B,C_L, beta_der,p, delta_r)

global aeroDat

%Coefficiente del timone calcolato in 1/deg, il delta_r và quindi passato
%in gradi
pyaw=convangvel(p,'rad/s','deg/s');
cyB=aeroDat{1,3}.cyb(1)*beta_der;
cyp=interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.cyp,alpha_B)*pyaw;
cypCL=aeroDat{1,3}.cypcl*C_L*p; %Incremento con il CL
cyr= -9E-5*delta_r;

C_yaw = cyB + cyp + cypCL + cyr;

end
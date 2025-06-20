function [CY_A] = CY_A(CL,beta_der,p)
global aeroDat
%Effetto del delta_r non modellato



CY_A = aeroDat{1,3}.cyb(1)*beta_der + aeroDat{1,3}.cypcl*CL*p;

end
function [C_yaw] = Cyaw(alpha_B,~, beta_der,p, delta_r)

global aeroDat

pyaw=convangvel(p,'rad/s','deg/s');
cnb=aeroDat{1,3}.cnb(1)*beta_der;
cnp=interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.cnp,alpha_B)*pyaw;
cnr= interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.cnr,alpha_B)*delta_r;
C_yaw = cnb + cnp + cnr;

end
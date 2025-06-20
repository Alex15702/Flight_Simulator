function [Croll] = Croll(alpha_B,beta_der,delta_a,delta_r,p,r)
global aeroDat
pr=convangvel(p,'rad/s','deg/s');
rr=convangvel(r,'rad/s','deg/s');
crollAlphaB=interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.clb, alpha_B)*beta_der;
crollDeltaa=interp1(aeroDat{1,2}.deltar,aeroDat{1,2}.clroll, delta_a);
crollp=interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.clp,alpha_B) * pr;
crollr=interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.clr,alpha_B) * rr;

Croll = crollAlphaB + crollDeltaa + crollp + crollr;
end

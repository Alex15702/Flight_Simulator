function [C_pitch] = Cpitch(alpha_B,delta_e,q)
global aeroDat

if alpha_B >=18
    error("Angolo d'attacco troppo elevato");
end
qp = convangvel(q,'rad/s','deg/s');
cmAlpha = interp1(aeroDat{1,3}.alpha,aeroDat{1,3}.cm,alpha_B);
dcmDelta = interp1(aeroDat{1,3}.delta,aeroDat{1,3}.dcm_sym, delta_e);
dcmq=aeroDat{1,3}.cmq(1)*qp;

C_pitch = cmAlpha + dcmDelta + dcmq;

end
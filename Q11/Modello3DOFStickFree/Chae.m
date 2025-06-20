function [Cha_e] = Chae(alpha_B, delta_e)

global aeroDat

%Includo l'effetto del downwash
epsilon = interp1(aeroDat{1,3}.alpha, aeroDat{1,3}.eps, alpha_B);
alpha_Elevator = alpha_B - epsilon;

Cha_e = -aeroDat{1,3}.cha_sym(1)*alpha_Elevator - aeroDat{1,3}.chd_sym(1)*delta_e;


end


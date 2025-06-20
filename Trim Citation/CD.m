function [C_D] = CD(alpha_B)
global aeroDat

delta_f=0;
alphaDat=aeroDat{1, 3}.alpha;
delta_fDat=aeroDat{1, 1}.delta;  %Deflessione flaps
cdDat=aeroDat{1, 3}.cd;          %Cd configurazione pulita
cdDeltafDat=aeroDat{1,1}.dcdi_sym;%Variazione dovuta ai flaps

cd=interp1(alphaDat,cdDat,alpha_B,'linear');

[A,B]=meshgrid(alphaDat,delta_fDat);

deltaCdflaps=interp2(A,B, cdDeltafDat', alpha_B, delta_f); %Interpolazione con alpha_B e deltaf


C_D=cd + deltaCdflaps;

end

function [C_L] = CL(alpha_B,delta_e)
global aeroDat

% Tengo conto sia dei flaps che dell'elevatore prelevando i dati dai due
% case di Datcom, case 1 flaps, case 2 elevatore

%Prelevo il Cl dal modello completo (2)


alphaDat=aeroDat{1, 3}.alpha;
delta_eDat=aeroDat{1, 3}.delta; %Deflessione elevatore
delta_fDat=aeroDat{1, 1}.delta;    %Deflessione flaps
clDat=aeroDat{1, 3}.cl;         %Cl configurazione pulita
clDeltaeDat=aeroDat{1,3}.dcl_sym;%Variazione dovuta all'elevatore
clDeltafDat=aeroDat{1,1}.dcl_sym;%Variazione dovuta ai flaps

cl=interp1(alphaDat,clDat,alpha_B,'linear');
dCldeltae=interp1(delta_eDat,clDeltaeDat,delta_e,'linear');
%dClflaps=interp1(delta_fDat,clDeltafDat,delta_f,'linear'); %Flaps non
%considerati

C_L=cl + dCldeltae; %+ dClflaps;


end
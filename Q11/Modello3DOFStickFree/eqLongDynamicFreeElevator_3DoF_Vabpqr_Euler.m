function [dStatedt] = eqLongDynamicFreeElevator_3DoF_Vabpqr_Euler(t,state)

%%
%Modello aerodinamico per volo a 3DOF per Citation I basato su dati DATCOM
%Come il 6DOF ma con meno 3 variabili di stato in meno



global g...
       delta_T delta_f ...
       myAC
   
%  Assegnazione delle componenti del vettore di stato
%  deltae diventa variabile di stato, equilibratore supposto staticamente
%  compensato

V = state(1);
alpha_B = state(2);
q = state(3);
xEG = state(4);
zEG = state(5);
theta = state(6);
u=V*cos(alpha_B);
w=V*sin(alpha_B);
delta_ePunto = state(7); 
delta_e = state(8); 

%% Quantità caratteristiche
%  Equazioni generali del moto del velivolo
omegatilde = [0,  0,  q;
              0,  0,  0;
             -q,  0,  0];

%  Equazioni cinematiche ausiliari
T_BE = angle2dcm(0,theta,0,'ZYX');
T_EB = T_BE';
T_gimb = [1,        0,             sin(theta)/cos(theta);
          0,        1,             0;
          0,        0,             1/cos(theta)];

%Proprietà termodinamiche
rho = density(-zEG);
   
%% Definizione delle forze e dei momenti agenti sul velivolo
%  Componenti in assi velivolo della forza risultante
C_L=CL(convang(alpha_B,'rad','deg'),...
                          convang(delta_e,'rad','deg'), convang(delta_f(t),'rad','deg'));
L = 0.5*rho*V^2*myAC.S*C_L;
D = 0.5*rho*V^2*myAC.S*CD(convang(alpha_B,'rad','deg'), convang(delta_f(t),'rad','deg'));

X_T = delta_T(t)*myAC.T_max_SL*cos(myAC.mu_T);
%Come richiesto, si assume che la spinta propulsiva massima disponibile sia
%costante al variare della quota e della velocità e che sia pari al valore
%della spinta propulsiva massima disponibile al livello del mare.
Z_T = delta_T(t)*myAC.T_max_SL*sin(myAC.mu_T);
X = X_T - D*cos(alpha_B) + L*sin(alpha_B) - myAC.W*sin(theta);
Z = Z_T - D*sin(alpha_B) - L*cos(alpha_B) + myAC.W*cos(theta);  

%  Componenti in assi velivolo del momento risultante
M_pitch_A = 0.5*rho*V^2*myAC.S*myAC.mac*Cpitch(convang(alpha_B,'rad','deg'),...
                                               convang(delta_e,'rad','deg'),...
                                               q);
C_M_T = (delta_T(t)*myAC.T_max_SL*myAC.e_T)/(0.5*rho*V^2*myAC.S*myAC.mac);
M_pitch_T = 0.5*rho*V^2*myAC.S*myAC.mac*C_M_T;

%Prime 3 equazioni
Ca=cos(alpha_B);
Sa=sin(alpha_B);
Cb=cos(0);
Sb=sin(0);
Ct=cos(theta);
St=sin(theta);
Cp=cos(0);
Sp=sin(0);
m=myAC.mass;
Vpunto=((Ca)/m)*X+Ca*q*w+Sa/m*Z+Sa*q*u;
alphapunto=1/(m*V)*(-L+Z_T*Ca-X_T*Sa+m*g*(Ca*Ct+Sa*St))+q;

%  Costruzione della funzione integranda

DVA=[Vpunto;alphapunto];
temp=omegatilde*myAC.I_matrix(2,2)*[0;q;0];
DQ=myAC.I_matrix(2,2)\(M_pitch_A+M_pitch_T-temp(2));
DXZ=T_EB*[u;0;w];DXZ=[DXZ(1);DXZ(3)];
DT=q;


% Dinamica dell'elevatore
%PHETE dal file di configurazione citation per datcom per ottenere LambdaeC
%CB dell'elevator per ottenere la corda media

LambdaeC = deg2rad(3);
Selev=1.17; %In m^2, calcolata a partire dai dati .dcm
Ihe = 4; %Stima rozza considerando l'elevatore un rettangolo
ce=convlength(0.84,'ft','m'); %Corda elevatore
Hce=0;
Hae=ce*Selev*0.5*rho*V^2*Chae(rad2deg(alpha_B),rad2deg(delta_e));




delta_eDuePunti = (cos(LambdaeC) * (DQ) + Hae + Hce)/Ihe;
delta_ePunto = delta_eDuePunti;



dStatedt=[DVA;DQ;DXZ;DT;delta_eDuePunti; delta_ePunto];


% dStatedt = ([-omegatilde,                      zeros(3);
%               zeros(3),       myAC.I_matrix\(-omegatilde*myAC.I_matrix);
%                T_EB,                           zeros(3);
%               zeros(3),                       T_gimb]*[u;
%                                                        v;
%                                                        w;
%                                                        p;
%                                                        q;
%                                                        r])+...
%            [(g/myAC.W)*X; 
%             (g/myAC.W)*Y;
%             (g/myAC.W)*Z; 
%             myAC.I_matrix\[L_roll_A + L_roll_T;
%                            M_pitch_A + M_pitch_T;
%                            N_yaw_A + N_yaw_T];
%             zeros(6,1)];
%dstatedt[12x1] = Matrix[12x6]*Matrix[6x1] + Matrix[12x1]
%t
%rad2deg(alpha_B)
t
end









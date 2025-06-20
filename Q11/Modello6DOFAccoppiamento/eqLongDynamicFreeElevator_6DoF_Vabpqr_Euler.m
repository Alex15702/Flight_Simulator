function [dStatedt] = eqLongDynamicFreeElevator_6DoF_Vabpqr_Euler(t,state)

%%




global g...
       delta_a delta_r delta_T delta_f ...
       myAC
   
%  Assegnazione delle componenti del vettore di stato
%  deltae diventa variabile di stato, equilibratore supposto staticamente
%  compensato

V = state(1);
alpha_B = state(2);
beta_der = state(3);
p = state(4);
q = state(5);
r = state(6);
xEG = state(7);
yEG = state(8);
zEG = state(9);
phi = state(10);
theta = state(11);
psi = state(12);
u=V*cos(alpha_B)*cos(beta_der);
v=V*sin(beta_der);
w=V*cos(beta_der)*sin(alpha_B);
delta_ePunto = state(13); 
delta_e = state(14); 

%% Quantità caratteristiche
%  Equazioni generali del moto del velivolo
omegatilde = [0, -r,  q;
              r,  0, -p;
             -q,  p,  0];

%  Equazioni cinematiche ausiliari
T_BE = angle2dcm(psi,theta,phi,'ZYX');
T_EB = T_BE';
T_gimb = [1,   sin(phi)*sin(theta)/cos(theta),   cos(phi)*sin(theta)/cos(theta);
          0,             cos(phi),                        -sin(phi);
          0,        sin(phi)/cos(theta),             cos(phi)/cos(theta)];

%Parametri cinematici
V = (u^2 + v^2 + w^2)^(1/2);
alpha_B = atan(w/u);
beta_der = asin(v/V);

%Proprietà termodinamiche
rho = density(-zEG);
   
%% Definizione delle forze e dei momenti agenti sul velivolo
%  Componenti in assi velivolo della forza risultante
C_L=CL(convang(alpha_B,'rad','deg'),...
                          convang(delta_e,'rad','deg'), convang(delta_f(t),'rad','deg'));
L = 0.5*rho*V^2*myAC.S*C_L;
D = 0.5*rho*V^2*myAC.S*CD(convang(alpha_B,'rad','deg'), convang(delta_f(t),'rad','deg'));

Y_A = 0.5*rho*V^2*myAC.S*CY_A(C_L,...
                              convang(beta_der,'rad','deg'),p);
X_T = delta_T(t)*myAC.T_max_SL*cos(myAC.mu_T);
%Come richiesto, si assume che la spinta propulsiva massima disponibile sia
%costante al variare della quota e della velocità e che sia pari al valore
%della spinta propulsiva massima disponibile al livello del mare.
Y_T = 0;
Z_T = delta_T(t)*myAC.T_max_SL*sin(myAC.mu_T);
X = X_T - D*cos(alpha_B) + L*sin(alpha_B) - myAC.W*sin(theta);                     
Y = Y_T + Y_A + myAC.W*sin(phi)*cos(theta);      
Z = Z_T - D*sin(alpha_B) - L*cos(alpha_B) + myAC.W*cos(phi)*cos(theta);  

%  Componenti in assi velivolo del momento risultante
L_roll_A = 0.5*rho*V^2*myAC.S*myAC.b*Croll(convang(alpha_B,'rad','deg'),...
                                           convang(beta_der,'rad','deg'),...
                                           convang(delta_a(t),'rad','deg'),...
                                           convang(delta_r(t),'rad','deg'),...
                                           p,r);
L_roll_T = 0;
M_pitch_A = 0.5*rho*V^2*myAC.S*myAC.mac*Cpitch(convang(alpha_B,'rad','deg'),...
                                               convang(delta_e,'rad','deg'),...
                                               q);
C_M_T = (delta_T(t)*myAC.T_max_SL*myAC.e_T)/(0.5*rho*V^2*myAC.S*myAC.mac);
M_pitch_T = 0.5*rho*V^2*myAC.S*myAC.mac*C_M_T;
N_yaw_A = 0.5*rho*V^2*myAC.S*myAC.b*Cyaw(convang(alpha_B,'rad','deg'),...
                                         C_L,...
                                         convang(beta_der,'rad','deg'),...
                                         p, ...
                                         convang(delta_r(t),'rad','deg'));
N_yaw_T = 0;





%Prime 3 equazioni
Ca=cos(alpha_B);
Sa=sin(alpha_B);
Cb=cos(beta_der);
Sb=sin(beta_der);
Ct=cos(theta);
St=sin(theta);
Cp=cos(phi);
Sp=sin(phi);
m=myAC.mass;
Vpunto=((Ca*Cb)/m)*X+Ca*Cb*(r*v-q*w)+Sb/m*Y+Sb*(p*w-r*u)+(Sa*Cb)/m*Z+(Sa*Cb)*(q*u-p*v);
alphapunto=1/(m*V*Cb)*(-L+Z_T*Ca-X_T*Sa+m*g*(Ca*Cp*Ct+Sa*St))+q-tan(beta_der)*(p*Ca+r*Sa);
betapunto=1/(m*V)*(D*Sb+Y_A*Cb-X_T*Ca*Sb+Y_T*Cb-Z_T*Sa*Sb+m*g*(Ca*Sb*St+Cb*Sp*Ct-Sa*Sb*Cp*Ct))+p*Sa-r*Ca;

% upunto=Vpunto*Ca*Cb;
% vpunto=Vpunto*Sb;
% wpunto=Vpunto*Sa*Cb;

%  Costruzione della funzione integranda

DVAB=[Vpunto;alphapunto;betapunto];
DPQR=myAC.I_matrix\([L_roll_A+L_roll_T;M_pitch_A+M_pitch_T;N_yaw_A+N_yaw_T]+...
    -omegatilde*myAC.I_matrix*[p;q;r]);
DXYZ=T_EB*[u;v;w];
DPTP=T_gimb*[p;q;r];


% Dinamica dell'elevatore
%PHETE dal file di configurazione citation per datcom per ottenere LambdaeC
%CB dell'elevator per ottenere la corda media

LambdaeC = deg2rad(3);
Selev=1.17; %In m^2, calcolata a partire dai dati .dcm
Ihe = 4; %Stima rozza considerando l'elevatore un rettangolo
ce=convlength(0.84,'ft','m');
Hce=0;
Hae=ce*Selev*0.5*rho*V^2*Chae(rad2deg(alpha_B),rad2deg(delta_e));




delta_eDuePunti = (cos(LambdaeC) * (DPQR(2)-p*r) + Hae + Hce)/Ihe;
delta_ePunto = delta_eDuePunti;



dStatedt=[DVAB;DPQR;DXYZ;DPTP;delta_eDuePunti; delta_ePunto];


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









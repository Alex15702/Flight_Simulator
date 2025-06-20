function [J_function] = funzioneObiettivo6DOF(x)

global g phi0 theta0 psi0 p0 q0 r0 V0  myAC zEG_0 %beta_der0
   
%  Assegnazione delle componenti del vettore di stato

alpha_B = x(1);
delta_e=x(2);
delta_a=x(3);
delta_r=x(4);
delta_T=x(5);
beta_der = x(6);

V = V0;
%beta_der = beta_der0;
p = p0;
q = q0;
r = r0;
zEG = zEG_0;
phi = phi0;
theta = theta0;
psi = psi0;
u=V*cos(alpha_B)*cos(beta_der);
v=V*sin(beta_der);
w=V*cos(beta_der)*sin(alpha_B);
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
                          convang(delta_e,'rad','deg'));
L = 0.5*rho*V^2*myAC.S*C_L;
D = 0.5*rho*V^2*myAC.S*CD(convang(alpha_B,'rad','deg'));

Y_A = 0.5*rho*V^2*myAC.S*CY_A(C_L,...
                              convang(beta_der,'rad','deg'),p);
X_T = delta_T*myAC.T_max_SL*cos(myAC.mu_T);
%Come richiesto, si assume che la spinta propulsiva massima disponibile sia
%costante al variare della quota e della velocità e che sia pari al valore
%della spinta propulsiva massima disponibile al livello del mare.
Y_T = 0;
Z_T = delta_T*myAC.T_max_SL*sin(myAC.mu_T);
X = X_T - D*cos(alpha_B) + L*sin(alpha_B) - myAC.W*sin(theta);                     
Y = Y_T + Y_A + myAC.W*sin(phi)*cos(theta);      
Z = Z_T - D*sin(alpha_B) - L*cos(alpha_B) + myAC.W*cos(phi)*cos(theta);  

%  Componenti in assi velivolo del momento risultante
L_roll_A = 0.5*rho*V^2*myAC.S*myAC.b*Croll(convang(alpha_B,'rad','deg'),...
                                           convang(beta_der,'rad','deg'),...
                                           convang(delta_a,'rad','deg'),...
                                           convang(delta_r,'rad','deg'),...
                                           p,r);
L_roll_T = 0;
M_pitch_A = 0.5*rho*V^2*myAC.S*myAC.mac*Cpitch(convang(alpha_B,'rad','deg'),...
                                               convang(delta_e,'rad','deg'),...
                                               q);
C_M_T = (delta_T*myAC.T_max_SL*myAC.e_T)/(0.5*rho*V^2*myAC.S*myAC.mac);
M_pitch_T = 0.5*rho*V^2*myAC.S*myAC.mac*C_M_T;
N_yaw_A = 0.5*rho*V^2*myAC.S*myAC.b*Cyaw(convang(alpha_B,'rad','deg'),...
                                         C_L,...
                                         convang(beta_der,'rad','deg'),...
                                         p, ...
                                         convang(delta_r,'rad','deg'));
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
% DXYZ=T_EB*[u;v;w];
% DPTP=T_gimb*[p;q;r];

J_function = sum([DVAB;DPQR].^2);



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

end
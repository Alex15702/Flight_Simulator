%% Dinamica laterodirezionale
global Nr_1 Nbeta_1 Np_1 Yr_1 U_0 Ybeta_1 Yp_1 g_0 Lr_1 Lbeta_1

clc; clear; close all;

% Flag that identifies the exercise
exercise = 'free rensponse';
%% Import data from Excel file and assign them to variables

Excel_data = readtable('Nominal_conditions_747.xlsx',"VariableNamingRule","preserve");

list = {'Condition 2','Condition 5','Condition 7','Condition 9','Condition 10'};

[indx,tf] = listdlg('PromptString',...
{'Which case do you want to analyze?',''},...
'SelectionMode','single','ListString',list);

switch indx
    
    case 1
          Condition = Excel_data.("Condition 2");
         
    case 2
          Condition = Excel_data.("Condition 5");
         
    case 3
          Condition = Excel_data.("Condition 7");
         
    case 4
          Condition = Excel_data.("Condition 9");
           
    case 5
          Condition = Excel_data.("Condition 10");
  
end

% Variables assignment
h            = Condition(1);
Mach_0       = Condition(2);
alpha_b      = Condition(3);
Weight_0     = Condition(4);
Iyy_0        = Condition(5);
C_L          = Condition(6);
C_D          = Condition(7);
C_L_alpha    = Condition(8);  % 1/rad
C_D_alpha    = Condition(9);  % 1/rad
C_m_alpha_0  = Condition(10); % 1/rad
C_L_alphadot = Condition(11); % 1/rad
C_m_alphadot = Condition(12); % 1/rad
C_L_q        = Condition(13); % 1/rad
C_m_q        = Condition(14); % 1/rad
C_L_Mach     = Condition(15);
C_D_Mach     = Condition(16);
C_m_Mach     = Condition(17);
C_L_de       = Condition(18); % 1/rad
C_m_de       = Condition(19); % 1/rad

%% DATA



g_0 = 9.81;
h_0 = convlength(h,'ft','m');
[T_0, a_0, p_0, rho_0] = atmosisa(h_0);
U_0     = Mach_0 * a_0;

SLUGFT2toKGM2 = convmass(1,'slug','kg')*(convlength(1,'ft','m')^2);
Iyy           = Iyy_0*SLUGFT2toKGM2;

Weight        = convforce(Weight_0,'lbf','N');
mass          = Weight/g_0;

S       = 5500*(convlength(1,'ft','m'))^2; % m^2
cbar    = convlength(27.3,'ft','m'); % m

qbar_0  = 0.5*rho_0*(U_0^2);
mu_0    = mass/(0.5*rho_0*S*cbar);
Gamma_0 = 0.0; % rad
SM_0    = 0.22;

%% Dinamica longitudinale
%Non fa parte del mio esercizio

SM        = 0.22; % Try also: 0.0158 0.0021 0.0 -0.0145
C_m_alpha = C_m_alpha_0*(SM/SM_0); % 1/rad
X_u       = -(qbar_0*S/(mass*U_0))*(2*C_D + Mach_0*C_D_Mach); % constant thrust
X_w       = (qbar_0*S/(mass*U_0))*(C_L - C_D_alpha);
X_wdot    = 0;
X_q       = 0;
X_de      = 0;
X_dT      = 0;
Z_u       = -(qbar_0*S/(mass*U_0))*( ...
              2*C_L + (Mach_0^2/(1-Mach_0^2))*C_L_Mach); % constant thrust
Z_w       = -(qbar_0*S/(mass*U_0))*(C_D + C_L_alpha);
Z_wdot    = -(1/(2*mu_0))*C_L_alphadot;
Z_q       = -(U_0/(2*mu_0))*C_L_q; 
Z_de      = 0;
Z_dT      = 0;

M_u       = (qbar_0*S*cbar/(Iyy*U_0))*Mach_0*C_m_Mach;
M_w       = (qbar_0*S*cbar/(Iyy*U_0))*C_m_alpha; 
M_wdot    = (rho_0*S*(cbar^2)/(4*Iyy))*C_m_alphadot;
M_q       = (rho_0*U_0*S*(cbar^2)/(4*Iyy))*C_m_q;
M_de      = 0;
M_dT      = 0;

k_hat     = M_wdot/(1-Z_wdot);
                                                      
A_lon = [ ...
X_u, X_w, 0, ...
-g_0*cos(Gamma_0); ...
Z_u/(1-Z_wdot), Z_w/(1-Z_wdot), (Z_q + U_0)/(1-Z_wdot), ...
-g_0*sin(Gamma_0)/(1-Z_wdot); ...
M_u + k_hat*Z_u, M_w+k_hat*Z_w, M_q + k_hat*(Z_q + U_0), ...
-k_hat*g_0*sin(Gamma_0); ...
0, 0, 1, ...
0 ];
% Input matrix ! cfr. (16.147c)
B_lon = [ ...
          X_de/mass, X_dT/mass; ...
          Z_de/(mass-Z_wdot), Z_dT/(mass-Z_wdot); ...
          (M_de + Z_de*M_wdot/(mass-Z_wdot))/Iyy, ...
          (M_dT + Z_dT*M_wdot/(mass-Z_wdot))/Iyy; ...
          0, 0];

% Output matrices
C_lon = eye(3,4);
D_lon = zeros(3,2);

%% Eigenvector, eigenvalues

[V,D]     = eig(A_lon);

W         = inv(V);

V_SP      = V(:,1);
V_SP      = V_SP/V_SP(4);
V_SP(1,1) = V_SP(1,1)/U_0;
V_SP(2,1) = V_SP(2,1)/U_0;
V_SP(3,1) = V_SP(3,1)/(2*U_0/cbar);

V_Ph      = V(:,3);
V_Ph      = V_Ph/V_Ph(4);
V_Ph(1,1) = V_Ph(1,1)/U_0;
V_Ph(2,1) = V_Ph(2,1)/U_0;
V_Ph(3,1) = V_Ph(3,1)/(2*U_0/cbar);


% right eigenvectors (columns) and eigenvalues
[V1,D1] = eig(A_lon);

% left eigenvectors (rows)
W1 = inv(V1);

% make a state-space representation
sys = ss( ...
A_lon, ... % A
zeros(size(A_lon,1),1), ... % B
eye(4,4), ... % C
zeros(4,1) ... % D
);

% Short-Period/Phugoid, normalized left eigenvector
% V1SP = V1(:,1);
% V1SP = V1SP/V1SP(4,1);
% V1Ph = V1(:,3);
% V1Ph = V1Ph/V1Ph(4,1);

% time response taking the initial state coincident
% with the original eigenvectors
x_init_1   = real(V_SP);
[y1,t1,x1] = initial(sys,x_init_1);

x_init_3   = real(V_Ph);
[y3,t3,x3] = initial(sys,x_init_3);
%%   Anonymous functions

% anonymous functions
zeta = @(sigma,omega) ...
sqrt(1/(1+(omega/sigma)^2));
omega_n = @(sigma,omega) ...
    -sigma/zeta(sigma,omega);
Period = @(sigma,omega) ...
2*pi/(omega_n(sigma,omega)*sqrt(1-(zeta(sigma,omega))^2));
t_half = @(sigma,omega) ...
log(2)/(omega_n(sigma,omega)*zeta(sigma,omega));
N_half = @(sigma,omega) ...
t_half(sigma,omega)/Period(sigma,omega);

% Short Period
lambda_SP  = D(1,1);
sigma_SP   = real(lambda_SP);
omega_SP   = imag(lambda_SP);
zeta_SP    = zeta(sigma_SP,omega_SP); % sqrt(1/(1+(omega_SP/sigma_SP)^2));
omega_n_SP = omega_n(sigma_SP,omega_SP); % -sigma_SP/zeta_SP;
T_SP       = Period(sigma_SP,omega_SP); % 2*pi/(omega_n_SP*sqrt(1-zeta_SP^2));
t_half_SP  = t_half(sigma_SP,omega_SP); % log(2)/(omega_n_SP*zeta_SP);
N_half_SP  = N_half(sigma_SP,omega_SP); % t_half_SP/T_SP;

% Phugoid
lambda_Ph  = D(3,3);
sigma_Ph   = real(lambda_Ph);
omega_Ph   = imag(lambda_Ph);
zeta_Ph    = sqrt(1/(1+(omega_Ph/sigma_Ph)^2));
omega_n_Ph = -sigma_Ph/zeta_Ph;
T_Ph       = 2*pi/(omega_n_Ph*sqrt(1-zeta_Ph^2));
t_half_Ph  = log(2)/(omega_n_Ph*zeta_Ph);
N_half_Ph  = t_half_Ph/T_Ph;

%% Dinamica laterodirezionale
b = convlength(195.68,'ft','m'); % m
% 'Condition 2', see data on B747
alphaB0 = convang(5.70,'deg','rad');
% Inertias
Ixx = 14.30e+6 * SLUGFT2toKGM2; % kg*m^2
Izz = 45.30e+6 * SLUGFT2toKGM2; % kg*m^2
Ixz = -2.23e+6 * SLUGFT2toKGM2; % kg*m^2
i1 = (Ixz/Ixx);
i2 = (Ixz/Izz);
% Lateral-Directional Stability Derivatives
% (dimensionless - along stability axes)
Clbeta = -0.221;
Clp = -0.450;
Clr = 0.101;
Cybeta = -0.96;
Cyp = 0;
Cyr = 0.0;
Cnbeta = 0.150;
CnTbeta = 0;
Cnp = -0.121;
Cnr = -0.300;
% Lateral-Directional Control Derivatives
% (dimensionless - along stability axes)
Cldeltaa = 0.0461;
Cldeltar = 0.007;
Cydeltaa = 0;
Cydeltar = 0.175;
Cndeltaa = 0.0064;
Cndeltar = -0.109;
% Lateral-Directional Dimensional Stability Derivatives
Ybeta = (qbar_0*S*Cybeta)/mass;
Yp = (qbar_0*S*b*Cyp)/(2*mass*U_0);
Yr = (qbar_0*S*b*Cyr)/(2*mass*U_0);
Lbeta = (qbar_0*S*b*Clbeta)/Ixx;
Lp = (qbar_0*S*(b^2)*Clp)/(2*Ixx*U_0);
Lr = (qbar_0*S*(b^2)*Clr)/(2*Ixx*U_0);
Nbeta = (qbar_0*S*b*Cnbeta)/Izz;
NTbeta = (qbar_0*S*b*CnTbeta)/Izz;
Np = (qbar_0*S*(b^2)*Cnp)/(2*Izz*U_0);
Nr = (qbar_0*S*(b^2)*Cnr)/(2*Izz*U_0);
% Lateral-Directional Dimensional Control Derivatives
Ydeltaa = (qbar_0*S*Cydeltaa)/mass;
Ydeltar = (qbar_0*S*Cydeltar)/mass;
Ldeltaa = (qbar_0*S*b*Cldeltaa)/Ixx;
Ldeltar = (qbar_0*S*b*Cldeltar)/Ixx;
Ndeltaa = (qbar_0*S*b*Cndeltaa)/Izz;
Ndeltar = (qbar_0*S*b*Cndeltar)/Izz;
% Lateral-Directional Primed Stability Derivatives
Ybeta_1 = Ybeta;
Yp_1 = Yp;
Yr_1 = Yr;
Lbeta_1 = (Lbeta + i1*Nbeta)/(1-i1*i2);
Lp_1 = (Lp + i1*Np)/(1-i1*i2);
Lr_1 = (Lr + i1*Nr)/(1-i1*i2);
Nbeta_1 = (i2*Lbeta + Nbeta)/(1-i1*i2);
Np_1 = (i2*Lp + Np)/(1-i1*i2);
Nr_1 = (i2*Lr + Nr)/(1-i1*i2);
% Lateral-Directional Primed Control Derivatives
Ydeltaa_1 = Ydeltaa;
Ydeltar_1 = Ydeltaa;
Ldeltaa_1 = (Ldeltaa + i1*Ndeltaa)/(1-i1*i2);
Ldeltar_1 = (Ldeltar + i1*Ndeltar)/(1-i1*i2);
Ndeltaa_1 = (i2*Ldeltaa + Ndeltaa)/(1-i1*i2);
Ndeltar_1 = (i2*Ldeltar + Ndeltar)/(1-i1*i2);
% Lateral-Directional matrices

%% Qui inizia il mio lavoro


% Rendo Ald una funzione così che possa variare i valori di Lp

    A_ldf = @(Lp_1) [ ...
Nr_1, Nbeta_1, Np_1, 0; ...
Yr_1/U_0 - 1, Ybeta_1/U_0, Yp_1/U_0, g_0/U_0; ...
Lr_1, Lbeta_1, Lp_1, 0; ...
0, 0, 1, 0];

 


A_ld = A_ldf(Lp_1);

B_ld = [ ...
Ndeltaa_1, Ndeltar_1; ...
Ydeltaa_1/U_0, Ydeltar_1/U_0; ...
Ldeltaa_1, Ldeltar_1; ...
0, 0];

%% Caratteristiche del 747 per i moti puri
%Non interessano il mio esercizio
% Lateral-Directional eigenvalues/eigenvectors
% [Vld,Dld] = eig(A_ld); % right eigenvectors (columns) and eigenvalues
% Vld_Roll = Vld(:,1);
% Vld_Roll = Vld_Roll/Vld_Roll(4,1);
% Vld_Spiral = Vld(:,4);
% Vld_Spiral = Vld_Spiral/Vld_Spiral(4,1);
% lambda_DR = Dld(2,2);
% Vld_DR = Vld(:,2);
% Vld_DR = Vld_DR/Vld_DR(4,1);
% sigma_DR = real(lambda_DR);
% omega_DR = imag(lambda_DR);
% zeta_DR = zeta(sigma_DR,omega_DR);
% omega_n_DR = omega_n(sigma_DR,omega_DR);
% T_DR = Period(sigma_DR,omega_DR);
% t_half_DR = t_half(sigma_DR,omega_DR);
% N_half_DR = N_half(sigma_DR,omega_DR);

%% Risposta del 747 a deflessioni alterne degli alettoni
% Esercizio 16.8
%Fatto da me
%Costruisco la matrice del sistema

SistLatDir = sysLatDir(A_ld, B_ld);

x_init   = zeros(4,1);

% Import delle leggi di comando
comandi = load('LeggiDiComando.mat');
delta_a = @(t) deg2rad(interp1(comandi.delta_a.Time,comandi.delta_a.Data,t,"previous","extrap"));
delta_r = @(t) deg2rad(interp1(comandi.delta_r.Time,comandi.delta_r.Data,t,"previous","extrap"));




% Visualizzazione impulso alettoni
Nt=300;
Times = linspace(0,100,Nt);
comandi = [delta_a(Times)', zeros(Nt,1)];
y_Alettoni = lsim(SistLatDir,comandi,Times,x_init);
%Impulso sul timone
comandi = [zeros(Nt,1), delta_r(Times)'];
y_Rudder = lsim(SistLatDir,comandi,Times,x_init);





%% Risposta ad una raffica lineare
%Raffica sostenuta per t<0, la raffica si interrompe al tempo 0, valutiamo
%la stabilità dinamica del velivolo a questo fenomeno

wTip = 1; %velocità vista dall'ala destra
b=59.6; %Wingspan del 747
p=wTip*2/b;%Velocità di rotazione p che darebbe la stessa velocità del vento alla tip

x0Raffica=[0,0,p,0];
Times = 0:0.5:100;
y_raffica=initial(SistLatDir,x0Raffica, Times);

figure(1);
plot(Times, y_raffica,LineWidth=1); % Risposta dell'aereo
legend('r','\beta','p','\phi');

% Prova a vedere come varia all'aumentare in valore assoluto di Clp (Roll)
% Significa aumentare Lp

% 10% maggiore
A_ld1 = A_ldf(Lp_1*1.1);
s1=sysLatDir(A_ld1,B_ld);
y_raffica1=initial(s1,x0Raffica, Times);
% 20% maggiore
A_ld2 = A_ldf(Lp_1*1.2);
s2=sysLatDir(A_ld2,B_ld);
y_raffica2=initial(s2,x0Raffica, Times);
% 30% maggiore
A_ld3 = A_ldf(Lp_1*1.3);
s3=sysLatDir(A_ld3,B_ld);
y_raffica3=initial(s3,x0Raffica, Times);
% 40% maggiore
A_ld4 = A_ldf(Lp_1*1.4);
s4=sysLatDir(A_ld4,B_ld);
y_raffica4=initial(s4,x0Raffica, Times);
% 50% maggiore
A_ld5 = A_ldf(Lp_1*1.5);
s5=sysLatDir(A_ld5,B_ld);
y_raffica5=initial(s5,x0Raffica, Times);

%Autovalori
figure(10);
realValues=[real(eig(A_ld)),real(eig(A_ld1)),real(eig(A_ld2)),real(eig(A_ld3)),real(eig(A_ld4)),real(eig(A_ld5))];
imaginaryValues=[imag(eig(A_ld)),imag(eig(A_ld1)),imag(eig(A_ld2)),imag(eig(A_ld3)),imag(eig(A_ld4)),imag(eig(A_ld5))];
scatter(realValues,imaginaryValues,MarkerEdgeColor="flat",MarkerFaceColor="flat");
legend("0%","10%","20%","30%","40%","50%","Location","northwest");
grid on;
%Plotting dei risultati

%Valori di r
figure(2);
plot(Times, [y_raffica(:,1),y_raffica1(:,1),y_raffica2(:,1),y_raffica3(:,1),y_raffica4(:,1),y_raffica5(:,1)]);
legend('0%','10%','20%','30%','40%','50%');
title('Grafico di r');
%Valori di beta
figure(3);
plot(Times, [y_raffica(:,2),y_raffica1(:,2),y_raffica2(:,2),y_raffica3(:,2),y_raffica4(:,2),y_raffica5(:,2)]);
legend('0%','10%','20%','30%','40%','50%');
title('Grafico di \beta');
%Valori di p
figure(4);
plot(Times, [y_raffica(:,3),y_raffica1(:,3),y_raffica2(:,3),y_raffica3(:,3),y_raffica4(:,3),y_raffica5(:,3)]);
legend('0%','10%','20%','30%','40%','50%');
title('Grafico di p');
%Valori di phi
figure(5);
plot(Times, [y_raffica(:,4),y_raffica1(:,4),y_raffica2(:,4),y_raffica3(:,4),y_raffica4(:,4),y_raffica5(:,4)]);
legend('0%','10%','20%','30%','40%','50%');
title('Grafico di \phi');
%% Controllo

%Implementazione di un controllo in retroazione sul valore di p che và ad
%impartire una deflessione degli alettoni 
%PID Tuned con il control system design toolkit di simulink

sistemaConControllo=sim('SistemaPID.slx');
figure(6);
Evoluzione = sistemaConControllo.yout.getElement('systemEvolution');
Evoluzione = Evoluzione.Values;
stato = interp1(Evoluzione.Time, Evoluzione.Data, 0:0.1:Evoluzione.Time(end),'linear');
plot(0:0.1:Evoluzione.Time(end), stato,LineWidth=1);
title('Dinamica del velivolo con controllo in retroazione');
legend('r','\beta','p','\phi');

figure(7)
Alettoni = sistemaConControllo.yout.getElement('deflessioneAlettoni');
Alettoni = Alettoni.Values;
defAlettoni = interp1(Alettoni.Time, rad2deg(Alettoni.Data), 0:0.1:Alettoni.Time(end),'linear');
plot(0:0.1:Evoluzione.Time(end), defAlettoni,LineWidth=1);
title('Deflessione degli alettoni');


function SistLatDir = sysLatDir(A_ld , B_ld)

    SistLatDir = ss( ...
    A_ld, ... % A
    B_ld, ... % B
    eye(4,4), ... % C
    zeros(4,2) ... % D
    );

end
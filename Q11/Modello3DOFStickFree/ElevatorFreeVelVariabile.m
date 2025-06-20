%% Simulazione del moto a 3-DOf a comandi liberi
%Cessna Citation I

%Voglio osservare l'effetto dell'accoppiamento tra la dinamica del velivolo
%e quella dell equilibratore

clear all; close all; clc;

disp('Moto del velivolo a 6 gradi di libertà');
%% Dichiarazione delle variabili globali
global g...                  %Accelerazione di gravità 
       myAC...
       aeroDat

%  Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'CITATION.txt';
myAC = DSVAircraft(aircraftDataFileName);
aeroDat=datcomimport("CitationM025.out");


%% Interagisce con modello 3DOF

%  Costanti e condizioni iniziali   
g = 9.81; %Accelerazione di gravità [m/s^2]
xEG_0 = 0; %[m]
zEG_0 = -1500; %Altitudine [m]
[air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
theta0 = convang(4.460,'deg','rad'); %Angolo di elevazione
q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
M0 = 0.25; %Numero di Mach
V0 = M0*sound_speed0; %Velocità lineare del baricentro
alpha_B0 = convang(0.5,'deg','rad'); %Angolo di attacco valutato rispetto l'asse x Body
u0 = V0*cos(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse x Body
w0 = V0*sin(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse z Body

%% Metodo alternativo per la definizione delle leggi di comando mediante il Matlab Signal Editor
%Permette di creare in maniera veloce diversi scenari di volo
global delta_T delta_f
flightScenario = load("Scenario.mat");

t_fin = 160;

delta_T = @(t) interp1(flightScenario.delta_tSignal.Time,flightScenario.delta_tSignal.Data,t,'previous','extrap');
delta_f = @(t) deg2rad(interp1(flightScenario.delta_fSignal.Time,flightScenario.delta_fSignal.Data,t,'linear','extrap'));




%  Integrazione del sistema di equazioni differenziali
delta_epunto0=convangvel(0,'deg/s','rad/s');
delta_e0=0;
state_0 = [V0,alpha_B0,q0,xEG_0,zEG_0,theta0,delta_epunto0,delta_e0];
%options = odeset('RelTol',1e-9,'AbsTol',1e-9*ones(1,8));
[vTime,mState] = ode45(@eqLongDynamicFreeElevator_3DoF_Vabpqr_Euler,[0 t_fin],state_0);


% Vettori per la stampa

vVel_u = mState(:,1)*cos(mState(2));
vVel_w = mState(:,1)*sin(mState(2));
vVel = mState(:,1);
vAlpha_B = rad2deg(mState(:,2));
v_q = convangvel(mState(:,3),'rad/s','deg/s');
vXe = mState(:,4);
vZe = mState(:,5);
vTheta = convang(mState(:,6),'rad','deg');
vDelta_e = @(t) interp1(vTime,convang(mState(:,8),'rad','deg'),t,"linear");



%% Grafica
%  Diagrammi dei comandi di volo
figure(1)
subplot 411
plot(vTime,convang(vDelta_e(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('$\delta_{e} (deg)$','interpreter','latex','fontsize',11);
subplot 412
plot(vTime,vVel,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('V (m/s)','interpreter','latex','fontsize',11);
subplot 413
plot(vTime,vAlpha_B,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('$\alpha_{B} (deg)$','interpreter','latex','fontsize',11);
subplot 414
plot(vTime,v_q,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('q (deg/s)','interpreter','latex','fontsize',11);

figure(2)
subplot 411
plot(vTime,-vZe,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('Z','interpreter','latex','fontsize',11);
subplot 412
plot(vTime,vXe,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('X (deg)','interpreter','latex','fontsize',11);
subplot 413
plot(vTime,vTheta,'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('$\theta (deg)$','interpreter','latex','fontsize',11);
subplot 414
plot(vTime,delta_T(vTime),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylabel('$\delta_T$','interpreter','latex','fontsize',11);

%% Traiettoria e modello
plot = true;
if plot == true
    % Setup the figure/scene
    h_fig1 = figure(6);
    light('Position',[3000 3000 -5500],'Style','local');
    % Trick to have Ze pointing downward and correct visualization
    set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
    grid on; hold on;
    % Load aircraft shape
    shapeScaleFactor = 450.0;
    shape = loadAircraftMAT('Embraer-Phenom.mat', shapeScaleFactor); 
    % Set the aircraft in place
    % Posision in Earth axes
    
    vXYZe = [vXe, vXe*0, vZe]; 
    % psi, theta, phi -> 'ZYX'
    vEulerAngles=[deg2rad(vTheta*0), deg2rad(vTheta+180), deg2rad(vTheta*0)];
    % Observer point-of-view
    theView = [105 15];
    % body axes settings
    bodyAxesOptions.show = true; 
    bodyAxesOptions.magX = 2.0*shapeScaleFactor;
    bodyAxesOptions.magY = 2.0*shapeScaleFactor;
    bodyAxesOptions.magZ = 1.5*shapeScaleFactor;
    bodyAxesOptions.lineWidth = 2.5;
    
    
    % Plot Earth axes
    hold on;
    xMax = 1;
    yMax = 1;
    zMax = 1; % max([abs(max(vXYZe(1))),0.18*xMax]);
    vXYZ0 = [xEG_0,0,zEG_0];
    vExtent = [xMax,yMax,zMax];
    plotEarthAxes(h_fig1, vXYZ0, vExtent);
    options.samples=[1,5,10,24,30,38].*1000;
    options.theView=theView;
    options.bodyAxes.show = 1; 
    options.bodyAxes.magX = 2.0*shapeScaleFactor;
    options.bodyAxes.magY = 2.0*shapeScaleFactor;
    options.bodyAxes.magZ = 2.0*shapeScaleFactor;
    options.bodyAxes.lineWidth = 2.5;
    options.helperLines.show= 1;
    options.helperLines.lineColor='k';
    options.helperLines.lineWidth=1.0;
    options.helperLines.lineStyle='-';
    options.trajectory.show = 1;
    options.trajectory.lineColor = 'blue';
    options.trajectory.lineWidth=1.0;
    options.trajectory.lineStyle = '--';
    plotTrajectoryAndBodyE(h_fig1,shape,vXYZe,vEulerAngles, options);
end





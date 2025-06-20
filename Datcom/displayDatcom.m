%% Script per il display dei dati ottenuti da Digital Datcom per il Citation I
close all; clc;
%Import dati

aeroDatM025 = datcomimport("CitationM025.out");
aeroDatM05 = datcomimport("CitationM05.out");

% CL vs alpha vs Mach vs deltaflap
set(0, 'DefaultLineLineWidth', 1.5);
figure(1);
title("CL Citation I")
hold on;
grid on;
plot(aeroDatM025{1,1}.alpha,aeroDatM025{1,1}.cl,Color="blue",LineStyle="-");
plot(aeroDatM025{1,1}.alpha,aeroDatM025{1,1}.cl+aeroDatM025{1,1}.dcl_sym(2),Color="blue",LineStyle=":");%5
plot(aeroDatM025{1,1}.alpha,aeroDatM025{1,1}.cl+aeroDatM025{1,1}.dcl_sym(3),Color="blue",LineStyle="--");%10
plot(aeroDatM05{1,1}.alpha,aeroDatM05{1,1}.cl,Color="red",LineStyle="-");
plot(aeroDatM05{1,1}.alpha,aeroDatM05{1,1}.cl+aeroDatM05{1,1}.dcl_sym(2),Color="red",LineStyle=":");%5
plot(aeroDatM05{1,1}.alpha,aeroDatM05{1,1}.cl+aeroDatM05{1,1}.dcl_sym(3),Color="red",LineStyle="--");%10
ylabel("CL");
xlabel("\alpha");
yline(0,Color='black');
xline(0,Color='black');
xlim([-15,15]);
legend("M=0.25, \delta_f = 0 (deg)","M=0.25, \delta_f = 5 (deg)","M=0.25, \delta_f = 10 (deg)", ...
    "M=0.5, \delta_f = 0 (deg)","M=0.5, \delta_f = 5 (deg)","M=0.5, \delta_f = 10 (deg)",'Location','northwest');

%Cd vs alpha? vs flap





figure(2);
title("CD Citation I")
hold on;
grid on;
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM025,aeroDatM025{1,1}.alpha,0),Color="blue",LineStyle="-");%0
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM025,aeroDatM025{1,1}.alpha,5),Color="blue",LineStyle=":");%5
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM025,aeroDatM025{1,1}.alpha,10),Color="blue",LineStyle="--");%10
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM05,aeroDatM025{1,1}.alpha,0),Color="red",LineStyle="-");%0
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM05,aeroDatM025{1,1}.alpha,5),Color="red",LineStyle=":");%5
plot(aeroDatM025{1,1}.alpha,CD(aeroDatM05,aeroDatM025{1,1}.alpha,10),Color="red",LineStyle="--");%10
ylabel("CD");
xlabel("\alpha");
yline(0,Color='black');
xline(0,Color='black');
xlim([-15,15]);
legend("M=0.25, \delta_f = 0 (deg)","M=0.25, \delta_f = 5 (deg)","M=0.25, \delta_f = 10 (deg)", ...
    "M=0.5, \delta_f = 0 (deg)","M=0.5, \delta_f = 5 (deg)","M=0.5, \delta_f = 10 (deg)",'Location','northwest');


%Cm vs alpha


figure(3);
title("Momento di beccheggio Citation I")
hold on;
grid on;
cm025=aeroDatM025{1,3}.cm;
cm05=aeroDatM05{1,3}.cm;
cma025=aeroDatM025{1,3}.cma;
cma05=aeroDatM05{1,3}.cma;
plot(aeroDatM025{1,1}.alpha(1:12),cm025(1:12),Color="blue",LineStyle="-");
plot(aeroDatM025{1,1}.alpha(1:12),cma025(1:12),Color="blue",LineStyle="--");
plot(aeroDatM025{1,1}.alpha(1:12),cm05(1:12),Color="red",LineStyle="-");
plot(aeroDatM025{1,1}.alpha(1:12),cma05(1:12),Color="red",LineStyle="--");
xlabel("\alpha");
yline(0,Color='black');
xline(0,Color='black');
xlim([-12,12]);
legend("CM, M=0.25","CM_{\alpha} ,M=0.25","CM, M=0.5","CM_{\alpha} ,M=0.5",'Location','northeast');


%Controllo al rollio alettoni

figure(4);
title("Momento di rollio dovuto alla deflessione degli alettoni Citation I")
hold on;
grid on;
plot(aeroDatM025{1,2}.deltar,aeroDatM025{1,2}.clroll,Color="blue",LineStyle="-");
xlabel("\delta_a");
ylabel("C_{roll}");
yline(0,Color='black');
xline(0,Color='black');
legend("C_{roll}, M=0.25");

%Controllo all'imbardata timone

figure(5);
title("Momento di imbardata dovuto alla deflessione del timone Citation I")
hold on;
grid on;
plot(aeroDatM025{1,3}.alpha,aeroDatM025{1,3}.cnr,Color="blue",LineStyle="-");
xlabel("\alpha");
ylabel("C_{\delta_r}");
yline(0,Color='black');
xline(0,Color='black');
xlim([-12,12]);
legend("C_{yaw}, M=0.25");

%Controllo al beccheggio timone
figure(6);
title("Variazione del Momento di beccheggio dovuto alla deflessione dell elevatore Citation I")
hold on;
grid on;
plot(aeroDatM025{1,3}.delta,aeroDatM025{1,3}.dcm_sym,Color="blue",LineStyle="-");
xlabel("\delta_e");
ylabel("\Delta_{C_{\delta_e}}");
yline(0,Color='black');
xline(0,Color='black');
%xlim([-12,12]);
legend("\Delta_{C_{\delta_e}}, M=0.25");

%Downwash
figure(7);
title("Downwash visto dal piano orizzontale di coda Citation I")
hold on;
grid on;
plot(aeroDatM025{1,3}.alpha,aeroDatM025{1,3}.eps,Color="blue",LineStyle="-");
xlabel("\alpha");
ylabel("\epsilon (deg)");
yline(0,Color='black');
xline(0,Color='black');
xlim([-12,12]);
legend("\epsilon, M=0.25");

function C_D = CD(aeroDat,alpha_B,delta_f)

alphaDat=aeroDat{1, 3}.alpha;
delta_fDat=aeroDat{1, 1}.delta;  %Deflessione flaps
cdDat=aeroDat{1, 3}.cd;          %Cd configurazione pulita
cdDeltafDat=aeroDat{1,1}.dcdi_sym;%Variazione dovuta ai flaps
cd=interp1(alphaDat,cdDat,alpha_B,'linear');
[A,B]=meshgrid(alphaDat,delta_fDat);
deltaCdflaps=interp2(A,B, cdDeltafDat', alpha_B, delta_f); %Interpolazione con alpha_B e deltaf
C_D=cd + deltaCdflaps;

end


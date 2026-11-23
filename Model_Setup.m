%% Model_Setup.m -- VehicleDiagram.slx parametreleri (S1: Plant, acik cevrim)
% Tum degiskenler base workspace'e yazilir. Testler bu betigi calistirip
% degiskenleri (base'te) ezdikten sonra sim('VehicleDiagram') cagirabilir.
% Not: bilincli olarak 'clear' yok; bir fonksiyon icinden cagrilinca
% cagiranin calisma alanini silmesin.

%% --- Arac parametreleri ---
m     = 1500;         % kutle [kg]
Iz    = 2250;         % yaw ataleti [kg m^2]
L     = 2.7;          % dingil mesafesi [m]
lr    = 1.35;         % CG -> arka aks [m]
lf    = L - lr;       % CG -> on aks [m]
Cf    = 2*19000;      % on aks kose rijitligi [N/rad] (aks basina)
Cr    = 2*19000;      % arka aks kose rijitligi [N/rad] (aks basina)

%% --- Direnc ve yol ---
rho_air    = 1.225;   % hava yogunlugu [kg/m^3]
CdA        = 0.66;    % suruklenme katsayisi x on alan [m^2]
C_rr       = 0.015;   % yuvarlanma direnci katsayisi
g          = 9.81;    % yercekimi [m/s^2]
theta_road = 0;       % yol egimi [rad] (+ = yokus yukari)

%% --- Plant ayarlari ---
vx_floor    = 1;      % kayma acisinda bolme tabani [m/s]
brake_split = 0.6;    % on aks fren payi (F_x < 0); itis yalniz arkadan

%% --- Simulasyon ---
Ts_plant = 1e-3;      % sabit adim [s] (ode4)
T_end    = 20;        % bitis zamani [s]

%% --- Baslangic kosullari ---
vx0_init = 8;         % boyuna hiz [m/s]
vy0_init = 0;         % yanal hiz [m/s]
r0_init  = 0;         % yaw hizi [rad/s]
X0       = 0;         % global konum X [m]
Y0       = 0;         % global konum Y [m]
psi0     = 0;         % yaw acisi [rad]

%% --- Test girisleri (From Workspace, [t u]; son degerde tutulur) ---
Fx_in    = [0 250; T_end 250];   % toplam boyuna kuvvet [N]
delta_in = [0 0;   T_end 0];     % on teker direksiyon acisi [rad]

%% --- Workspace ---
names = {'m','Iz','L','lr','lf','Cf','Cr', ...
         'rho_air','CdA','C_rr','g','theta_road', ...
         'vx_floor','brake_split','Ts_plant','T_end', ...
         'vx0_init','vy0_init','r0_init','X0','Y0','psi0', ...
         'Fx_in','delta_in'};
for k = 1:numel(names)
    assignin('base', names{k}, eval(names{k}));
end
clear k names

fprintf('Model_Setup: parametreler base workspace''e yuklendi (T_end = %g s, Ts_plant = %g s).\n', ...
        T_end, Ts_plant);

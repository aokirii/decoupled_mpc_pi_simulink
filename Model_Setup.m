%% Model_Setup.m -- VehicleDiagram.slx parametreleri (S1 Plant, S2 Referans + Hata Hesabi, S3 MPC)
% Tum degiskenler base workspace'e yazilir. Calistirdiktan sonra degiskenler
% (base'te) ezilerek sim('VehicleDiagram') cagrilabilir.
% Not: bilincli olarak 'clear' yok; bir fonksiyon icinden cagrilinca
% cagiranin calisma alanini silmesin.
% Harita: map_name ('Circle' | 'Rectangle' | 'Hand') cagiranin calisma
% alaninda varsa o kullanilir, yoksa 'Rectangle'. Hand ginput ister
% (yalniz elle kullanim).

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
mu          = 0.75;   % yol tutus katsayisi (lonlat; Dugoff doymasi, surtunme cemberi)
h           = 0.55;   % CG yuksekligi [m] (lonlat, yuk transferi)

%% --- Simulasyon ---
Ts_plant = 1e-3;      % sabit adim [s] (ode4)
Ts_ctrl  = 0.02;      % kontrol / referans ornekleme suresi [s] (Ts_plant'in tam kati)
T_end    = 300;       % ust sinir [s]; kapali dongude kosu Bitis blogunda biter (3 tur: Rectangle 134 s, Circle 283 s)

%% --- Referans yol ve hata hesabi (S2) ---
% Harita -> interpolasyon yontemi / kose yaricapi (diyagram-simulink-matematik.md 3.2)
if ~exist('map_name', 'var') || isempty(map_name), map_name = 'Rectangle'; end
switch map_name
    case 'Circle',    ms_map__ = {'YuvarlakMap.m',   'spline', 0};
    case 'Rectangle', ms_map__ = {'KareMap.m',       'spline', 8};
    case 'Hand',      ms_map__ = {'HandFigureMap.m', 'pchip',  0};
    otherwise, error('Model_Setup: map_name ''Circle'', ''Rectangle'' ya da ''Hand'' olmali.');
end
[waypoints, startPt] = ms_load_map__(fullfile(fileparts(mfilename('fullpath')), 'maps', ms_map__{1}));
prm_ref = struct('ds_target', 0.1, 'corner_radius', ms_map__{3}, ...
                 'interp_method', ms_map__{2}, 'v_max', 8, 'ay_max', 0.3*g, ...   % olagan surus: a_y 0.3 g, fren -2, hizlanma 1.5 m/s^2
                 'ax_max', 1.5, 'ax_min', -2, 'v_min', 2, 'kappa_win', 3);
clear ms_map__
ref = reference_path(waypoints, startPt, prm_ref);   % yol uzunlugu ref.L (L = dingil mesafesi)
win_fwd  = 120;       % en yakin nokta arama penceresi, ileri [dugum]
win_back = 20;        % geri [dugum]
fin = struct('n_laps', 3, 'v_stop', 0.05, 't_hold', 0.5);   % bitis olcutleri (Ts = Ts_ctrl ayri)
stop_enable = 1;      % 0: bitis mantigi kapali

%% --- Kontrolculer (S3) ---
% Yanal MPC: x = [e_y e_y' (e_psi - e_psi,ss) e_psi'], giris delta (lat_mpc)
mpc_lat = struct('Np', 30, 'Q', diag([70 10 150 30]), 'R', 0, 'Rd', 1, ...
    'ulim', deg2rad([-30 30]), 'dumax', 1.5*Ts_ctrl, 'isoft', 0, 'ws', 1, ...
    'vx_floor', vx_floor, 'Cf', Cf, 'Cr', Cr, 'lf', lf, 'lr', lr, 'm', m, 'Iz', Iz);
% Boyuna MPC: x = [e_x e_v], giris a = F_x/m; yumusak sinir v_x <= yol hizi (isoft = 2)
mpc_lon = struct('Np', 30, 'Q', diag([150 10]), 'R', 0, 'Rd', 1, ...
    'ulim', [-3 2], 'dumax', 2.5*Ts_ctrl, 'isoft', 2, 'ws', 1e4, ...   % olagan surus: jerk 2.5 m/s^3
    'ay_lim', 0.75*mu*g, 'lr', lr, 'lf', lf, 'Cf', Cf, 'Cr', Cr, 'm', m, 'rho_air', rho_air, 'CdA', CdA, 'C_rr', C_rr, 'g', g);

%% --- Baslangic kosullari ---
vx0_init = ref.v(ref.idx0);   % boyuna hiz [m/s] (hiz profilinden)
vy0_init = 0;                 % yanal hiz [m/s]
r0_init  = 0;                 % yaw hizi [rad/s]
X0       = ref.x(ref.idx0);   % global konum X [m] (yolun baslangic noktasi)
Y0       = ref.y(ref.idx0);   % global konum Y [m]
psi0     = ref.theta0;        % yaw acisi [rad] (yol yonu)

%% --- Workspace ---
ms_names__ = {'m','Iz','L','lr','lf','Cf','Cr', ...
         'rho_air','CdA','C_rr','g','theta_road', ...
         'vx_floor','brake_split','mu','h','Ts_plant','T_end', ...
         'vx0_init','vy0_init','r0_init','X0','Y0','psi0', ...
         'Ts_ctrl','map_name','waypoints','startPt','prm_ref','ref', ...
         'win_fwd','win_back','fin','stop_enable', ...
         'mpc_lat','mpc_lon'};
% Dongu/liste degiskenleri cagiranin degiskenleriyle cakismasin diye
% benzersiz adli; yalniz bunlar temizlenir.
for ms_i__ = 1:numel(ms_names__)
    assignin('base', ms_names__{ms_i__}, eval(ms_names__{ms_i__}));
end
clear ms_i__ ms_names__

fprintf(['Model_Setup: parametreler base workspace''e yuklendi (T_end = %g s, Ts_plant = %g s, ' ...
         'harita %s, L_yol = %.2f m).\n'], T_end, Ts_plant, map_name, ref.L);

%% ======================================================================
function [wp, sp] = ms_load_map__(file)
% Harita betigini fonksiyon calisma alaninda calistirir; ara degiskenleri
% (C, P, N, x, y, ...) base'e sizmaz. Ancak harita betiklerinin kendi
% assignin('base', ...) cagrilari waypoints, startPt ve goal'u base'e
% yazar (goal S2'de kullanilmaz). Yalniz haritanin actigi figurler kapanir.
figs0 = findall(groot, 'Type', 'figure');
run(file);
wp = eval('waypoints');
sp = eval('startPt');
close(setdiff(findall(groot, 'Type', 'figure'), figs0));
end

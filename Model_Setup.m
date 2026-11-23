%% Model_Setup.m -- VehicleDiagram.slx parametreleri (S1 Plant, S2 Referans + Hata Hesabi)
% Tum degiskenler base workspace'e yazilir. Testler bu betigi calistirip
% degiskenleri (base'te) ezdikten sonra sim('VehicleDiagram') cagirabilir.
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

%% --- Lastik modeli (S1b) ---
% tire_mode = 0: "notlar" (S1: F_x dogrudan dagitilir, lineer lastik, anlik
%                atan kayma, vx_floor); 1: "tam" (aktuator + tekerlek
%                dinamigi + gecici kayma + Dugoff + yuk transferi).
tire_mode = 1;
R_e      = 0.3;       % efektif yuvarlanma yaricapi [m] (Simscape Driveline Tire (Magic Formula) varsayilani r_e)
I_w      = 2*1.0;     % aks basina tekerlek ataleti [kg m^2] (teker basina 1, ayni bilesenin varsayilani)
C_kappa  = 19*m*g/2;  % aks basina boyuna kayma rijitligi [N] = B*C*D*F_z; B=10, C=1.9, D=1: bilesenin 'constant coefficients' parametrelemesi varsayilani (bkz. belge)
c_x      = 2*1e6;     % aks basina boyuna karkas rijitligi [N/m] (teker basina 1e6, ayni bilesenin compliance varsayilani)
sigma_x  = C_kappa/c_x;   % boyuna gevseme uzunlugu [m] (Pacejka: sigma_kappa = C_Fkappa / C_Fx)
sigma_y  = 0.3;       % yanal gevseme uzunlugu [m] (tekerlek yaricapi mertebesi, bkz. belge)
mu       = 0.75;      % yol tutus katsayisi (lonlat)
h        = 0.55;      % CG yuksekligi [m] (lonlat, yuk transferi)
tau_ax   = 0.1;       % a_x filtresi zaman sabiti [s] (govde pitch modu ~1.5 Hz, bkz. belge)
k_low0   = 13000;     % dusuk hiz sonum katsayisi [N s/m] (Pacejka k_Vlow formu, bkz. belge)
v_low    = 1;         % dusuk hiz bolgesi siniri [m/s] (sonum ve doyma cekisi bunun altinda)
tau_c    = 0.02;      % dusuk hiz doyma cekisi zaman sabiti [s] (bkz. belge)
v_eps    = 0.02;      % tam modda yuvarlanma direnci tanh(v_x/v_eps) gecis hizi [m/s]; Coulomb'a yakin:
                      % kilit sonrasi fren birakmada kayma 1.3 mm (0.1 ile 5.9 mm); viskoz egim C_rr*m*g/v_eps -> h*lambda ~ 0.007

%% --- Simulasyon ---
Ts_plant = 1e-3;      % sabit adim [s] (ode4)
Ts_ctrl  = 0.02;      % kontrol / referans ornekleme suresi [s] (Ts_plant'in tam kati)
T_end    = 20;        % bitis zamani [s]

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
                 'interp_method', ms_map__{2}, 'v_max', 8, 'ay_max', 0.85*mu*g, ...
                 'ax_max', 3, 'ax_min', -6, 'v_min', 2, 'kappa_win', 3);
clear ms_map__
ref = reference_path(waypoints, startPt, prm_ref);   % yol uzunlugu ref.L (L = dingil mesafesi)
win_fwd  = 120;       % en yakin nokta arama penceresi, ileri [dugum]
win_back = 20;        % geri [dugum]
fin = struct('n_laps', 3, 'v_stop', 0.05, 't_hold', 0.5);   % bitis olcutleri (Ts = Ts_ctrl ayri)
stop_enable = 1;      % 0: bitis mantigi kapali (plant testleri)

%% --- Baslangic kosullari ---
vx0_init = ref.v(ref.idx0);   % boyuna hiz [m/s] (hiz profilinden)
vy0_init = 0;                 % yanal hiz [m/s]
r0_init  = 0;                 % yaw hizi [rad/s]
X0       = ref.x(ref.idx0);   % global konum X [m] (yolun baslangic noktasi)
Y0       = ref.y(ref.idx0);   % global konum Y [m]
psi0     = ref.theta0;        % yaw acisi [rad] (yol yonu)

%% --- Test girisleri (From Workspace, [t u]; son degerde tutulur) ---
Fx_in    = [0 250; T_end 250];   % toplam boyuna kuvvet [N]
delta_in = [0 0;   T_end 0];     % on teker direksiyon acisi [rad]

%% --- Workspace ---
ms_names__ = {'m','Iz','L','lr','lf','Cf','Cr', ...
         'rho_air','CdA','C_rr','g','theta_road', ...
         'vx_floor','brake_split','Ts_plant','T_end', ...
         'tire_mode','R_e','I_w','C_kappa','c_x','sigma_x','sigma_y', ...
         'mu','h','tau_ax','k_low0','v_low','tau_c','v_eps', ...
         'vx0_init','vy0_init','r0_init','X0','Y0','psi0', ...
         'Fx_in','delta_in', ...
         'Ts_ctrl','map_name','waypoints','startPt','prm_ref','ref', ...
         'win_fwd','win_back','fin','stop_enable'};
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

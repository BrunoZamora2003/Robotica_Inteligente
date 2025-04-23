%% Limpieza y parámetros
clear; close all; clc;

ts        = 0.1;            % paso de simulación [s]
tf        = 800;            % tiempo total [s]
t         = 0:ts:tf;
maxSteps  = length(t);

%% Estado inicial
x1    = zeros(1,maxSteps+1);
y1    = zeros(1,maxSteps+1);
phi   = zeros(1,maxSteps+1);
x1(1) = 0.474198490222;   y1(1) = 1.4910681048178;   phi(1) = 0;

waypoints = [
    0.474198490222, 1.4910681048178;   % C
    0.8644612960156, 2.0904002708581;   % D
    1.3244138885582, 2.5364149060509;   % E
    1.6589248649528, 3.0939320000418;   % F
    2.0770635854460, 4.1392800012749;   % G
    3.5823629792216, 3.3587543896875;   % H
    5.1852280744457, 2.9406156691943;   % I
    5.8124361551855, 2.9127397544948;   % J
    5.5754908802393, 4.3341114041717;   % K
    6.0354434727819, 3.8744588116292;   % L
    6.4535821932751, 4.1532179586247;   % M
    6.6347756388222, 4.3622873188713;   % N
    6.6765895108715, 4.7943639967143;   % O
    7.0389764019656, 3.7768931101808;   % P
    7.3456114636607, 3.6514514940328;   % Q
    7.6522465253557, 4.1392800012749;   % R
    8.0000000000000, 5.0000000000000;   % S
    8.6279035398399, 6.5784225374854;   % T
    8.6418414971897, 7.3310722343732;   % U
    9.1017940897322, 7.2056306182252;   % V
    9.9241335733689, 7.2474444902746;   % W
    10.5095277820594, 7.2474444902746;  % Z
    11.3318672656961, 7.4425758931714;  % A1
    10.7046591849563, 6.2299736037410;  % B1
    10.5513416541088, 5.4215720774541;  % C1
    9.5338707675752, 4.7664880820147;   % D1
    9.3248014073286, 4.4459150629699;   % E1
    9.1296700044318, 3.4981339631853;   % F1
    9.1157320470820, 2.6061046927997;   % G1
    8.3073305207951, 2.1740280149567;   % H1
    7.6522465253557, 1.2680607872214;   % I1
    7.4989289945082, 0.4596592609345;   % J1
    7.0529143593154, 0.8638600240779;   % K1
    5.9936296007326, 1.2541228298716;   % L1
    4.8089032260018, 0.6966045358806;   % M1
    3.9726257850153, 1.4353162754187;   % N1
    3.5823629792216, 1.6861995077146;   % O1
    3.1642242587284, 1.4492542327685;   % P1
    2.3697606897913, 1.0450534696250;   % Q1
    1.5056073341053, 1.4492542327685;   % R1
    0.9202131254147, 1.6304476783155;   % S1
    0.5020744049215, 1.4910681048178;
    0.474198490222, 1.4910681048178;   % C
% T1
];

numWP = size(waypoints,1);

%% Parámetros de control “girar→avanzar”
K_rho    = 0.5;    % ganancia de avance
K_alpha  = 1.0;    % ganancia de giro
rho_th   = 0.1;    % umbral de llegada [m]
alpha_th = 0.05;   % umbral de alineación [rad]

u   = zeros(1,maxSteps);
w   = zeros(1,maxSteps);
err = zeros(1,maxSteps);

currentWP = 2;     % primer destino
step      = 1;

%% Bucle WHILE con dead‑reckoning
while currentWP<=numWP && step<=maxSteps

  % 1) Vector de error y distancia euclidiana
  dx  = waypoints(currentWP,1) - x1(step);
  dy  = waypoints(currentWP,2) - y1(step);
  rho = sqrt(dx^2 + dy^2);
  err(step) = rho;

  % 2) Si llegamos, avanzamos al siguiente waypoint
  if rho < rho_th
    currentWP = currentWP + 1;
    continue    % recalcula con el nuevo destino
  end

  % 3) Error angular
  theta_d = atan2(dy, dx);
  alpha   = atan2(sin(theta_d - phi(step)), cos(theta_d - phi(step)));

  % 4) Ley de control
  if abs(alpha)>alpha_th
    u(step) = 0;
    w(step) = K_alpha * alpha;
  else
    u(step) = K_rho * rho;
    w(step) = 0;
  end

  % 5) Dead‑reckoning (Odómetro)
  delta_d     = u(step)   * ts;    % Δd = v·Δt
  delta_theta = w(step)   * ts;    % Δθ = ω·Δt

  x1(step+1)  = x1(step)  + delta_d * cos(phi(step));
  y1(step+1)  = y1(step)  + delta_d * sin(phi(step));
  phi(step+1) = phi(step) + delta_theta;

  step = step + 1;
end

%% Recorta vectores al tamaño real
x1    = x1(1:step);
y1    = y1(1:step);
phi   = phi(1:step);
u     = u(1:step-1);
w     = w(1:step-1);
err   = err(1:step-1);
t_sim = (0:step-2)*ts;

%% Animación 3D
scene = figure;
set(scene,'Color','white','Position',get(0,'ScreenSize'));
camlight('headlight'); axis equal; grid on; box on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
view([15 15]); axis auto;

scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2 = plot3(x1(1),y1(1),0,'r-','LineWidth',2);

for k = 1:5:length(x1)
    delete(H1); delete(H2);
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2 = plot3(x1(1:k),y1(1:k),zeros(1,k),'r-','LineWidth',2);
    pause(ts);
end

%% Gráficas de u, w y ρ
fig = figure; set(fig,'Position',get(0,'ScreenSize'));
subplot(3,1,1)
plot(t_sim, u, 'b','LineWidth',1.5); grid on;
ylabel('v [m/s]'); title('Velocidad lineal');
subplot(3,1,2)
plot(t_sim, w, 'r','LineWidth',1.5); grid on;
ylabel('\omega [rad/s]'); title('Velocidad angular');
subplot(3,1,3)
plot(t_sim, err,'k','LineWidth',1.5); grid on;
xlabel('Tiempo [s]'); ylabel('\rho [m]');
title('Distancia euclidiana al waypoint');
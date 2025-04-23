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
x1(1) = 0;   y1(1) = 2.63;   phi(1) = 0;

%% Lista completa de waypoints (C → L2)
waypoints = [
    0.00, 2.63;   % C
    0.00, 4.37;   % D
    1.28, 5.65;   % E
    2.16, 5.69;   % F
    2.18, 5.25;   % G
    1.30, 4.39;   % H
    0.84, 4.35;   % I
    0.84, 3.05;   % J
    1.72, 3.93;   % K
    2.60, 3.93;   % L
    3.44, 3.07;   % M
    3.46, 4.75;   % N
    4.30, 3.93;   % O
    4.78, 3.49;   % P
    3.90, 3.47;   % Q
    3.90, 2.61;   % R
    4.74, 2.63;   % S
    4.74, 2.17;   % T
    5.62, 2.17;   % U
    5.20, 2.61;   % V
    5.60, 2.63;   % W
    6.48, 2.65;   % Z
    6.48, 3.47;   % A1
    5.62, 3.51;   % B1
    6.16, 3.89;   % C1
    4.32, 3.93;   % D1
    4.80, 3.51;   % E1
    4.74, 2.65;   % F1
    4.74, 2.17;   % G1
    5.62, 2.17;   % H1
    5.20, 2.63;   % I1
    5.60, 2.65;   % J1
    5.62, 3.51;   % K1
    6.16, 3.89;   % L1
    6.92, 4.79;   % M1
    6.90, 2.19;   % N1
    6.48, 1.75;   % O1
    5.60, 1.29;   % P1
    4.74, 1.29;   % Q1
    3.92, 1.77;   % R1
    3.46, 2.17;   % S1
    3.48, 0.89;   % T1
    3.88, 0.87;   % U1
    3.84, 0.05;   % V1
    2.58, 0.01;   % W1
    2.60, 1.75;   % Z1
    1.74, 1.77;   % A2
    1.28, 1.33;   % B2
    1.28, 0.89;   % C2
    2.16, 0.89;   % D2
    2.16, 0.03;   % E2
    0.40, 0.03;   % F2
    0.00, 2.59;
    0.00, 2.63% G2
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
%% Limpieza y parámetros
clear; close all; clc;

ts        = 0.1;            % paso de simulación [s]
tf        = 800;            % tiempo total [s]
t         = 0:ts:tf;
maxSteps  = length(t);

%% Lista completa de waypoints (extraídos de imágenes)
waypoints = [
    1.28, 0.22;   % A
    0.58, 0.88;   % B
    0.50, 1.58;   % C
    1.88, 2.24;   % D
    1.28, 2.96;   % E
    0.56, 4.26;   % F
    1.24, 5.66;   % G
    2.58, 5.60;   % Hhaz que la primera coordenada de waypoints sea el punto de partida del robot
    4.00, 5.00;   % I
    4.62, 5.60;   % J
    4.02, 6.18;   % K
    4.64, 5.62;   % L
    5.32, 6.32;   % M
    4.64, 5.60;   % N
    5.32, 4.92;   % O
    5.32, 1.54;   % P
    6.68, 0.82;   % Q
    7.26, 0.28;   % R
    8.66, 0.90;   % S
    8.66, 1.54;   % T
    7.38, 2.20;   % U
    8.00, 3.00;   % V
    8.64, 4.32;   % W
    8.06, 5.58;   % Z
    6.62, 5.70;   % A1
    5.32, 4.92;   % B1
    5.30, 1.52;   % C1
    4.64, 0.90;   % D1
    3.96, 1.54;   % E1
    4.00, 5.00;   % F1
    3.92, 1.52;   % G1
    2.60, 0.88;
    1.28, 0.22% H1
];
numWP = size(waypoints,1);

%% Estado inicial (desde primer waypoint)
x1    = zeros(1,maxSteps+1);
y1    = zeros(1,maxSteps+1);
phi   = zeros(1,maxSteps+1);
x1(1) = 1.28;   % x inicial
y1(1) = 0.22;   % y inicial
phi(1)= 0;                % orientación inicial (hacia el eje X)

K_rho    = 0.5;   % ganancia de avance (antes 0.5)
K_alpha  = 2.0;   % ganancia de giro (antes 1.0)
rho_th   = 0.05;  % umbral de llegada (antes 0.1)
alpha_th = 0.01;  % umbral de alineación (antes 0.05)


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
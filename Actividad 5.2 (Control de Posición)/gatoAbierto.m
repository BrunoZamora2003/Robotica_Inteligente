clear; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIGURACION INICIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts = 0.1;
v_const = 0.5;           % Velocidad lineal constante
w_const = pi/6;          % Velocidad angular constante

waypoints = [
    0.52, -0.06;
    0.06,  3.18;
    0.00,  5.52;
    1.64,  7.02;
    2.74,  6.92;
    2.72,  6.36;
    1.68,  5.36;
    1.12,  5.32;
    1.14,  3.72;
    2.16,  4.82;
    3.24,  4.78;
    4.32,  3.78;
    4.42,  5.96;
    5.40,  4.82;
    7.64,  4.82;
    7.10,  4.24;
    8.16,  4.30;
    8.12,  3.20;
    7.12,  3.22;
    7.10,  4.26;
    7.14,  3.20;
    6.54,  3.28;
    7.08,  2.68;
    5.98,  2.68;
    5.98,  3.20;
    4.92,  3.18;
    4.94,  4.22;
    5.96,  4.30;
    5.98,  3.24;
    5.98,  4.26;
    5.44,  4.82;
    7.64,  4.82;
    8.72,  5.96;
    8.70,  2.72;
    8.00,  2.00;
    7.12,  1.52;
    6.00,  1.64;
    5.00,  2.00;
    4.38,  2.62;
    4.32,  1.02;
    5.00,  1.00;
    5.00,  0.00;
    3.26, -0.04;
    3.22,  2.22;
    2.18,  2.08;
    1.66,  1.60;
    1.66,  1.00;
    2.70,  0.98;
    2.72, -0.06;
    0.3, -0.08
];

% Inicialización de estados
x = 0.52; y = -0.06; phi = 0;
x_traj = x; y_traj = y; phi_traj = phi;

%%%%%%%%%%%%%%%%%%%%%% GENERAR TRAYECTORIA EN LAZO ABIERTO %%%%%%%%%%%%%%%%%%%%

for i = 1:size(waypoints,1)-1
    dx = waypoints(i+1,1) - waypoints(i,1);
    dy = waypoints(i+1,2) - waypoints(i,2);
    
    ang_objetivo = atan2(dy, dx);
    dtheta = ang_objetivo - phi;
    
    t_giro = abs(dtheta) / w_const;
    pasos_giro = ceil(t_giro / ts);
    
    for k = 1:pasos_giro
        phi = phi + sign(dtheta) * w_const * ts;
        x_traj(end+1) = x;
        y_traj(end+1) = y;
        phi_traj(end+1) = phi;
    end
    
    dist = sqrt(dx^2 + dy^2);
    t_avance = dist / v_const;
    pasos_avance = ceil(t_avance / ts);
    
    for k = 1:pasos_avance
        x = x + v_const * cos(phi) * ts;
        y = y + v_const * sin(phi) * ts;
        x_traj(end+1) = x;
        y_traj(end+1) = y;
        phi_traj(end+1) = phi;
    end
end

N = length(x_traj);

%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuración de escena
scene = figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([15 15]);
axis([-11 11 -8 10 0 2]);

% b) Cargar y graficar robot
scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x_traj(1), y_traj(1), phi_traj(1), scale); hold on;

% c) Inicializar trayectoria
H2 = plot3(x_traj(1), y_traj(1), 0, 'r', 'LineWidth', 2);

% d) Animación del movimiento
for k = 1:N
    delete(H1); delete(H2);

xlim([-11.0 7.5])
ylim([-5.7 10.0])
zlim([0.00 2.00])
view([0.29 90.00])
    H1 = MobilePlot_4(x_traj(k), y_traj(k), phi_traj(k), scale);
    H2 = plot3(x_traj(1:k), y_traj(1:k), zeros(1,k), 'r', 'LineWidth', 2);
    pause(ts);
end
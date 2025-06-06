clear all
close all
clc

% Base del robot (con orientación inicial)
H0 = SE3(rotx(pi), [0 0 0]);

% Definición de transformaciones relativas
H{1}  = SE3(rotz(pi/2),     [0 0 0]);       
H{2}  = SE3(rotx(-pi/5),    [2 0 2]);        
H{3}  = SE3(rotx(-2*pi/6),  [0 0 0]);       
H{4}  = SE3(roty(pi/2),     [0 0 0]);          
H{5}  = SE3(rotx(pi/2),     [0 0 0]);        
H{6}  = SE3(roty(-6*pi/8),  [0 0 0]);
H{7}  = SE3(rotx(0),        [4 0 0]);
H{8}  = SE3(rotz(2*pi/3),   [4 0 0]);
H{9}  = SE3(roty(pi/2),       [0 0 0]);
H{10} = SE3(rotz(pi/2),     [0 0 1]);
% Acumulación de transformaciones
H_global{1} = H0 * H{1};
for i = 2:10
    H_global{i} = H_global{i-1} * H{i};
end

% Puntos de cada trama
p = zeros(3, 11); % p(:,1) es H0
p(:,1) = transl(H0);
for i = 1:10
    p(:,i+1) = transl(H_global{i});
end

% Coordenadas para plot3
X = p(1,:); Y = p(2,:); Z = p(3,:);

% Visualización de estructura del robot
plot3(X, Y, Z, '-o', 'LineWidth', 2, 'MarkerSize', 6);
axis([-8 5 -8 1 -8 1]); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Animacion del robot con 10 eslabones');
hold on;

% Trama base
trplot(H0, 'rgb', 'frame', 'S0', 'length', 0.5);

% Animaciones H0 -> H1 -> ... -> H10
pause;
tranimate(H0, H_global{1}, 'rgb', 'frame', 'S1', 'axis', [-8 5 -8 1 -8 1]);

for i = 2:10
    pause;
    frame_name = ['S', num2str(i)];
    tranimate(H_global{i-1}, H_global{i}, 'rgb', 'frame', frame_name, 'axis', [-8 5 -8 1 -8 1]);
end

% Mostrar transformación final
disp('Transformación final H10:');
disp(H_global{10});

% -----------------------------------------------------------------
% SIMULACIÓN DE UN ROBOT PÉNDULO DE 1 GDL
% -----------------------------------------------------------------
% Este script principal configura y ejecuta la simulación de la
% dinámica del robot definida en 'f.m' usando el solver ode45.

% Limpieza del entorno
clear; clc; close all;

% 1. Parámetros de Simulación
% -----------------------------------------------------------------
ti = 0;     % Tiempo inicial (s)
tf = 10;    % Tiempo final (s)
h = 0.0025; % Tamaño de paso (step size) para el muestreo
ts = ti:h:tf; % Vector de tiempo para la simulación

% 2. Condiciones Iniciales
% -----------------------------------------------------------------
% Definimos el estado inicial del sistema x0 = [q(0); qp(0)]
q0 = 0;   % Posición inicial (0 rad)
qp0 = 0;  % Velocidad inicial (0 rad/s)
x0 = [q0, qp0]; % Vector de estado inicial

% 3. Resolución de la Ecuación Diferencial
% -----------------------------------------------------------------
% Se utiliza ode45, un solver estándar de MATLAB para ecuaciones
% diferenciales ordinarias (EDO).
% Llama a la función @f (definida en f.m)
% en el intervalo de tiempo 'ts'
% comenzando desde las condiciones iniciales 'x0'
[t, x] = ode45(@f, ts, x0);

% 't' es el vector de tiempo resultante
% 'x' es una matriz donde cada fila es el vector de estado [q, qp]
% en el instante 't' correspondiente.
% x(:,1) -> Todas las posiciones (q)
% x(:,2) -> Todas lasvelocidades (qp)

% 4. Procesamiento y Gráficas de Resultados
% -----------------------------------------------------------------
% Convertimos radianes a grados para visualización
posicion_grados = (180/pi) * x(:,1);
velocidad_grados = (180/pi) * x(:,2);

% Gráfica de Posición
subplot(2, 1, 1) % Divide la ventana en 2 filas, 1 col, y usa la 1ra
plot(t, posicion_grados, 'b', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('\theta (°)');
title('Posición Articular del Péndulo');
grid on;

% Gráfica de Velocidad
subplot(2, 1, 2) % Usa la 2da parte de la ventana
plot(t, velocidad_grados, 'r', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('d\theta/dt (°/s)');
title('Velocidad Articular del Péndulo');
grid on;

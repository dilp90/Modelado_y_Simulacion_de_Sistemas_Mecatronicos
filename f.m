% --- Definición de la Ecuación Dinámica del Robot Péndulo ---
% Esta función es llamada por el solver ode45 para calcular la derivada
% del vector de estados en un tiempo 't' y un estado 'x' dados.

function dxdt = f(t,x)

% 1. Parámetros del modelo (del libro de F. Reyes Cortés)
% -----------------------------------------------------------------
m = 5;      % Masa de la barra (kg)
lc = 0.01;  % Distancia al centro de masa de la barra (m)
g = 9.81;   % Aceleración de la gravedad (m/s^2)
b = 0.14;   % Coeficiente de fricción viscosa (Nms/rad)
fc = 0.45;  % Coeficiente de fricción de Coulomb (Nm)
Ir = 0.16;  % Inercia rotacional del péndulo (mlc^2 + I)

% 2. Vector de Estados (Entrada)
% -----------------------------------------------------------------
% 'x' es un vector 2x1 que contiene la posición y velocidad
% q = x(1); % Posición articular (rad)
% qp= x(2); % Velocidad articular (rad/s)

% 3. Ecuación Dinámica (Modelo Físico)
% -----------------------------------------------------------------
% Basado en la ecuación de movimiento:
% Suma de Torques = I*qpp + b*qp + mglc*sin(q) + Fricción_Coulomb
%
% tau = Ir*qpp + b*qp + mglc*sin(q) + fc*sign(qp)
%
% Despejamos la aceleración (qpp) para la simulación:
% qpp = (1/Ir) * [tau - mglc*sin(q) - b*qp - fc*sign(qp)]

% Torque de entrada (motor)
% Llama a la función local 'tau' definida al final de este archivo
tau_motor = tau(t);

% Modelo de Fricción de Coulomb (Stribeck)
% Usamos tanh(c*qp) como una aproximación suave de la función sign(qp)
% para evitar discontinuidades que afecten al solver ode45.
% El signo negativo indica que la fricción se opone al movimiento.
friccion_coulomb = fc * tanh(100000 * x(2)); 

% Cálculo de la aceleración articular (qpp)
qpp = (1/Ir) * (tau_motor - m*g*lc*sin(x(1)) - b*x(2) - friccion_coulomb);

% 4. Vector de Salida (Derivada del Estado)
% -----------------------------------------------------------------
% El solver ode45 necesita la derivada del vector de estados:
% d/dt [q]  = [ qp ]
% d/dt [qp] = [ qpp]
dxdt = [ x(2);   % dx(1)/dt = qp
         qpp     % dx(2)/dt = qpp
       ];
end

% --- Función Local para el Torque de Entrada ---
% Esta función solo es visible dentro del archivo f.m
function z = tau(t)
    % Se aplica un torque sinusoidal con amplitud 1.5 Nm
    z = 1.5*sin(t);
end
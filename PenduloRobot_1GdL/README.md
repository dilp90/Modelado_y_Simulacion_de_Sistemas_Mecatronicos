# **Simulación de Péndulo Robótico de 1 GDL en MATLAB**

Este repositorio contiene el código de MATLAB para el **Ejemplo 5.1** (página 271\) del libro *"Robótica, control de robots manipuladores"* (2011) de Fernando Reyes Cortés. El proyecto simula la dinámica de un robot péndulo simple de un grado de libertad (GDL).

Este proyecto fue desarrollado para la clase de **Modelado y Simulación de Sistemas Mecatrónicos** de la carrera de Ingeniería en Mecatrónica.

## **Descripción del Sistema Físico**

El sistema consiste en un péndulo simple (una barra o eslabón) de 1 GDL que puede rotar alrededor de un eje fijo. El péndulo tiene una masa $m$, una inercia rotacional $I\_r$ y un centro de masa a una distancia $l\_c$ del eje de rotación.

El sistema está sujeto a:

* Un **torque de entrada** $\\tau(t)$ (el motor).  
* Un **torque gravitacional** ($mgl\_c \\sin(q)$).  
* **Fricción viscosa** ($b\\dot{q}$), que se opone a la velocidad.  
* **Fricción de Coulomb** ($f\_c \\text{sign}(\\dot{q})$), que es una fuerza constante que se opone al movimiento.

## **El Modelo Dinámico (La Ecuación de Movimiento)**

La dinámica del sistema se deriva de la Segunda Ley de Newton para la rotación ($\\sum \\text{Torques} \= I\\alpha$), lo que nos da:

$\\tau(t) \= I\_r \\ddot{q} \+ b \\dot{q} \+ mgl\_c \\sin(q) \+ f\_c \\text{sign}(\\dot{q})$

Para simular esto en una computadora, necesitamos despejar la aceleración ($\\ddot{q}$), que es la segunda derivada de la posición:

$\\ddot{q} \= \\frac{1}{I\_r} \\left( \\tau(t) \- b \\dot{q} \- mgl\_c \\sin(q) \- f\_c \\text{sign}(\\dot{q}) \\right)$

## **Implementación Computacional**

El simulador ode45 de MATLAB no puede resolver ecuaciones de segundo orden ($\\ddot{q}$) directamente. Requiere un sistema de ecuaciones de **primer orden**. Para lograr esto, usamos el **Modelo en Variables de Estado**.

1. **Definimos el vector de estado x:**  
   * $x\_1 \= q$ (Posición articular)  
   * $x\_2 \= \\dot{q}$ (Velocidad articular)  
2. **Definimos la derivada del vector de estado dxdt:**  
   * $\\dot{x}\_1 \= \\frac{d}{dt}(q) \= \\dot{q} \= x\_2$  
   * $\\dot{x}\_2 \= \\frac{d}{dt}(\\dot{q}) \= \\ddot{q} \= \\frac{1}{I\_r} \\left( \\tau(t) \- b x\_2 \- mgl\_c \\sin(x\_1) \- f\_c \\text{sign}(x\_2) \\right)$

El archivo f.m implementa esta función dxdt \= f(t, x).

### **Archivos del Repositorio**

1. **f.m**:  
   * Es la función que define el modelo dinámico (las ecuaciones de estado).  
   * Toma el tiempo t y el vector de estado x como entradas.  
   * Calcula la aceleración qpp ($\\ddot{q}$) basándose en los parámetros físicos.  
   * Devuelve dxdt, la derivada del vector de estado, que es \[x(2); qpp\].  
   * Incluye una función local tau(t) que define el torque de entrada como una onda sinusoidal: 1.5\*sin(t).  
2. **ResPS.m**:  
   * Este es el **script principal** que se debe ejecutar.  
   * Define los parámetros de la simulación (tiempo inicial, final y paso de muestreo).  
   * Establece las **condiciones iniciales** del sistema: x0 \= \[0, 0\], lo que significa que el péndulo comienza en la posición 0 (vertical hacia abajo, o el origen) y en reposo (velocidad 0).  
   * Llama al solver \[t, x\] \= ode45(@f, ts, x0); para resolver las ecuaciones diferenciales.  
   * Finalmente, procesa los resultados (convirtiendo de radianes a grados) y genera dos gráficas:  
     1. Posición articular vs. Tiempo.  
     2. Velocidad articular vs. Tiempo.

## **Cómo Ejecutar la Simulación**

1. Asegúrate de que los archivos f.m y ResPS.m estén en la misma carpeta.  
2. Abre MATLAB y navega a esa carpeta.  
3. Abre el archivo ResPS.m.  
4. Presiona el botón "Run" (Ejecutar).  
5. Se generará una ventana con las gráficas de posición y velocidad del péndulo.

## **Nota sobre el Modelo de Fricción**

En el código f.m, la fricción de Coulomb (que es una función sign(qp) discontinua) se modela usando una función tanh(c \* qp):

friccion\_coulomb \= fc \* tanh(1000 \* x(2))

Esta es una práctica estándar en simulación. La función tanh (tangente hiperbólica) es una aproximación continua y "suave" de la función sign, lo que previene problemas de convergencia en el solver ode45 causados por la discontinuidad abrupta en $\\dot{q}=0$.
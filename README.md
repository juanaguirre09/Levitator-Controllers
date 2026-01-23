# Levitator Controllers

This repository presents the implementation of several control strategies for a **pneumatic levitation system**, developed as part of the course **Control Systems IV**.

The project focuses on the design, implementation, and comparison of classical and advanced controllers using both **Arduino** and **LabVIEW**, combining embedded programming and graphical control design.

The implemented control approaches include:
- PID controllers (Type 1 and Type 2)
- Generalized polynomial controllers
- State-space controllers
- Fuzzy logic controllers
- Neural network–based controllers

---

## Project Overview

The main objective of this project is to control the vertical position of a pneumatic levitator and evaluate the system behavior under different control techniques. Each controller was tested to analyze stability, response time, and overall performance.

Two development environments were used:
- **Arduino IDE** for embedded control implementation
- **LabVIEW** for graphical control design and analysis

---

## Project Objectives

- Design controllers for a pneumatic levitation system  
- Implement classical and advanced control techniques  
- Compare controller performance in real-time experiments  
- Integrate sensors and actuators using Arduino and LabVIEW  

---

## Technologies Used

- **Arduino UNO / Arduino Mega**
- **C++ (Arduino IDE)**
- **LabVIEW**
- **Sensor:** Proximity sensor
- **Actuator:** 12 V DC motor
- **Control techniques:** PID, polynomial control, state-space, fuzzy logic, neural networks

---

## Repository Structure

## PID Type 1 (Arduino) — Design & Implementation

This section summarizes the development of the **Type 1 PID controller** implemented in `Algoritmo1.ino`.  
The design follows a **discrete-time pole placement approach** using an **online estimated plant model**.

### 1) Estimated plant model (1st order)

The levitator is approximated by a first-order discrete transfer function:

\[
H(z)=\frac{b_0}{z+a_1}
\]

The parameters \(a_1\) and \(b_0\) are updated online using a hybrid estimator (Orthogonal Projection / Recursive Least Squares), depending on the estimation error.  
(See "Construcción Estimadores" in the report.)  

### 2) Controller structure (Type 1 PID)

The controller is defined as:

\[
C(z)=\frac{C_0 z^2 + C_1 z + C_2}{z(z-1)}
\]

### 3) Pole placement

The closed-loop characteristic polynomial is:

\[
P_o = 1 + H(z)\,C(z)
\]

We choose a desired polynomial:

\[
P_d = (z+\alpha)^3 = z^3 + 3\alpha z^2 + 3\alpha^2 z + \alpha^3
\]

By matching coefficients, the controller parameters are computed using a linear system:

\[
\begin{bmatrix}C_0\\C_1\\C_2\end{bmatrix}
=
\begin{bmatrix}
b_0&0&0\\
0&b_0&0\\
0&0&b_0
\end{bmatrix}^{-1}
\begin{bmatrix}
3\alpha + 1 - a_1\\
3\alpha^2 + a_1\\
\alpha^3
\end{bmatrix}
\]

Where \(\alpha\in(0,1)\) is chosen experimentally to balance speed and stability.

### 4) Discrete control law (difference equation)

Finally, the control signal is implemented as:

\[
u[k] = -u[k-1] + C_0 e[k] + C_1 e[k-1] + C_2 e[k-2]
\]

with \(e[k]=r[k]-y[k]\).

### 5) Practical implementation notes

- **Sampling time** used in experiments: typically **100 ms** for the PID controllers.
- The project uses `BasicLinearAlgebra.h` to compute matrix operations and solve the controller equations in real time.
- The Arduino Serial interface is used to adjust **reference (setpoint)** and controller mode during tests.

---

### Suggested figures to add

- **Block diagram** of the closed-loop system (plant + controller)
- **Serial Plotter response** (reference vs output vs error)
- **Photo of the levitator setup**
- **Transfer function / pole placement equations** (optional screenshot from the report)


### `controladores-arduino/`
Arduino-based controller implementations:

- `Algoritmo1.ino` – Type 1 PID controller  
- `Algoritmo2.ino` – Type 2 PID controller  
- `Algoritmo4.ino` – Generalized polynomial controller without integral action  
- `Algoritmo5.ino` – Generalized polynomial controller with feedforward gain  
- `1Entrada_9Reglas.ino` – Fuzzy logic controller (1 input, 9 rules)  
- `2Entradas_9Reglas.ino` – Fuzzy logic controller (2 inputs, 9 rules)  
- `2Entradas_25Reglas.ino` – Fuzzy logic controller (2 inputs, 25 rules)  

---

### `controladores-Labview/`
LabVIEW-based controller implementations:

- `Algoritmo_3.vi` – Generalized polynomial controller with integral action  
- `Algoritmo_6.vi` – Reduced state-space controller with feedforward gain  
- `Algoritmo_7.vi` – Reduced state-space controller with integral action  
- `Algoritmo_8.vi` – Extended state-space controller with feedforward gain  
- `Algoritmo_9.vi` – Extended state-space controller with integral action  
- `README.md` – Description of the controllers designed for the pneumatic levitator  

---

## Demonstration Video

Watch the system in operation:  
[Click here to watch the video](https://youtu.be/dqrvql4zS70?si=Z20IAbmv8brBYKhx)

---

## Authors

- **Juan Carlos Aguirre Arenas**  
- **Matthew Ocampo Herrera**

*Mechatronics Engineering*  
*Universidad Tecnológica de Pereira*


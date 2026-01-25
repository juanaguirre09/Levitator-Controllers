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
---

## PID Type 1 Controller — Design and Implementation (Arduino)

This section describes the development of the **Type 1 PID controller**
implemented in `Algoritmo1.ino`, based on the methodology presented in
the project report.

### Estimated plant model

The pneumatic levitation system is approximated by a first-order
discrete-time transfer function:

![Plant model](docs/images/formulas/pid_type1_plant_model.png)

The parameters of the model are estimated online, allowing the controller
to adapt to changes in system dynamics.

### PID Type 1 controller structure

The controller includes integral action and is defined by the following
discrete transfer function:

![PID structure](docs/images/formulas/pid_type1_controller_structure.png)

### Pole placement approach

The controller gains are obtained using a pole placement method.
A desired closed-loop polynomial of the form (z + α)³ is selected,
where the parameter α is tuned experimentally to balance stability and
response speed.

### Discrete control law

The final control signal applied to the actuator is computed using the
following difference equation:

![Difference equation](docs/images/formulas/pid_type1_difference_equation.png)

### Experimental response (Arduino Serial Plotter)

The following figure shows the real-time response of the pneumatic
levitator under the Type 1 PID controller. The plot includes the
reference signal (setpoint), the measured position, and the estimated
output.

![PID Type 1 response](docs/images/pid_type1_serial_plotter_response.png)

## PID Type 2 Controller — Design and Implementation (Arduino)

This section presents the implementation of the **Type 2 PID controller**
developed in `Algoritmo2.ino`.

Compared to the Type 1 controller, the Type 2 PID includes an additional
integral action, which improves steady-state accuracy and disturbance
rejection.

### PID Type 2 controller structure

The controller is defined by the following discrete transfer function:

![PID Type 2 structure](docs/images/formulas/pid_type2_controller_structure.png)

### Discrete control law

The control signal is computed using the following difference equation,
which is executed in real time on the Arduino:

![PID Type 2 difference equation](docs/images/formulas/pid_type2_difference_equation.png)

---

## Neural Network Controller — Design and Implementation (LabVIEW)

This section presents the implementation of an **inverse neural network
controller** for the pneumatic levitation system, developed using
**LabVIEW** and described in the project report.

This approach explores the use of data-driven techniques to control a
nonlinear system without relying on an explicit analytical model.

### Inverse neural network concept

The neural network is designed to act as an **inverse model of the
plant**. Instead of computing the system output from a control input,
the network estimates the **control action required** to achieve a
desired position.

The controller generates the actuator command based on the reference
signal and the measured system response.

### Neural network structure

A **feedforward neural network** is used, composed of:
- Input variables related to the reference and system behavior
- One hidden layer with nonlinear activation functions
- An output layer providing the control signal

The network is trained offline using experimental input–output data
obtained from the pneumatic levitator.

### LabVIEW implementation

The trained neural network is integrated into LabVIEW and executed in
real time within a closed-loop control structure.

**Front panel view:**

![Inverse NN front panel](docs/images/red_neuronal_inversa_control.png)

**Block diagram implementation:**

![Inverse NN block diagram](docs/images/red_neuronal_inversa.png)

During operation, the measured position is acquired by the sensor, the
neural network computes the required control action, and the result is
applied to the actuator.

### Experimental observations

The inverse neural network controller is capable of stabilizing the
levitation system and tracking the reference signal. The results show
that neural networks can be an effective alternative for controlling
nonlinear systems, although their performance strongly depends on the
quality of the training data.

### Pneumatic levitator setup

The following image shows the real pneumatic levitation system used for
experimental validation of the control strategies.

![Pneumatic levitator setup](docs/images/planta_levitador.png)


## Repository Structure

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


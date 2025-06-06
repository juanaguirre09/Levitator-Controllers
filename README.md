# Levitator-Controllers

Controllers of the following types: Type 1, Type 2 generalized polynomial, state-space for a pneumatic levitator, fuzzy logic and neuronal networks.

# Controladores en Arduino y Labview

Este repositorio contiene el desarrollo de un proyecto realizado como parte del curso de SISTEMAS DE CONTROL IV, el cual consistió en la implementación de diferentes controladores utilizando Arduino, programando el microcontrolador en Arduino IDE y en el entorno gráfico de Labview. Se incluyen los códigos fuente.

## 📌 Objetivo del proyecto

Desarrolar controladores para un sistema de levitación neumática usando Arduino y Labview. 

## ⚙️ Tecnologías utilizadas

- Arduino UNO / Mega
- Lenguaje C++ (Arduino IDE)
- Sensores: Sensor de proximidad
- Actuadores: Motor 12 VDC
- Labview

## 📁 Estructura del repositorio
controladores-arduino/
- Algoritmo1.ino # Controlador PID tipo 1
- Algoritmo2.ino # Controlador PID tipo 2
- Algoritmo4.ino # Polinomial Generalizado sin acción integral
- Algoritmo5.ino # Polinomial Generalizado con ganancia en lazo directo
- 1Entrada_9Reglas.ino #Lógica difusa con una entrada y 9 reglas
- 2Entradas_9Reglas.ino #Lógica difusa con dos entradas y 9 reglas
- 2Entradas_25Reglas.ino #Lógica difusa con dos entradas y 25 reglas
  
controladores-Labview/
- Algoritmo_3.vi # Polinomial generalizado con acción integral
- Algoritmo_6.vi # Espacio de estados reducido con ganancia en lazo directo
- Algoritmo_7.vi # Espacio de estados reducido con acción integral
- Algoritmo_8.vi # Espacio de estados extendido con ganancia en lazo directo
- Algoritmo_9.vi # Espacio de estados extendido con acción integral
- README.md # Controladores diseñados para un levitador neumático

## Video demostrativo
[Haz clic aquí para ver el video](https://youtu.be/dqrvql4zS70?si=Z20IAbmv8brBYKhx)

## 🔗 Autores

- Juan Carlos Aguirre y Matthew Ocampo Herrera
- Ingeniería Mecatrónica   
- Universidad Tecnológica de Pereira

---



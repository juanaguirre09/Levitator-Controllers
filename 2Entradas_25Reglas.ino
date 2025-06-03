#include <BasicLinearAlgebra.h>
using namespace BLA;

/* ------------- Pines ------------- */
const int trig = 8;
const int echo = 9;
const int pwm  = 10;

/* ------------- Planta ------------- */
const float Max = 41.0;
const float Min = 5.0;
long  sensor = 0;
int   salida = 0;

/* --------- Referencia y estados ----*/
float ref = 23;                 // se cambia por Serial
float yk  = 0;                  // variable de proceso

float e   = 0, e1 = 0, de = 0;  // error y derivada

/* -- Centros de los conjuntos difusos (5 niveles) --  */
float p5 = -41, p4 = -20, p3 = 0, p2 = 20, p1 = 41;   // error (posición)
float v5 = -82, v4 = -41,  v3 = 0, v2 = 41,  v1 = 82;      // derivada del error


float m11 = -26, m12 = -41, m13 = -35, m14 = -41, m15 = -41;
float m21 = -20, m22 = -24, m23 = -32, m24 = -35, m25 =  -41;
float m31 = 17, m32 = 0, m33 =  -6, m34 = -22, m35 =   -28;
float m41 =  23, m42 =   15, m43 =   16, m44 =  2, m45 =  -22;
float m51 =  28, m52 =  23, m53 =  17, m54 =  -22, m55 =  -15;

/* ---------- Desviaciones estándar ------------ */
float desv_p = 8.706, desv_v = 17.41;

/* ---------- Tiempo de muestreo 100 ms ---------- */
unsigned long actual, previo = 0;

/* ========= Única función de membresía ========= */
float calcularE(float ee, float p, float desv1)
{
  float exponente = -0.5 * pow((ee - p) / desv1, 2);
  return exp(exponente);
}

/* ================== SETUP ====================== */
void setup()
{
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(pwm,  OUTPUT);
  digitalWrite(trig, LOW);

  Serial.println(F("Controlador difuso 5×5 listo. Envíe nueva referencia (cm)."));
}

/* =================== LOOP ====================== */
void loop()
{
  /* --- Leer nueva referencia ------------------ */
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    ref = input.toFloat();
  }

  /* ----------- Ejecutar cada 100 ms ----------- */
  actual = millis();
  if (actual - previo >= 100) {
    previo = actual;

    /* ---------- Lectura ultrasónica ------------ */
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);

    sensor = pulseIn(echo, HIGH);
    yk     = sensor * 0.034f / 2.0f;      // distancia en cm

    /* ------------- Cálculo del error ----------- */
    e1 = e;
    e  = ref - yk;
    de = (e - e1) / 0.1f;                 // Δt = 0,1 s

    /* ---------- Centros y matriz en arrays ----- */
    float p_arr[5] = {p5, p4, p3, p2, p1};
    float v_arr[5] = {v5, v4, v3, v2, v1};
    float m[5][5] = {
      {m11, m12, m13, m14, m15},
      {m21, m22, m23, m24, m25},
      {m31, m32, m33, m34, m35},
      {m41, m42, m43, m44, m45},
      {m51, m52, m53, m54, m55}
    };

    /* --------- Membresías ep[i], ev[j] --------- */
    float ep[5], ev[5];
    for (uint8_t i = 0; i < 5; i++) {
      ep[i] = calcularE(e,  p_arr[i], desv_p);
      ev[i] = calcularE(de, v_arr[i], desv_v);
    }

    /* --------------- Inferencia ---------------- */
    float num = 0.0f, den = 0.0f;
    for (uint8_t i = 0; i < 5; i++) {
      for (uint8_t j = 0; j < 5; j++) {
        float mu = ep[i] * ev[j];
        den += mu;
        num += mu * m[i][j];
      }
    }
    float u = num / den;          // promedio ponderado

    /* --------- PWM:  86 % ↔ 219,  97 % ↔ 247 ---- */
    salida  = map(u, 41, -41, 219, 244);   // inverso: mayor u ⇒ menor PWM
    salida  = constrain(salida, 219, 244);
    analogWrite(pwm, salida);

    /* --------------- Telemetría ---------------- */
    Serial.print(yk);   Serial.print(",");
    Serial.print(ref);  Serial.print(",");
    Serial.print(50);   Serial.print(",");
    Serial.print(0);    Serial.print(",");
    Serial.print(u);    Serial.print(",");
    Serial.print(salida); Serial.print(",");
    Serial.println(e);
  }
}

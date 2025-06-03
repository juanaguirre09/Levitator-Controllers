#include <BasicLinearAlgebra.h>
using namespace BLA;

// pines
const int trig = 8;
const int echo = 9;
const int pwm = 10;

// Variables Planta
const float Max = 41.0;
const float Min = 5.0;
long sensor=0;
int salida=0;

// Variables Y y U
float ref=23;
float u=0, u1=0, u2=0, u3=0, u4=0, u5=0, u6=0, u7=0, u_8=0, u9=0,uden=0;
float yk=0;
float e=0, e1=0, e2=0, e3=0, e4=0, e5=0, e6=0, e7=0, e8=0, e9=0;
float p9=-41, p8=-30, p7=-20, p6=-10, p5=0, p4=10, p3=20, p2=30, p1=41;
float m9=-41, m8=-35, m7=-27, m6=-19, m5=-9, m4=2, m3=10, m2=20, m1=30;

// desviacion estandar
float desv=4.353;

//Tiempo
int actual,previo=0;

///////////////////////////////////////////////////////
float calcularE(float ee, float p, float desv1) 
{
  float exponente = -0.5 * pow((ee - p) / desv1, 2);
  float e = exp(exponente);
  return e;
}
///////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(9600);

  pinMode(trig,OUTPUT);// Definir como salida Ultrasonico 
  pinMode(echo,INPUT);//Definir como entrada ultrasonico 
  pinMode(pwm,OUTPUT);// Salida potencia ventilador 
  digitalWrite(trig,LOW);// Inicio apagado de trig
}

void loop() 
{
  //////////////////////////////////
  if(Serial.available()>0)
  {
    String input = Serial.readStringUntil('\n');
    ref = input.toInt();
    // if (uk<40 || uk>5) uk=uk1;
  }
  /////////////////////////////////
  actual=millis();
  if(actual-previo>=100)
  {
    previo=actual;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    sensor=pulseIn(echo,HIGH);
    yk=sensor*0.034 / 2;

    ////////////////////////////////////////////////////////////
    e=ref-yk;

    e9 = calcularE(e, p9, desv);
    e8 = calcularE(e, p8, desv);
    e7 = calcularE(e, p7, desv);
    e6 = calcularE(e, p6, desv);
    e5 = calcularE(e, p5, desv);
    e4 = calcularE(e, p4, desv);
    e3 = calcularE(e, p3, desv);
    e2 = calcularE(e, p2, desv);
    e1 = calcularE(e, p1, desv);

    u9=e9*m9;
    u_8=e8*m8;
    u7=e7*m7;
    u6=e6*m6;
    u5=e5*m5;
    u4=e4*m4;
    u3=e3*m3;
    u2=e2*m2;
    u1=e1*m1;
    uden=e9+e8+e7+e6+e5+e4+e3+e2+e1;
    u=(u9+u_8+u7+u6+u5+u4+u3+u2+u1)/uden;

    salida=map(u, 41, -41, 219, 245);
    salida=constrain(salida, 219,245);
    
    analogWrite(pwm, salida);


    Serial.print(yk);
    Serial.print(",");
    Serial.print(ref);
    Serial.print(",");
    Serial.print(50);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.print(u); 
    Serial.print(","); 
    Serial.print(salida); 
    Serial.print(","); 
    Serial.println(e);  
  }

}









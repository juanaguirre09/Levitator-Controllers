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
float e=0, e1=0, de=0, ep3=0, ep2=0, ep1=0, ev3=0, ev2=0, ev1=0;
float p3=-41, p2=0, p1=41, v3=-82, v2=0, v1=82;
// float m11=-32, m12=-34, m13=-41;
// float m21=18,  m22=-5,  m23=-19;
// float m31=34,  m32=30,  m33=20;

float m11=-30, m12=-33, m13=-41;
float m21=17,  m22=-5,  m23=-21;
float m31=30,  m32=27,  m33=23;

// desviacion estandar
float desv_v=34.8, desv_p=17.4;

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
    e1=e;
    e=ref-yk;
    de=(e-e1)/0.1f;

    ep1 = calcularE(e, p3, desv_p);
    ep2 = calcularE(e, p2, desv_p);
    ep3 = calcularE(e, p1, desv_p);

    ev1 = calcularE(de, v3, desv_v);
    ev2 = calcularE(de, v2, desv_v);
    ev3 = calcularE(de, v1, desv_v);

    u9=ep1*ev1*m11;
    u_8=ep1*ev2*m12;
    u7=ep1*ev3*m13;
    u6=ep2*ev1*m21;
    u5=ep2*ev2*m22;
    u4=ep2*ev3*m23;
    u3=ep3*ev1*m31;
    u2=ep3*ev2*m32;
    u1=ep3*ev3*m33;
    uden=ep1*ev1+ep1*ev2+ep1*ev3+ep2*ev1+ep2*ev2+ep2*ev3+ep3*ev1+ep3*ev2+ep3*ev3;
    u=(u9+u_8+u7+u6+u5+u4+u3+u2+u1)/uden;

    salida=map(u, 41, -41, 218, 248);//ajustar
    salida=constrain(salida, 218,248);//ajustar
    
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









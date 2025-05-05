#include <BasicLinearAlgebra.h>
using namespace BLA;

// MAtricez
Matrix<4,1> TETA;
Matrix<4,1> PHI, X3;
Matrix<1,1> X1;
Matrix<4,4> P, X2;

// Control
Matrix<4,4> A, Ainv;
Matrix<4,1> B, X;

// Variablees de control
float e=0, e1=0, e2=0, e3=0;
float pol=0.9, pol1=0, pol2=0, pol3=0, pol4=0;//0.3
float c0=1, c1=1, c2=1, c3=0;

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
float ref=20;
float u=0, uk=20, uk1=0, uk2=0;
float yk=0, yk1=0, yk2=0, ye=0;

// Variables formulas
float x1=0,  x2=0, den=0, ee=0;

// Teta
float a1=-0.883495, a2=-0.0861467;
float b1= -0.0308776, b2=0.0378642;

//Tiempo
int actual,previo=0;


char mode="m";
int value=23;
float value1=0.9;
String numberStr;



void setup() 
{
  Serial.begin(9600);
  TETA={a1,
        a2,
        b1,
        b2};
  P={1000000,0,0,0,
    0,1000000,0,0,
    0,0,1000000,0,
    0,0,0,1000000};

  pinMode(trig,OUTPUT);// Definir como salida Ultrasonico 
  pinMode(echo,INPUT);//Definir como entrada ultrasonico 
  pinMode(pwm,OUTPUT);// Salida potencia ventilador 
  digitalWrite(trig,LOW);// Inicio apagado de trig

  pol1=4*pol;
  pol2=6*pow(pol, 2);
  pol3=4*pow(pol, 3);
  pol4=pow(pol, 4);

  A={b1,0,0,1,
    b2,b1,0,(a1-1),
    0,b2,b1,(-a1+a2),
    0,0,b2,-a2};

  Ainv = A;
  Invert(Ainv);

  B={pol1-a1+1,
    pol2-a2+a1,
    pol3+a2,
    pol4};
  X=Ainv*B;

  c0 = X(0,0);
  c1 = X(1,0);
  c2 = X(2,0);
  c3 = X(3,0);
}

void loop() 
{
  //////////////////////////////////
    if(Serial.available()>0)
  {
    String input = Serial.readStringUntil('\n');

    input.trim();

    if (input.length() >=2) 
    {
      numberStr = input.substring(0, input.length() - 1);
      mode = input.charAt(input.length() - 1);
      value1 = numberStr.toFloat();
      if (mode=='p')
      {
        Serial.print("SI");  
        pol=value1;
      }
    }
    if (input.length() == 3) 
    {
      numberStr = input.substring(0, 2);
      mode = input.charAt(2);
      value = numberStr.toInt();
      if (mode=='m')
      {
        Serial.print("SI");  
        u=map(value, 41, 5, 218, 243);
        salida=constrain(u, 218,243);
        analogWrite(pwm, salida);
      }
    }
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
    PHI={-yk1,-yk2,uk1,uk2};
    ye=(~PHI*TETA)(0,0);
    ye=constrain(ye, 0,50);
    ee=yk-ye;
    
    if(false)
    {
      X3=(P*PHI);
      den=1+(~PHI*P*PHI)(0,0);
      TETA=TETA+(X3*(ee/den));
      X2=P*PHI*~PHI*P;
      P=P-(X2*(1/den));
    }
    else
    {
      X1=~PHI*PHI;
      if(X1(0,0)==0)
      {
        den=0.001;
      }
      else
      {
        den=ee/X1(0,0);
      }
      TETA=TETA+PHI*den;

    }

    a1=TETA(0,0);
    a2=TETA(1,0);
    b1=TETA(0,0);
    b2=TETA(1,0);

    pol1=4*pol;
    pol2=6*pow(pol, 2);
    pol3=4*pow(pol, 3);
    pol4=pow(pol, 4);

    A={b1,0,0,1,
      b2,b1,0,(a1-1),
      0,b2,b1,(-a1+a2),
      0,0,b2,-a2};

    Ainv = A;
    Invert(Ainv);

    pol1=4*pol;
    pol2=6*pow(pol, 2);
    pol3=4*pow(pol, 3);
    pol4=pow(pol, 4);

    B={pol1-a1+1,
      pol2-a2+a1,
      pol3+a2,
      pol4};
    X=Ainv*B;

    c0 = X(0,0);
    c1 = X(1,0);
    c2 = X(2,0);
    c3 = X(3,0);

    u=uk2*c3+uk1*(1-c3)+c0*e+c1*e1+c2*e2;
    if(mode=='a')
    {
      salida=map(u, -41, 41, 218, 243);
      salida=constrain(salida, 218,243);
    }
    analogWrite(pwm, salida);

    yk2=yk1;
    yk1=yk;
    uk2=uk1;
    uk1=value;

    e3=e2;
    e2=e1;
    e1=e;
    e=value-yk;


    Serial.print(yk);
    Serial.print(",");
    Serial.print(ye);
    Serial.print(",");
    Serial.print(value); 
    Serial.print(",");
    Serial.print(50);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.print(u); 
    Serial.print(","); 
    Serial.print(salida); 
    Serial.print(","); 
    Serial.print(pol); 
    Serial.print(","); 
    Serial.println(e);  
  }

}









#include <BasicLinearAlgebra.h>
using namespace BLA;

// Matrices
Matrix<6,1> TETA;
Matrix<6,1> PHI, X3;
Matrix<1,1> X1;
Matrix<6,6> P, X2;

// Control
Matrix<6,6> A, Ainv;
Matrix<6,1> B, X;

// Variablees de control
float e=0, e1=0, e2=0, e3=0;
float pol=0.9, pol1=0, pol2=0, pol3=0, pol4=0, pol5=0, pol6=0;
float p1=0, p2=0, p3=0, l0=0, l1=0, l2=0;

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
float u=0, uk=25, uk1=0, uk2=0, uk3=0;
float yk=0, yk1=0, yk2=0, yk3=0, ye=0;

// Variables formulas
float x1=0,  x2=0, den=0, ee=0;

// Teta
float a1=-0.84819, a2=-0.164262, a3=0.0408;
float b1=0.00527998, b2=0.00223577, b3=0.105075;

//Tiempo
int actual,previo=0;


void setup() 
{
  Serial.begin(9600);
  TETA={a1,
        a2,
        a3,
        b1,
        b2,
        b3};
  P={1000000,0,0,0,0,0,
    0,1000000,0,0,0,0,
    0,0,1000000,0,0,0,
    0,0,0,10000000,0,0,
    0,0,0,0,1000000,0,
    0,0,0,0,0,1000000};

  pinMode(trig,OUTPUT);// Definir como salida Ultrasonico 
  pinMode(echo,INPUT);//Definir como entrada ultrasonico 
  pinMode(pwm,OUTPUT);// Salida potencia ventilador 
  digitalWrite(trig,LOW);// Inicio apagado de trig

  pol1=6*pol;
  pol2=15*pow(pol, 2);
  pol3=20*pow(pol, 3);
  pol4=15*pow(pol, 4);
  pol5=6*pow(pol, 5);
  pol6=pow(pol,6);


  A = {
    1,    0,   0,  0,      0,   0,
    a1-1,   1,   0, b1,      0,   0,
    a2-a1,  a1-1,   1, b2,     b1,   0,
    -a3,  a2-a1,  a1-1, b3,     b2,  b1,
    0,  -a3,  a2-a1,  0,     b3,  b2,
    0,    0, -a3,  0,      0,  b3,
  };
  // A = {
  //   1,    0,   0,  0,      0,   0,
  //   a1,   1,   0, b1,      0,   0,
  //   a2,  a1,   1, b1,     b1,   0,
  //   a3,  a2,  a1, b3,     b2,  b1,
  //   0,  -a3,  a2,  0,     b3,  b2,
  //   0,    0, -a3,  0,      0,  b3,
  // };

  Ainv = A;
  Invert(Ainv);

  B={pol1-a1,
    pol2-a2,
    pol3-a3,
    pol4,
    pol5,
    pol6};

  X=Ainv*B;

  p1 = X(0,0);
  p2 = X(1,0);
  p3 = X(2,0);
  l0 = X(3,0);
  l1 = X(4,0);
  l2 = X(5,0);

}

void loop() 
{
  //////////////////////////////////
  if(Serial.available()>0)
  {
    String input = Serial.readStringUntil('\n');
    uk = constrain(input.toInt(), Min, Max);
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

    sensor=pulseIn(echo, HIGH, 30000);
    yk=sensor*0.034 / 2;

    //////////////////////////////////////////////////////////////////////////
    
    PHI={-yk1,-yk2,-yk3,uk1,uk2,uk3};
    ye=(~PHI*TETA)(0,0);
    ye=constrain(ye, 0,50);
    ee=yk-ye;

    if(ee>=100)
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
    a3=TETA(2,0);
    b1=TETA(3,0);
    b2=TETA(4,0);
    b3=TETA(5,0);

    A = {
      1,    0,   0,  0,      0,   0,
      a1-1,   1,   0, b1,      0,   0,
      a2-a1,  a1-1,   1, b2,     b1,   0,
      -a2,  a2-a1,  a1-1, b3,     b2,  b1,
      0,  -a2,  a2-a1,  0,     b3,  b2,
      0,    0, -a2,  0,      0,  b3,
    };
    // A = {
    //   1,    0,   0,  0,      0,   0,
    //   a1,   1,   0, b1,      0,   0,
    //   a2,  a1,   1, b1,     b1,   0,
    //   a3,  a2,  a1, b3,     b2,  b1,
    //   0,   a3,  a2,  0,     b3,  b2,
    //   0,    0,  a3,  0,      0,  b3,
    // };

    Ainv = A;
    Invert(Ainv);

    B={pol1-a1,
      pol2-a2,
      pol3-a3,
      pol4,
      pol5,
      pol6};

    X=Ainv*B;

    p1 = X(0,0);
    p2 = X(1,0);
    p3 = X(2,0);
    l0 = X(3,0);
    l1 = X(4,0);
    l2 = X(5,0);

    ////////////////////////////////////////////////////////////////////////////
    
    u = -(p1)*uk1 - (p2)*uk2 - (p3)*uk3 + l0*e1 + l1*e2 + l2*e3;
    salida=map(u, 41, -41, 220, 240);
    salida=constrain(salida, 220,240);
    analogWrite(pwm, salida);

    yk3=yk2;
    yk2=yk1;
    yk1=yk;


    uk3=uk2;
    uk2=uk1;
    uk1=uk;

    e3=e2;
    e2=e1;
    e1=e;
    e=uk-yk;

    Serial.print(yk);
    Serial.print(",");
    Serial.print(ye);
    Serial.print(",");
    Serial.print(uk); 
    Serial.print(","); 
    Serial.print(u);
    Serial.print(","); 
    Serial.print(salida);
    Serial.print(","); 
    Serial.print(0); 
    Serial.print(","); 
    Serial.print(50);  
    Serial.print(","); 
    Serial.println(e); 

     
  }

}









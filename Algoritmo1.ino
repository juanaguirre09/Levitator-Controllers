#include <BasicLinearAlgebra.h>
using namespace BLA;
//Si oscila mucho bajar el polo por pasos de 0.01
// MAtricez de Estimacion
Matrix<2,1> TETA;
Matrix<2,1> PHI, X3;
Matrix<1,1> X1;
Matrix<2,2> P, X2;
// MAtricez control
Matrix<3,3> A, Ainv;
Matrix<3,1> B, X;

// Variablees de control
float e=0, e1=0, e2=0;
float pol=0.9, pol1=0, pol2=0, pol3=0;//0.2
float c0=1, c1=1, c2=1;

// pines
const int trig = 8;
const int echo = 9;
const int pwm = 10;

// Funcionales
const float Max = 41.0;
const float Min = 5.0;
int salida=0;

// Variables
float ref=23;
float u=0, uk=23 , uk1=0;
float yk=0, yk1=0, ye=0;
float x1=0,  x2=0, den=0, ee=0;

// Teta
float a1=-0.280447;
float b1= 0.779912;

//Tiempo
int actual,previo=0;



char mode="m";
int value=23;
float value1=0.9;
String numberStr;



/////////////////////////////////////////////Set UP
void setup() 
{
  Serial.begin(9600);
  TETA={a1,
        b1};
  P={1000000, 0,
    0, 1000000};

  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  pinMode(pwm,OUTPUT);
  digitalWrite(trig,LOW);

  pol1=3*pol;
  pol2=3*pow(pol, 2);
  pol3=pow(pol, 3);
  ////////////Matricez
  A={b1,0,0,
    0,b1,0,
    0,0,b1};

  Ainv = A;
  Invert(Ainv);

  B={pol1+1-a1,pol2+a1,pol3};

  X=Ainv*B;

  c0 = X(0,0);
  c1 = X(1,0);
  c2 = X(2,0);

}

void loop() 
{
  //////////////////////////////////Referencia puerto serial
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
        u=map(value, 41, 5, 222, 255);
        salida=constrain(u, 222,255);
        analogWrite(pwm, salida);
      }
    }
  }
  /////////////////////////////////Loop por bandera, leer ultrasonico, yk
  actual=millis();
  if(actual-previo>=100)////////////////////////////////////////////////////////////////////////////////////269/0.82
  {
    previo=actual;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    yk=pulseIn(echo,HIGH);
    yk=yk*0.034 / 2;

    ////////////////////////////////////////////////////

    PHI={-yk1,uk1};
    ye=(~PHI*TETA)(0,0);
    ye=constrain(ye, 0,50);
    ee=yk-ye;
    
    ////////////////////////////////////////////////Estimador Hibrido, Minimos cuadrados recursivos
    if(false)
    {
      X3=(P*PHI);
      den=1+(~PHI*P*PHI)(0,0);
      TETA=TETA+(X3*(ee/den));
      X2=P*PHI*~PHI*P;
      P=P-(X2*(1/den));
    }
    //////////////////////////////////////////////////// Estimador Hibirdo, Proyeccion Ortogonal
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
    b1=TETA(1,0);

    ////////////Matricez
    A={b1,0,0,
      0,b1,0,
      0,0,b1};
    Ainv = A;
    Invert(Ainv);
    pol1=3*pol;
    pol2=3*pow(pol, 2);
    pol3=pow(pol, 3);
    B={pol1+1-a1,pol2+a1,pol3};
    X=Ainv*B;

    c0 = X(0,0);
    c1 = X(1,0);
    c2 = X(2,0);

    ////////////////////////////////////////////////Variables de estimacion



    u=uk1+c0*e+c1*e1+c2*e2;
    if(mode=='a')
    {
      // u=constrain(salida, -41,41);
      u=map(u, 41, -41, 228, 255);
      salida=constrain(u, 228,255);
      
    }
    analogWrite(pwm, salida);

    yk1=yk;
    uk1=uk;
    e2=e1;
    e1=e;
    e=value-yk;

    Serial.print(yk);
    Serial.print(",");
    Serial.print(ye);
    Serial.print(",");
    Serial.print(50); 
    Serial.print(","); 
    Serial.print(0); 
    Serial.print(","); 
    Serial.print(e); 
    Serial.print(","); 
    Serial.print(salida); 
    Serial.print(","); 
    Serial.print(pol); 
    Serial.print(","); 
    Serial.println(value);  
  }

}









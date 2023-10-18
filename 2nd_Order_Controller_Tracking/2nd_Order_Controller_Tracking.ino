// Tracking senoidal

// ******************************************************** //
//                                                          //
// Control Tracking por asignacion de polos                 //
//                                                          //
// Noviembre 8 del 2022, MAPG, EAG                          //
//                                                          //
// Instrucciones                                            //
//                                                          //
// Modificar solamente                                      //
//    Polos de control                                      //
//    Polos de observador                                   //
//    Valores de R1, C1, R2, C2 en planta                   // 
//                                                          //
//                                                          //
// ******************************************************** //

// ********************************************************//
// ------- Frecuencia de la referenca------------
// *********************************************************//

// ******************************************************** //
//---------- Polos cont --------//                          //
// ******************************************************** //
  #define a1r -1                                            //
  #define a1i -1.7320508                                    //
  #define a2r -1                                          //
  #define a2i  1.7320508                                    //
// ******************************************************** //

  #define omega .2
  
// ******************************************************** //
//---------- Polos obs --------//                           //
// ******************************************************** //
  #define b1r -10                                           //
  #define b1i   0                                           //
  #define b2r -10                                           //
  #define b2i   0                                           //
// ******************************************************** //


// ******************************************************** //
//----------  Planta  --------//                            //
// ******************************************************** //
  #define R1 1000000                                        //
  #define R2 1000000                                        //
  #define C1 0.000001                                       //
  #define C2 0.000001                                       //
// ******************************************************** //

// ******************************************************** //
//----------  Muestreo  --------//                          //
// ******************************************************** //
  unsigned long TS = 50;  // Muestreo 50 miliseg
  float Tseg = 0.05;      // Muestreo en segundos
// ******************************************************** //

                             // Fin de parámetros modificables

// ******************************************************** //
//----------  Definiciones  --------//                      //
// ******************************************************** //

//Elementos de PI
float Pi11 = 0; 
float Pi12 = 0;
float Pi21 = 0;
float Pi22 = 0;

//Elementos de GAMA
float Ga11 = 0;
float Ga12 = 0;

// Elementos de Cr

float Cr1 = 0;
float Cr2 = 0;

// ******************************************************** //
//----------  Constantes  --------//                        //
// ******************************************************** //

//---------- Escalamientos --------//
  #define mX 0.004882813
  #define bX 0
  #define mU 51
  #define bU 0

//---------- Definición Pines IO discretos--------//                  
  #define pLED8 8             // LED 8 en tarjeta
  #define pLED9 9             // LED 9 en tarjeta   
  #define pSW2 2              // SW 2 en tarjeta                                    
  #define pSW3 3              // SW 3 en tarjeta   

//---------- Definicion Pines --------//
  #define pR 0                  
  #define pX1 1 
  #define pX2 2                                    
  #define pU 10      


// ******************************************************** //
//----------  Variables globales  --------//                //
// ******************************************************** //


//---------- Ganancias de Controlador --------//

  float Kp = 0;
  float K1 = 0;
  float K2 = 0;
  float G1 = 0;
  float G2 = 0;

//---------- Ganancias de Observador --------//

  float L1 = 0;
  float L2 = 0;

//---------- Definiciones --------//

  unsigned long TIC = 0;
  unsigned long TS_code = 0; 
  unsigned long TC = 0;
  
   
//----------  Señales --------//
  float R = 0;
  float Rw = 0;
  float Y = 0;
  float X1 = 0;
  float X2 = 0;
  float U = 0;
  int Ui = 0;
  int Cmin = 0;
  int Cmax = 5;


//----------  Observador --------//

//-- Estados obs --//

  float W1 = 0; // Estado exosistema 1 en k, W1[k]
  float W2 = 1; // Estado exosistema 2 en k, W2[k]
  float WN1 = 0; // Estado exosistema 1 en k+1, W1[k+1]
  float WN2 = 0; // Estado exosistema 2 en k+1, W2[k+1]
  float XeR1 = 0; // Estado estimado 1 en k, Xe1[k]
  float XeR2 = 0; // Estado estimado 2 en k, Xe2[k]
  float XeN1 = 0; // Estado estimado 1 en k+1, Xe1[k+1]
  float XeN2 = 0; // Estado estimado 2 en k+1, Xe2[k+1]
  float f1 = 0; // Dinamica de observador
  float f2 = 0; // Dinamica de observador

//-- Modelo obs --//
  float Am11 = 0;
  float Am12 = 0;
  float Am21 = 0;
  float Am22 = 0;
  // Matriz B
  float Bm1 = 0;
  float Bm2 = 0;  
  // Matriz C
  float Cm1 = 0;
  float Cm2 = 0;   
                
//---------- Otros --------//
  bool Habilitado = 0;        // Señal {0,1} para entradas escalón




// ******************************************************** //
//----------  Librerias  --------//                      //
// ******************************************************** //

  #include <SoftwareSerial.h>
  

// ******************************************************** //
//---------- Rutinia de inicio --------//                   //
// ******************************************************** //
void setup() {
  //--Inicia serial--//
  Serial.begin(9600);

  //--Configura pines digitales--//  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);


  //-- Determina ganancias --//
  // Factores comunes
  float sum_a = -(a1r  + a2r);
  float prod_a = a1r*a2r - a1i*a2i;
  float sum_b = -(b1r  + b2r);
  float prod_b = b1r*b2r - b1i*b2i;

  // Planta
  // Elementos Matriz A
  Am11 = -1/(R1*C1) - 1/(R2*C1);  //el error fue que no se puso el producto de resistencia capacitor entre parentesis, error de jerarquía de operaciones)
  Am12 = 1/(R2*C1);
  Am21 = 1/(R2*C2);
  Am22 = -1/(R2*C2);
  // Elementos Matriz B
  Bm1 = 1/(R1*C1);
  Bm2 = 0;
  // Elementos Matriz C
  Cm1 = 0;
  Cm2 = 1;
  //Elementos Cr
  Cr1 = 1;
  Cr2 = 0;
  
  //Elementos de Gama
  Ga12 = ((Bm2*Cm2 + Bm1*Cm1)*Cr1*pow(omega,3) - ((Am22*Bm2 + Am21*Bm1)*Cm2 + (Am12*Bm2 + Am11*Bm1)*Cm1)*Cr2*pow(omega,2) + (((Am12*Am21 + Am11*Am11)*Bm2 - (Am21*Am22 + Am11*Am21)*Bm1)*Cm2 + ((Am22*Am22 + Am12*Am21)*Bm1 - (Am12*Am22 + Am11*Am12)*Bm2)*Cm1)*Cr1*omega + (((Am11*Am12*Am21 - Am11*Am11*Am22)*Bm2 + (Am11*Am21*Am22 - Am12*Am21*Am21)*Bm1)*Cm2 + ((Am11*Am12*Am22 - Am12*Am12*Am21)*Bm2 + (Am12*Am21*Am22 - Am11*Am22*Am22)*Bm1)*Cm1)*Cr2) / ((Bm2*Bm2 + Cm2*Cm2 + 2*Bm1*Bm2*Cm1*Cm2 + Bm1*Bm1*Cm1*Cm1)*pow(omega,2) + (Am11*Am11*Bm2*Bm2 - 2*Am11*Am21*Bm1*Bm2 + Am21*Am21*Bm1*Bm1)*Cm2*Cm2 + ((2*Am11*Am22 + 2*Am12*Am21)*Bm1*Bm2 - 2*Am11*Am12*Bm2*Bm2 - 2*Am21*Am22*Bm1*Bm1)*Cm1*Cm2 + (Am12*Am12*Bm2*Bm2 - 2*Am12*Am22*Bm1*Bm2 + Am22*Am22*Bm1*Bm1)*Cm1*Cm1);
  Ga11 = ((-Bm2*Cm2 - Bm1*Cm1)*Cr2*pow(omega,3) - ((Am22*Bm2 + Am21*Bm1)*Cm2 + (Am12*Bm2 + Am11*Bm1)*Cm1)*Cr1*pow(omega,2) + (((Am21*Am22 + Am11*Am21)*Bm1 - (Am12*Am21 + Am11*Am11)*Bm2)*Cm2 + ((Am12*Am22 + Am11*Am12)*Bm2 - (Am22*Am22 + Am12*Am21)*Bm1)*Cm1)*Cr2*omega + (((Am11*Am12*Am21 - Am11*Am11*Am22)*Bm2 + (Am11*Am21*Am22 - Am12*Am21*Am21)*Bm1)*Cm2 + ((Am11*Am12*Am22 - Am12*Am12*Am21)*Bm2 + (Am12*Am21*Am22 - Am11*Am22*Am22)*Bm1)*Cm1)*Cr1) / ((Bm2*Bm2*Cm2*Cm2 + 2*Bm1*Bm2*Cm1*Cm2 + Bm1*Bm1*Cm1*Cm1)*pow(omega,2) + (Am11*Am11*Bm2*Bm2 - 2*Am11*Am21*Bm1*Bm2 + Am21*Am21*Bm1*Bm1)*Cm2*Cm2 + ((2*Am11*Am22 + 2*Am12*Am21)*Bm1*Bm2 - 2*Am11*Am12*Bm2*Bm2 - 2*Am21*Am22*Bm1*Bm1)*Cm1*Cm2 + (Am12*Am12*Bm2*Bm2 - 2*Am12*Am22*Bm1*Bm2 + Am22*Am22*Bm1*Bm1)*Cm1*Cm1);
  //Ga12 = 1.8;
  //Ga11 = 0.6;

  // Elementos de Pi
  Pi11 = (Bm1*Ga12*pow(omega,3) - (Am12*Bm2 + Am11*Bm1)*Ga11*pow(omega,2) + ((Am22*Am22 + Am12*Am21)*Bm1 - (Am12*Am22 + Am11*Am12)*Bm2)*Ga12*omega + ((Am11*Am12*Am22 - Am12*Am12*Am21)*Bm2 + (Am12*Am21*Am22 - Am11*Am22*Am22)*Bm1)*Ga11) / (pow(omega,4) +(Am22*Am22 + 2*Am12*Am21 + Am11*Am11)*pow(omega,2) + Am11*Am11*Am22*Am22 - 2*Am11*Am12*Am21*Am22 + Am12*Am12*Am21*Am21);
  Pi12 = (-Bm1*Ga11*pow(omega,3) - (Am12*Bm2 + Am11*Bm1)*Ga12*pow(omega,2) + ((Am12*Am22 + Am11*Am12)*Bm2 - (Am22*Am22 + Am12*Am21)*Bm1)*Ga11*omega + ((Am11*Am12*Am22 - Am12*Am12*Am21)*Bm2 + (Am12*Am21*Am22 - Am11*Am22*Am22)*Bm1)*Ga12) / (pow(omega,4) +(Am22*Am22 + 2*Am12*Am21 + Am11*Am11)*pow(omega,2) + Am11*Am11*Am22*Am22 - 2*Am11*Am12*Am21*Am22 + Am12*Am12*Am21*Am21);
  Pi21 = (Bm2*Ga12*pow(omega,3) - (Am22*Bm2 + Am21*Bm1)*Ga11*pow(omega,2) + ((Am12*Am21 + Am11*Am11)*Bm2 - (Am21*Am22 + Am11*Am21)*Bm1)*Ga12*omega + ((Am11*Am12*Am21 - Am11*Am11*Am21)*Bm2 + (Am11*Am21*Am22 - Am12*Am21*Am21)*Bm1)*Ga11) / (pow(omega,4) +(Am22*Am22 + 2*Am12*Am21 + Am11*Am11)*pow(omega,2) + Am11*Am11*Am22*Am22 - 2*Am11*Am12*Am21*Am22 + Am12*Am12*Am21*Am21);
  Pi22 = (-Bm2*Ga11*pow(omega,3) - (Am22*Bm2 + Am21*Bm1)*Ga12*pow(omega,2) + ((Am21*Am22 + Am11*Am21)*Bm1 - (Am12*Am21 + Am11*Am11)*Bm2)*Ga11*omega + ((Am11*Am12*Am21 - Am11*Am11*Am22)*Bm2 + (Am11*Am21*Am22 - Am12*Am21*Am21)*Bm1)*Ga12) / (pow(omega,4) +(Am22*Am22 + 2*Am12*Am21 + Am11*Am11)*pow(omega,2) + Am11*Am11*Am22*Am22 - 2*Am11*Am12*Am21*Am22 + Am12*Am12*Am21*Am21);
  //Pi11 = 1;
  //Pi12 = 0.2;
  //Pi21 = 1;
  //Pi22 = 0;

  // Ganancias de Controlador
  // u = Kp*(Vref) - K1*(Vcap1) - K2*(Vcap2) + G1*W1 + G2*W2
  K1 = ((Am21*Bm1 - Am11*Bm2)*(sum_a + Am11 + Am22) - (prod_a + Am12*Am21 - Am11*Am22)*Bm2)/((Am21*Bm1 - Am11*Bm2)*Bm1 - (Am12*Bm2 - Am22*Bm1)*Bm2);
  K2 = ((Am22*Bm1 - Am12*Bm2)*(sum_a + Am11 + Am22) + (prod_a + Am12*Am21 - Am11*Am22)*Bm1)/((Am21*Bm1 - Am11*Bm2)*Bm1 - (Am12*Bm2 - Am22*Bm1)*Bm2);
  Kp = ((Am12*Bm2 - Am22*Bm1)*K1 + (Am21*Bm1 - Am11*Bm2)*K2 + (Am11*Am22 - Am12*Am21))/(Am21*Cm2*Bm1 + Am12*Cm1*Bm2 - Am22*Cm1*Bm1 - Am11*Cm2*Bm2);
  
  G1 = K2*Pi21 +K1*Pi11 + Ga11;
  G2 = K2*Pi22 + K1*Pi12 + Ga12;
  

  // Ganancias de Observador
  // Xe[k+1] = A*Xe[k] + B*U[k] + H*(Y[k]-Xe2[k])

  L1 = ((Am11*Cm2 - Am12*Cm1)*sum_b + Cm2*prod_b + (Am12*Am21 + Am11*Am11)*Cm2 - (Am12*Am22 + Am11*Am12)*Cm1)/(Am21*Cm2*Cm2 + (Am11 - Am22)*Cm1*Cm2 - Am12*Cm1*Cm1);
  L2 = ((Am21*Cm2 - Am22*Cm1)*sum_b + (Am21*Am22 + Am11*Am21)*Cm2 - (Am22*Am22 + Am12*Am21)*Cm1 - Cm1*prod_b)/(Am21*Cm2*Cm2 + (Am11 - Am22)*Cm1*Cm2 - Am12*Cm1*Cm1);  

  //Serial.print("Am11: ");
  //Serial.print(Am11);
  //Serial.print("Am12: ");
  //Serial.print(Am12);
  //Serial.print("Am21: ");
  //Serial.print(Am21);
  //Serial.print("Am22: ");
  //Serial.print(Am22);
  //Serial.print("Bm1: ");
  //Serial.print(Bm1);
  //Serial.print("Bm2: ");
  //Serial.println(Bm2);

  //Serial.print("K1: ");
  //Serial.print(K1);
  //Serial.print("K2: ");
  //Serial.print(K2);
  //Serial.print("Kp: ");
  //Serial.print(Kp);
  //Serial.print("G1: ");
  //Serial.print(G1);
  //Serial.print("G2: ");
  //Serial.print(G2);
  //Serial.print("L1: ");
  //Serial.print(L1);
  //Serial.print("L2: ");
  //Serial.print(L2);
  
  //Serial.println("");
  //Serial.println("");
  //Serial.print("Pi11: ");
  //Serial.print(Pi11);
  //Serial.print(" Pi12: ");
  //Serial.print(Pi12);
  //Serial.print(" Pi21: ");
  //Serial.print(Pi21);
  //Serial.print(" Pi22: ");
  //Serial.print(Pi22);
  //Serial.print(" Ga11: ");
  //Serial.print(Ga11);
  //Serial.print(" Ga12: ");
  //Serial.print(Ga12);

}



// ******************************************************** //
//---------- Rutinia principal  --------//                  //
// ******************************************************** //
void loop() {                     
  proc_entradas();                    // Procesamiento de Entradas
  observador();                       // Observador
  control();                          // Control
  proc_salidas();                     // Procesado de Salidas
  coms_arduino_ide();               // Comunicaciones
  //coms_python(&Rw,&Y,&U);
  espera();
}




// ******************************************************** //
//---------- Rutinias --------//                            //
// ******************************************************** //

// ******************************************************** //
//---------- Rutinias de control y observador--------       //                          
// ******************************************************** //

//-- Observador e integradores --//
void observador(){
  f1 = Am11*XeR1 + Am12*XeR2 + Bm1*U + L1*(Y-XeR2);     // Dinamica estado 1 observador
  f2 = Am21*XeR1 + Am22*XeR2 + Bm2*U + L2*(Y-XeR2);     // Dinamica estado 2 observador

  XeN1 = XeR1 + Tseg*f1;              // Integrador estado 1 mediante Euler
  XeN2 = XeR2 + Tseg*f2;              // Integrador estado 2 mediante Euler
  WN1 = W1 + Tseg*omega*W2;
  WN2 = W2 - Tseg*omega*W1;
  XeR1 = XeN1;
  XeR2 = XeN2;
  W1 = WN1;
  W2 = WN2;
}

//-- Control --//
void control(){
  // Ley de control
  U = Habilitado*(Kp*R + G1*W1 + G2*W2 - K1*XeR1 - K2*XeR2);       // Ley de control retro estado estimado

  // Saturacion
  if(U >= 5.0) U = 5.0;               // Saturacion de control en rango 0 a 5V                      
  else if(U < 0) U = 0;
}

// ******************************************************** //
//---------- Rutinias de IO y control de tiempo     --------//                          
// ******************************************************** //


//-- Procesado de entradas --//
void proc_entradas(){
  // No se ocupa leer X1 (Vc1) porque se estima con observador
  X1=analogRead(pX1)*mX+bX;           // Lectura de salida de planta en pin pX3
  X2=analogRead(pX2)*mX+bX;           // Lectura de salida de planta en pin pX2
  R=analogRead(pR)*mX+bX;             // Lectura de referencia en pin pR
  Y = X2;
  Rw = Habilitado*(R + W1);
}


//-- Procesado de salidas --//
void proc_salidas(){
  Ui = int(U*mU+bU);                    // Escalamiento
  analogWrite(pU, Ui);                  // Salida PWM en pin pU
  botonesyleds();                       // Manejo de IO discretas
}


//-- Memoria {0,1} para entrada escalón --//
void botonesyleds(){

  static int n = 0; 
  if(n >= 1000/TS) n=0;                                // Señal cuadrada para led blink
  else n = n+1;

  if(digitalRead(pSW2) == 1) Habilitado = 1;      // Memoria on/off en Habilitado
  else if(digitalRead(pSW3) == 1) Habilitado = 0; // Set con SW2. Reset con SW3

  if(n >= 500/TS && Habilitado == 1) digitalWrite(pLED8,HIGH); // Led blink en LED8
  else digitalWrite(pLED8, LOW);                           // Cuando Habilitado = 1

  if(Habilitado == 0) digitalWrite(pLED9,HIGH);            // LED9 = 1
  else digitalWrite(pLED9, LOW);                           // Cuando Habilitado = 0
}


//-- Para muestreo uniforme --//
void espera(){   
  TS_code = millis()- TIC;                 // Tiempo de ciclo
  TC = TS - TS_code;                       // Calcula altante para TS
  if (TS_code < TS) delay(TC);             // Espera para completar ciclo de TS   
  TIC = millis();
}


//-- Comunicación con monitor --//
void coms_arduino_ide(){
  //Serial.print("Min:");            // Referencia
  //Serial.print(Cmin);
  //Serial.print(",");  
  //Serial.print("y_d(t):");            // Referencia
  //Serial.print(Rw);                    // Referencia
  //Serial.print(",");                  // Separador     
  //Serial.print("y(t):");              // Salida
  //Serial.print(Y);                  // Salida (terminar con "serial.println")
  //Serial.print(",");
  //Serial.print("Max:");              // Salida
  //Serial.println(Cmax);

  Serial.print("Min:");            // Referencia
  Serial.print(Cmin);
  Serial.print(",");  
  Serial.print("y_d(t):");            // Referencia
  Serial.print(Rw);                    // Referencia
  Serial.print(",");                  // Separador     
  Serial.print("y(t):");              // Salida
  Serial.print(Y);                  // Salida (terminar con "serial.println")
  Serial.print(",");
  Serial.print("Max:");              // Salida
  Serial.println(Cmax);


}


void coms_python(float* Rp, float* Yp, float* Up)
{
  byte* byteData1 = (byte*)(Rp);
  byte* byteData2 = (byte*)(Yp);
  byte* byteData3 = (byte*)(Up);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}

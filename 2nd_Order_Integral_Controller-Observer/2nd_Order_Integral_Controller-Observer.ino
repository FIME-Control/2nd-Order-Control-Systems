// Control-Observador-RC-RC
// Codificar arduino desde VS Code: https://learn.sparkfun.com/tutorials/efficient-arduino-programming-with-arduino-cli-and-visual-studio-code/all#:~:text=Open%20VS%20Code%2C%20then%20open,%22examples%22%20directories%20are%20contained.
// ******************************************************** //
//                                                          //
// Control por asignación de polos para circuito RC-RC      //
//                                                          //
// Julio 26 del 2023, MAPG, EAG                         //
//                                                          //
// Instrucciones                                            //
//                                                          //
// Modificar solamente:                                     //
//    Polos de control                                      //
//    Polos de observador                                   //
//    Modelo Valores de R1, C1, R2, C2 en planta            // 
//    Muestreo                                              //
//                                                          //
// ******************************************************** //


// ******************************************************** //
//---------- Polos de control --------//                    //
// ******************************************************** //
  #define a1r -1.25                                         //
  #define a1i -1.7320508                                   //
  #define a2r -1.25                                         //
  #define a2i 1.7320508                                    //
  #define a3r -1                                            //
  #define a3i 0                                            //
// ******************************************************** //

  
// ******************************************************** //
//---------- Polos de observador   --------//               //
// ******************************************************** //
  #define b1r -10                        // Real polo 1     //
  #define b1i   0                        // Imag polo 1     //
  #define b2r -10                        // Real polo 2     //
  #define b2i   0                        // Imag polo 2     //
// ******************************************************** //


// ******************************************************** //
//----------  Valores de R1, C1, R2, C2 en planta  -------- //
// ******************************************************** //
  #define R1 1000000                                        //
  #define R2 1000000                                        //
  #define C1 0.000001                                       //
  #define C2 0.000001                                       //
// ******************************************************** //


// ******************************************************** //
//----------  Muestreo  --------//                          //
// ******************************************************** //
  unsigned long TS = 50;      // Muestreo TS miliseg        //
  float Tseg = 0.05;          // Muestreo en Tseg segundos  //
// ******************************************************** //

                             // Fin de parámetros modificables

// ******************************************************** //
//----------  Definiciones  --------//                      //
// ******************************************************** //

// ******************************************************** //
//----------  Constantes  --------//                        //
// ******************************************************** //

//---------- Definición Pines IO analogicos--------//                  
  #define pR 0                // pin de referencia             
  #define pX1 1               // pin de estado 1   
  #define pX2 2               // pin de estado 2                     
  #define pU 10               // pin de salida de control (entrada a planta)      

//---------- Definición Pines IO discretos--------//                  
  #define pLED8 8             // LED 8 en tarjeta
  #define pLED9 9             // LED 9 en tarjeta   
  #define pSW2 2              // SW 2 en tarjeta                                    
  #define pSW3 3              // SW 3 en tarjeta      

//---------- Escalamientos para analogicas de 0 a 5 V --------//
  #define mX 0.004882813      // Pendiente 0-1023 -> 0 - 5
  #define bX 0                // Ajuste cero para 0-1023 -> 0 - 5
  #define mU 51               // Pendiente 0 - 5 -> 0 - 1023
  #define bU 0                // Ajuste cero 0 - 5 -> 0 - 1023

// ******************************************************** //
//----------  Variables globales  --------//                //
// ******************************************************** //
  
//---------- Ganancias de Controlador -------//
  float K0 = 0;
  float K1 = 0;
  float K2 = 0;

//---------- Ganancias de Observador -------.//
  float L1 = 0;
  float L2 = 0;

//---------- Tiempo --------//
  unsigned long TS_code = 0;  // Tiempo que tarda programa
  unsigned long TIC = 0;      // Estampa de tiempo inicio ciclos
  unsigned long TC = 0;       // Faltante para TS
   
//----------  Señales --------//
  float R = 0;                // Referencia
  float Y = 0;                // Salida
  float X0 = 0;               // Estado 0
  float X1 = 0;               // Estado 1
  float X2 = 0;               // Estado 2
  float U = 0;                // Salida control
  int Ui = 0;                 // Salida control tarjeta 

//----------  Observador --------//

//-- Estados obs --//
  float XeR1 = 0; // Estado estimado 1 en k, Xe1[k]
  float XeR2 = 0; // Estado estimado 2 en k, Xe2[k]
  float XeN1 = 0; // Estado estimado 1 en k+1, Xe1[k+1]
  float XeN2 = 0; // Estado estimado 2 en k+1, Xe2[k+1]
  float X0N = 0; // Estado integrador en k+1, X0[k+1]
  float f1 = 0; // Dinamica de observador
  float f2 = 0; // Dinamica de observador
  float f3 = 0; // Dinamica del integrador

//-- Modelo obs --//
  // Matriz A
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

  // Operaciones con Polos para Controlador y Observador
  float sum_a = -(a1r  + a2r + a3r);
  float prod_a = -( a1r*a2r - a1i*a2i )*a3r;
  float coef2 = (a1r*a3r + a2r*a3r + a1r*a2r);
  float sum_b = -(b1r  + b2r);
  float prod_b = b1r*b2r - b1i*b2i;
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

  // Ganancias de Controlador
  K0 = (-prod_a)/((Am11*Bm2 - Am21*Bm1)*Cm2 + (Am22*Bm1 - Am12*Bm2)*Cm1);
  K1 = (sum_a*(Am11*Bm2 - Am21*Bm1) + Bm2*(Am12*Am21 + Am11*Am11) + (Bm2*coef2 - K0*(Bm2*Bm2*Cm2 + Bm1*Bm2*Cm1)) - Bm1*(Am21*Am22 + Am11*Am21))/(Bm1*Bm2*(Am11 - Am22) + (Am12*Bm2*Bm2 - Am21*Bm1*Bm1)); 
  K2 = ((Am12*Bm2 - Am22*Bm1)*K1 + (Bm2*Cm2 + Bm1*Cm1)*K0 + (Am11*Am22 - Am12*Am21 - coef2))/(Am11*Bm2 - Am21*Bm1);

  //Ganancias Observador
  L1 = ((Am11*Cm2 - Am12*Cm1)*sum_b + Cm2*prod_b + (Am12*Am21 + Am11*Am11)*Cm2 - (Am12*Am22 + Am11*Am12)*Cm1)/(Am21*Cm2*Cm2 + (Am11 - Am22)*Cm1*Cm2 - Am12*Cm1*Cm1);
  L2 = ((Am21*Cm2 - Am22*Cm1)*sum_b + (Am21*Am22 + Am11*Am21)*Cm2 - (Am22*Am22 + Am12*Am21)*Cm1 - Cm1*prod_b)/(Am21*Cm2*Cm2 + (Am11 - Am22)*Cm1*Cm2 - Am12*Cm1*Cm1);
}

// ******************************************************** //
//---------- Rutina principal  --------//                  //
// ******************************************************** //

void loop() {                     
  proc_entradas();                    // Procesamiento de Entradas
  observador();                       // Observador
  control();                          // Control
  proc_salidas();                     // Procesado de Salidas
  coms_arduino_ide();               // Comunicaciones
  //coms_python(&R,&Y,&U);
  espera();
}

// ******************************************************** //
//---------- Rutinias de control y observador--------       //                          
// ******************************************************** //

//-- Control --//
void control(){
  // Ley de control
  //U = g*R - K1*XeR1 - K2*XeR2;       // Ley de control retro estado estimado
   U = K0*X0 - K1*XeR1 - K2*XeR2;       // Retroalimentación de estado mas control integral con estados de observador 
   
  // Saturacion
  if(U >= 5.0) U = 5.0;               // Saturacion de control en rango 0 a 5V                      
  else if(U < 0) U = 0;
}

//-- Observador para estimar X1 --//
void observador(){
  f1 = Am11*XeR1 + Am12*XeR2 + Bm1*U + L1*(Y-XeR2);     // Dinamica estado 1 observador
  f2 = Am21*XeR1 + Am22*XeR2 + Bm2*U + L2*(Y-XeR2);     // Dinamica estado 2 observador
  f3 = R - Y;

  XeN1 = XeR1 + Tseg*f1;              // Integrador estado 1 mediante Euler
  XeN2 = XeR2 + Tseg*f2;              // Integrador estado 2 mediante Euler
  X0N  = X0 + Tseg*f3;
  XeR1 = XeN1;
  XeR2 = XeN2;
  X0  = Habilitado*X0N;
}

// ******************************************************** //
//---------- Rutinias de IO y control de tiempo     --------//                          
// ******************************************************** //

//-- Procesado de entradas --//
void proc_entradas(){
  // No se ocupa leer X1 (Vc1) porque se estima con observador
  //X1 = analogRead(pX1)*mX+bX;            // Lectura de salida de planta en pin pX3
  X2 = analogRead(pX2)*mX+bX;               // Lectura de salida de planta en pin pX2
  R = Habilitado*(analogRead(pR)*mX+bX);    // Lectura de referencia en pin pR, Habilitado = {0,1} es para escalones
  Y = X2;
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
  Serial.print("r(t):");             // Referencia
  Serial.print(R);                   // Referencia valor
  Serial.print("  ");                 
  Serial.print("U(t):");             // Entrada
  Serial.print(U);                   // Entrada valor
  Serial.print("  ");                 
  Serial.print("y(t):");             // Salida 2 (C2) 
  Serial.println(Y);                 // Salida 2 (C2) valor
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

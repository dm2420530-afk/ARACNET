/*---------------------------------------------------------------/
                         ARACNET <3  
----------------------------------------------------------------*/
#include <IRremote.h> //libreria para el control remoto V.2.6!!
//:botones
#define boton_ON 0x1CE348B7  // encendido
#define boton_OFF 0x1CE3C837  // apagado
#define boton1 0x1CE39867    // estrategia 1    ATAQUE
#define boton2 0x1CE330CF    // estrategia 2     PID
#define boton3 0x1CE38877    // estrategia 3    
#define boton4 0x1CE3807F   // estrategia 4     TEST
#define boton5 0x1CE5320F // Cambia GIRO IZQUIERDA en ATAQUE() y PID()
#define boton6 0x1C325481 // Cambia GIRO DERECHA en ATAQUE() y PID()
//leds, buzzer y control
int led1 = 4, led2 = 13;
int buzzer = 12, control = A0;
//Sensores
int js200xf = A1; 
int d80_1 = A2, d80_2 = A3;   // d80_1 = izquierda   d80_2 = derecha
int linea1 = 2, linea2 = 3; // linea1 = izquierda   linea2 = derecha
//Motores TITAN
int LPWM_izq = 5, RPWM_izq= 6;  //Motor Izquierdo  
int LPWM_der = 10, RPWM_der = 9;  //Motor Derecho

//parametros y variables
const unsigned long tRegresivo = 5000; //para la cuenta regresiva
unsigned long t = 0; //para la cuenta regresiva
enum ESTADO{ESPERA, CONFIGURACION, REGRESIVO, GO};
ESTADO estadoActual = ESPERA;
enum LADO{NADA, IZQUIERDA, DERECHA};
LADO ultimo_lado = NADA;
int estrategiaSeleccionada = 0;
IRrecv irrecv(control);
decode_results codigo;
int ladoArranque = 1;
//int lecturaBorde = 0;

const int velocidad_BASE = 60; //-MODIFICAR-

float valorPosicion = 1.0; //-MODIFICAR-
float valor_Memoria = 4.0; //-MODIFICAR-

float kp = 20;    //-MODIFICAR-
float ki = 0;
float kd = 20;    //-MODIFICAR-
 
float error = 0;
float error_anterior = 0;
float integral = 0;
float derivada = 0;
float salidaPID = 0;
float ultima_direccion = 0;

bool bandera_SensorIzq = 0;
bool bandera_SensorCen = 0;
bool bandera_SensorDer = 0;
int sensores_activados = 0;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(led1, OUTPUT); pinMode(led2, OUTPUT); pinMode(buzzer, OUTPUT); 
  pinMode(js200xf, INPUT); pinMode(d80_1, INPUT); pinMode(d80_2, INPUT); pinMode(linea1, INPUT); pinMode(linea2, INPUT); pinMode(control, INPUT);
  pinMode(LPWM_izq, OUTPUT); pinMode(RPWM_izq, OUTPUT); pinMode(LPWM_der, OUTPUT); pinMode(RPWM_der, OUTPUT); 
}
void loop() {
  lecturaControl();
  switch(estadoActual){
    case ESPERA:
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    break;
    case CONFIGURACION:
    break;

    case REGRESIVO:
    tiempoSeguridad();
    break;

    case GO:
    ESTRATEGIAS();
    break;
  }
}
void lecturaControl(){
  if(irrecv.decode(&codigo)){
    unsigned long valor = codigo.value;
    if(valor == boton1 || valor == boton2 || valor == boton3 || valor == boton4 || valor == boton5 || valor == boton6){
      if(estadoActual == ESPERA || estadoActual == CONFIGURACION){
        
        if (valor == boton1){
          estrategiaSeleccionada = 1;
          digitalWrite(led1, HIGH);
          digitalWrite(buzzer, HIGH); delay(200); digitalWrite(buzzer, LOW); 
          digitalWrite(led2, LOW);
        } 
        if (valor == boton2){
          estrategiaSeleccionada = 2;
          digitalWrite(led1, LOW);
          digitalWrite(buzzer, HIGH); delay(100); digitalWrite(buzzer, LOW); 
          digitalWrite(led2, HIGH);
          digitalWrite(buzzer, HIGH); delay(100); digitalWrite(buzzer, LOW); 
        }
        if (valor == boton3){
          estrategiaSeleccionada = 3;
          digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(buzzer, HIGH);
          delay(50);
          digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(buzzer, LOW);
          delay(50); digitalWrite(buzzer, HIGH);
          delay(100); digitalWrite(buzzer, LOW);
        }
        if (valor == boton4){
          estrategiaSeleccionada = 4;
          digitalWrite(led1, LOW); digitalWrite(led2, LOW); digitalWrite(buzzer, HIGH);
          delay(50);
          digitalWrite(led1, HIGH); digitalWrite(led2, HIGH); digitalWrite(buzzer, LOW);
          delay(100); digitalWrite(buzzer, HIGH); digitalWrite(led1, LOW); digitalWrite(led2, LOW);
          delay(100); digitalWrite(buzzer, LOW); digitalWrite(led1, HIGH); digitalWrite(led2, HIGH);
        }
        if(valor == boton5){
          ladoArranque = -1;
          Serial.println("El lado de arranque es IZQUIERDO");
        }
        if(valor = boton6){
          ladoArranque = 1;
          Serial.println("El lado de arranque es DERECHO");
        }
        estadoActual = CONFIGURACION;
        Serial.print("La estrategia actual es:");
        Serial.println(estrategiaSeleccionada);
      }
    }else if (valor == boton_ON){
      if (estadoActual == CONFIGURACION){
        estadoActual = REGRESIVO;
        t = millis();
        Serial.print(estadoActual);
        Serial.println("ya empieza");
      }
    }else if (valor == boton_OFF){
        estadoActual = CONFIGURACION;
        ultimo_lado = NADA;
        integral = 0;
        error = 0;
        error_anterior = 0;
        ultima_direccion = 0;
        STOP();
        digitalWrite(led1, HIGH); digitalWrite(led2, HIGH);
        digitalWrite(buzzer, HIGH); delay(50); digitalWrite(buzzer, LOW); 
        digitalWrite(led1, LOW); digitalWrite(led2, LOW);
        digitalWrite(buzzer, HIGH); delay(50); digitalWrite(buzzer, LOW); 
        Serial.println("robot quieto.....");  
    }
    irrecv.resume();
  }
}
void tiempoSeguridad(){
  unsigned long tiempoTranscurrido = millis() - t;
  parpadeoLED();
  if(tiempoTranscurrido >= tRegresivo){
    estadoActual = GO;
    ultima_direccion = ladoArranque; // ARANCA, GIRANDO A LA DERECHA O IZQUIERDA segun boton5 y boton6
    integral = 0;
    error_anterior = 0;
    Serial.println("EMPEZO!!!!");  
  }
}
void parpadeoLED(){
  //los leds 1 y 2, van a papear #D
  static unsigned long tAnterior = 0;
  static bool estado = false;
  int intervalo = 250;
  if (millis() - tAnterior >= intervalo){
    estado = !estado;
    digitalWrite(led1, estado);
    digitalWrite(led2, estado);
    tAnterior = millis();
  }
}
void leds(){
  bool encenderLed1 = bandera_SensorIzq || bandera_SensorCen;
  bool encenderLed2 = bandera_SensorDer || bandera_SensorCen;
  digitalWrite(led1, encenderLed1);
  digitalWrite(led2, encenderLed2);
}
void STOP(){
  digitalWrite(LPWM_izq, HIGH);
  digitalWrite(RPWM_izq, HIGH);
  //frenamos en seco ambos Motores
  digitalWrite(LPWM_der, HIGH);
  digitalWrite(RPWM_der, HIGH);
}
void ESTRATEGIAS(){
  switch(estrategiaSeleccionada){
    case 1:
      ATAQUE();
    break;
    
    case 2:
      PID();
    break;

    case 3:
      Sentido_Motor();
    break;
    
    case 4:
      test_sensores();
    break;
  }
}
void ATAQUE() {
  int deteccionIzq = digitalRead(d80_1);
  int deteccionCen = digitalRead(js200xf); 
  int deteccionDer = digitalRead(d80_2);
  leds();

  if(deteccionIzq) ultimo_lado = IZQUIERDA;
  if(deteccionDer) ultimo_lado = DERECHA;

  if(!deteccionCen){
    motores(200,200);
  } else if (deteccionIzq){
    motores(150,-150);
  } else if (deteccionDer){ 
    motores(-150,150);
  } else {
    switch(ultimo_lado){
      case IZQUIERDA:
      motores(200, -200);
      break;

      case DERECHA:
      motores(-200, 200);
      break;

      case NADA: 
      motores(-ladoArranque *150, ladoArranque*150);
      break;
    }
  }
}
void PID(){
  int deteccionIzq = digitalRead(d80_1);
  int deteccionCen = digitalRead(js200xf); 
  int deteccionDer = digitalRead(d80_2);

  bandera_SensorIzq = (deteccionIzq == 1);
  bandera_SensorCen = (deteccionCen == 1);
  bandera_SensorDer = (deteccionDer == 1);
  leds();

  //ETAPA DEL ERROR.
  error = (-valorPosicion * bandera_SensorIzq) + (0.0 * bandera_SensorCen) + (valorPosicion * bandera_SensorDer);
  
  sensores_activados = bandera_SensorIzq + bandera_SensorCen + bandera_SensorDer;

  if(sensores_activados > 0){
    error = error / sensores_activados;
    ultima_direccion = error;  
  } else {  //LOGICA de memoria de deteccion//
    if(ultima_direccion > 0){        // +ERROR --> derecha
      error = valor_Memoria;
    } else if (ultima_direccion < 0 ){    // -ERROR --> izqueirda
      error = -valor_Memoria;
    } else {
      error = 0;
    }
  } 
  //CONTROL PID
  float P = kp * error;     //Parte Proporcional
  
  integral += error;
  integral = constrain(integral, -50, 50); //-MODIFICAR-
  float I = ki * integral;    //Parte Integral

  derivada = error - error_anterior;
  float D = kd * derivada;   //Parte derivativa.

  salidaPID = P + I + D;  //Resultado del PID, que luega va a los motores.

  error_anterior = error;  //Se retroalimenta.

  int velocidadMotorIzq = velocidad_BASE - salidaPID;
  int velocidadMotorDer = velocidad_BASE + salidaPID;

  Serial.print(error);
  Serial.print(",");
  Serial.print(P);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(D);
  Serial.print(",");
  Serial.print(salidaPID);
  Serial.print(",");
  Serial.print(velocidadMotorIzq);
  Serial.print(",");
  Serial.println(velocidadMotorDer);

  motores(velocidadMotorDer, velocidadMotorIzq);
  delay(1);
}
void motores(int velocidad_A, int velocidad_B){

  //limitar la velocidad de los TITAN
  velocidad_A = constrain(velocidad_A, -255, 255);
  velocidad_B = constrain(velocidad_B, -255, 255);

  //Motor DERECHO
  if(velocidad_A > 0){
    digitalWrite(LPWM_der, LOW);
    analogWrite(RPWM_der, velocidad_A);
  } else if (velocidad_A < 0){
    analogWrite(LPWM_der, -velocidad_A);
    digitalWrite(RPWM_der, LOW);
  } else {
    digitalWrite(LPWM_der, LOW);
    digitalWrite(RPWM_der, LOW);
  }
  //Motor IZQUIERDO
  if(velocidad_B > 0){
    digitalWrite(LPWM_izq, LOW);
    analogWrite(RPWM_izq, velocidad_B);
  } else if (velocidad_B < 0){
    analogWrite(LPWM_izq, -velocidad_B);
    digitalWrite(RPWM_izq, LOW);
  } else {
    digitalWrite(LPWM_izq, LOW);
    digitalWrite(RPWM_izq, LOW);
  }
}
//--------------TEST---------------//
void Sentido_Motor(){
  int velocidad_A = 150;
  int velocidad_B = 150;
  // Motor Derecho
  digitalWrite(LPWM_der, LOW); // aca tendria que girar ADELANTE
  analogWrite(RPWM_der, velocidad_A);
  delay(2000);
  analogWrite(LPWM_der, velocidad_A); // aca tendria girar hacia ATRAS
  digitalWrite(RPWM_der, LOW);
  delay(2000);
  digitalWrite(LPWM_der, LOW); // se detiene
  digitalWrite(RPWM_der, LOW);
  delay(1000);
  // Motor Izquierdo
  digitalWrite(LPWM_izq, LOW); // aca tendria que girar ADELANTE
  analogWrite(RPWM_izq, velocidad_B);
  delay(2000);
  analogWrite(LPWM_izq, velocidad_B); // aca tendria girar hacia ATRAS
  digitalWrite(RPWM_izq, LOW);
  delay(2000);
  digitalWrite(LPWM_izq, LOW); // se detiene
  digitalWrite(RPWM_izq, LOW);
  delay(1000);
}
void test_sensores(){
  //int bordeIZQ = analogRead(linea1); // Umbral entre =
  //int bordeDER = analogRead(linea2);
  //-----para despues----
  //bool bordeIZQ = (digitalRead(linea1) == lecturaBorde); // borde izquierdo
  //bool bordeDER = (digitalRead(linea2) == lecturaBorde); // borde derecho
  int deteccionIzq = digitalRead(d80_1);
  int deteccionCen = digitalRead(js200xf); 
  int deteccionDer = digitalRead(d80_2);

  Serial.print("IZQ: ");
  Serial.print(deteccionIzq);
  Serial.print(" | CENTRO: ");
  Serial.print(deteccionCen);
  Serial.print(" | DER: ");
  Serial.println(deteccionDer);
  delay(5);

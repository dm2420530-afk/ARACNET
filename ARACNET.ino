/*---------------------------------------------------------------/
                         ARACNET <3  
----------------------------------------------------------------*/
#include <IRremote.h> //libreria para el control remoto V.2.6!!
//:botones
#define boton_ON 0xFFA25D  // encendido
#define boton_OFF 0xFF629D  // apagado
#define boton1 0xFF30CF    // estrategia 1
#define boton2 0xFF18E7    // estrategia 2
#define boton3 0xFF7A85    // estrategia 3
#define boton4 0xFF10EF    // estrategia 4
//leds, buzzer y control
int led1 = A2, led2 = 12, led3 = 13;
int buzzer = A5, control = A1;
//Sensores
int TRIG_izq = 0, ECHO_izq = 0;  
int TRIG_der = 0, ECHO_der =0;
int js200xf = A3; 
int d80_1 = 0, d80_2 = 0;   // d80_1 = izquierda        d80_2 = derecha
int linea1 = 0, linea2 = 0; // linea1 = izquierda         linea2 = derecha
//Motores
int motorA1 = 10, motorA2 = 9, pwmMotorA = 0; //sigue el orden del diagrama.
int motorB1 = 3, motorB2 = 11, pwmMotorB = 0;

//parametros y variables
const unsigned long tRegresivo = 5000; //para la cuenta regresiva
unsigned long t = 0; //para la cuenta regresiva
enum ESTADO{ESPERA, CONFIGURACION, REGRESIVO, GO};
ESTADO estadoActual = ESPERA;
int estrategiaSeleccionada = 0;
IRrecv irrecv(control);
decode_results codigo;
int lecturaBorde = 0;

int limite_Objetivo = 100; //-MODIFICAR-
const int velocidad_BASE = 40; //-MODIFICAR-

float valor_HCSR04 = 2.0, valor_d80nk = 1.0; //-MODIFICAR-
float valor_Memoria = 4.0; //-MODIFICAR-

float kp = 20;    //-MODIFICAR-
float ki = 0;
float kd = 16;    //-MODIFICAR-
 
float error = 0;
float error_anterior = 0;
float integral = 0;
float derivada = 0;
float salidaPID = 0;
float ultima_direccion = 0;

bool bandera_SensorIzq = 0;
bool bandera_SensorAnguloIZQ = 0;
bool bandera_SensorCen = 0;
bool bandera_SensorAnguloDER = 0;
bool bandera_SensorDer = 0;
bool bordeIZQ = 0;
bool bordeDER = 0;
int sensores_activados = 0;

//pasaje de tiempo a distacia
long LecturaDistancia (int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 4500);
  long distancia = duracion * 0.034 / 2;
  return distancia;
}
void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(led1, OUTPUT); pinMode(led2, OUTPUT); pinMode(led3, OUTPUT); pinMode(buzzer, OUTPUT); 
  pinMode(js200xf, INPUT); pinMode(d80_1, INPUT); pinMode(d80_2, INPUT); pinMode(linea1, INPUT); pinMode(linea2, INPUT); pinMode(control, INPUT);
  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT); pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT); pinMode(pwmMotorA, OUTPUT); pinMode(pwmMotorB, OUTPUT);
  pinMode(TRIG_izq, OUTPUT); pinMode(ECHO_izq, INPUT); pinMode(TRIG_der, OUTPUT); pinMode(ECHO_der, INPUT);
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
    if(valor == boton1 || valor == boton2 || valor == boton3 || valor == boton4){
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
        }
        if (valor == boton4){
          estrategiaSeleccionada = 4;
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
        estadoActual = ESPERA;
        estrategiaSeleccionada = 0;
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
    Serial.print("EMPEZO!!!!");  
    
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
    digitalWrite(led3, estado);
    tAnterior = millis();
  }
}
void STOP(){
  analogWrite(pwmMotorA, 0); 
  analogWrite(pwmMotorB, 0);
}
void ESTRATEGIAS(){
  switch(estrategiaSeleccionada){
    case 1:
      PID();
    break;
    
    case 2:
    //defensa();
    break;

    case 3:
    //busqueda();
    break;
    
    case 4:
    //.....();
    break;
  }
}
void PID(){
  //priorizamos las lecturas del borde*
  bool bordeLinea1 = (digitalRead(linea1) == lecturaBorde); // borde izquierdo
  bool bordeLinea2 = (digitalRead(linea2) == lecturaBorde); // borde derecho
  
  if(bordeIZQ || bordeDER){
    esquivarLinea(bordeLinea1, bordeLinea2);
    return;
  }

  //comenzamos leyendo las distancias de cada sensor del robot
  int distanciaIzq = LecturaDistancia(TRIG_der, ECHO_der);
  int distanciaANGULO_1 = digitalRead(d80_1); //angulo de 35° izquierdo
  int distanciaCen = digitalRead(js200xf); 
  int distanciaDer = LecturaDistancia(TRIG_izq, ECHO_izq);
  int distanciaANGULO_2 = digitalRead(d80_2); //angulo de 35° derecho

  //preguntamos si detectan dentro del limite propuesto. *un SI es = 1, un NO = 0*
  bandera_SensorIzq = (distanciaIzq <= limite_Objetivo);
  bandera_SensorAnguloIZQ = (distanciaANGULO_1 == 0);
  bandera_SensorCen = (distanciaCen == 0);
  bandera_SensorAnguloDER = (distanciaANGULO_2 == 0);
  bandera_SensorDer = (distanciaDer <= limite_Objetivo);
  
  error = (-valor_HCSR04 * bandera_SensorIzq) + (-valor_d80nk * bandera_SensorAnguloIZQ) + (0.0 * bandera_SensorCen) + (valor_d80nk * bandera_SensorAnguloDER) + (valor_HCSR04 * bandera_SensorDer);
  sensores_activados = bandera_SensorIzq + bandera_SensorAnguloIZQ + bandera_SensorCen + bandera_SensorAnguloDER + bandera_SensorDer;

  //LOGICA de memoria de deteccion//
  if(sensores_activados > 0){         //promediamos el error entre los sensores, y guardamos los datos.
    error = error / sensores_activados;
    ultima_direccion = error;
  } else {  // situacion en cuando no se detecta nada en ningun sensor.
    if (ultima_direccion > 0){ // si el error guardado era postivo, se reemplaza por el de memoria mas alto (va a la izquierda)
      error = valor_Memoria;
    } else if (ultima_direccion < 0){ //si el error guardado era negativo, se reemplazo con uno negativo mas alto (va a la derecha)
      error = -valor_Memoria;
    } else {
      error = 0;
    }
  }

  float P = kp * error;     //Parte Proporcional
  
  integral = integral + error;
  integral = constrain(integral, -50, 50); //-MODIFICAR-
  float I = ki * integral;    //Parte Integral

  derivada = error - error_anterior;
  float D = kd * derivada;   //Parte derivativa.

  salidaPID = P + I + D;  //Resultado del PID, que luega va a los motores.

  error_anterior = error; // se retroalimenta.

  int velocidadMotorIzq = velocidad_BASE - salidaPID;
  int velocidadMotorDer = velocidad_BASE + salidaPID;

  motores(velocidadMotorDer, velocidadMotorIzq);
  delay(1);
}
void motores(int velocidad_A, int velocidad_B){
  //limitar la velocidad de los Titan

  velocidad_A = constrain(velocidad_A, -255, 255);
  velocidad_B = constrain(velocidad_B, -255, 255);

  //Motor Derecho
  if(velocidad_A > 0){
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    analogWrite(pwmMotorA, velocidad_A);
  } else if (velocidad_A < 0){
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    analogWrite(pwmMotorA, -velocidad_A);
  } else {
    analogWrite(pwmMotorA, 0);
  }

  //Motor Izquirdo
  if(velocidad_B >0){
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    analogWrite(pwmMotorB, velocidad_B);
  } else if (velocidad_B < 0){
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    analogWrite(pwmMotorB, -velocidad_B);
  } else {
    analogWrite(pwmMotorB, 0);
  }
}
void esquivarLinea(bool izq, bool der){
  motores(-150, -150);
  delay(120);  // el robot retrocede hacia atras por 1 vez.

  if(izq && !der){  // detecta el sensor izquierdo el borde
    motores(120, -120);
  } else if (der && !izq){ // detecta el sensor derecho el borde
    motores(-120, 120);
  } else {
    motores(-120, -120); // los 2 a la vez.
  }
  delay(200);

  integral = 0; //se resetean los valores para el PID
  error_anterior = 0;
}
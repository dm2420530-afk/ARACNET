/*---------------------------------------------------------------/
                         ARACNET <3  
----------------------------------------------------------------*/
//leds, buzzer y control
int led1 = 4, led2 = 13;
int buzzer = 11, boton = 12, botonDIP = A6; // boton es 
//Sensores
int TRIG_izq = 7, ECHO_izq = 8;
int TRIG_der = A4, ECHO_der = A5;
int js200xf = A1;
int d80_1 = A2, d80_2 = A3;  // d80_1 = izquierda        d80_2 = derecha
int linea1 = 2, linea2 = 3;  // linea1 = izquierda         linea2 = derecha
//Motores TITAN
int LPWM_izq = 5, RPWM_izq = 6;  //sigue el orden del diagrama.
int LPWM_der = 9, RPWM_der = 10;

//parametros y variables
const unsigned long tRegresivo = 5000;  //para la cuenta regresiva
unsigned long t = 0;                    //para la cuenta regresiva
enum ESTADO { ESPERA,
              CONFIGURACION,
              REGRESIVO,
              GO };
ESTADO estadoActual = ESPERA; // -EN ULTIMO CASO---> cambia a REGRESIVO
int estrategiaSeleccionada = 0; // -EN ULTIMO CASO---> elegi la estrategia 1,2,3,4....
bool botonAnterior = HIGH;

int lecturaBorde = 0;

int limite_Objetivo = 125;      //-MODIFICAR-
const int velocidad_BASE = 40;  //-MODIFICAR-

float valor_HCSR04 = 2.0, valor_d80nk = 1.0;  //-MODIFICAR-
float valor_Memoria = 4.0;                    //-MODIFICAR-

float kp = 20;  //-MODIFICAR-
float ki = 0;
float kd = 16;  //-MODIFICAR-

float error = 0;
float error_anterior = 0;
float integral = 0;
float derivada = 0;
float salidaPID = 0;
float velocidadActual = 0;
float ultima_direccion = 0;
float direccionInicialIZQ = 1.0;  //-MODIFICAR-

bool bandera_SensorIzq = 0;
bool bandera_SensorAnguloIZQ = 0;
bool bandera_SensorCen = 0;
bool bandera_SensorAnguloDER = 0;
bool bandera_SensorDer = 0;
int sensores_activados = 0;

//pasaje de tiempo a distacia
long LecturaDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 7000);  // 120cm max
  long distancia = duracion * 0.034 / 2;
  return distancia;
}
void setup() {
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(js200xf, INPUT);
  pinMode(d80_1, INPUT);
  pinMode(d80_2, INPUT);
  pinMode(linea1, INPUT);
  pinMode(linea2, INPUT);
  pinMode(boton, INPUT_PULLUP);
  pinMode(botonDIP, INPUT);
  pinMode(LPWM_izq, OUTPUT);
  pinMode(RPWM_izq, OUTPUT);
  pinMode(LPWM_der, OUTPUT);
  pinMode(RPWM_der, OUTPUT);
  pinMode(TRIG_izq, OUTPUT);
  pinMode(ECHO_izq, INPUT);
  pinMode(TRIG_der, OUTPUT);
  pinMode(ECHO_der, INPUT);
  velocidadActual = velocidad_BASE;
}
void loop() {
  lecturaBotones();
  switch (estadoActual) {
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
void lecturaBotones() {
  unsigned long valor = analogRead(botonDIP);
  bool estadoBoton = digitalRead(boton);
  if (estadoActual == ESPERA || estadoActual == CONFIGURACION) {

    if (valor > 80 && valor < 120) { // estrategia 1 PID(hacia delante)
      estrategiaSeleccionada = 1;
      digitalWrite(led1, HIGH);
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      digitalWrite(led2, LOW);
    }else if (valor > 180 && valor < 220) {  // estrategia 2 PID(con giro)
      estrategiaSeleccionada = 2;
      digitalWrite(led1, LOW);
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
      digitalWrite(led2, HIGH);
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
    }else if (valor > 380 && valor < 420) {  // estrategia 3 TEST(PID, quieto)
      estrategiaSeleccionada = 3;
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(buzzer, LOW);
      delay(50);
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
    }else if (valor > 580 && valor < 620) {  // estrategia 4 ....por el momento nada....
      estrategiaSeleccionada = 4;
    }
    estadoActual = CONFIGURACION;
    Serial.print("La estrategia actual es:");
    Serial.println(estrategiaSeleccionada);

  } else if (estadoBoton == LOW && botonAnterior == HIGH) {
    if (estadoActual == CONFIGURACION) {
     while (digitalRead(boton) == LOW);
     delay(5);
     estadoActual = REGRESIVO;
     t = millis();
      Serial.print(estadoActual);
      Serial.println("ya empieza");
    }
  } else {
    if (estadoActual == GO) {
      estadoActual = CONFIGURACION;
      STOP();
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
      Serial.println("robot quieto.....");
    }
  }
 botonAnterior = estadoBoton;
}
void tiempoSeguridad() {
  unsigned long tiempoTranscurrido = millis() - t;
  parpadeoLED();
  if (tiempoTranscurrido >= tRegresivo) {
    estadoActual = GO;
    Serial.println("EMPEZO!!!!");
  }
}
void parpadeoLED() {
  //los leds 1 y 2, van a papear #D
  static unsigned long tAnterior = 0;
  static bool estado = false;
  int intervalo = 250;
  if (millis() - tAnterior >= intervalo) {
    estado = !estado;
    digitalWrite(led1, estado);
    digitalWrite(led2, estado);

    tAnterior = millis();
  }
}
void STOP() {
  digitalWrite(LPWM_izq, HIGH);
  digitalWrite(RPWM_izq, HIGH);
  //frenamos en seco ambos Motores
  digitalWrite(LPWM_der, HIGH);
  digitalWrite(RPWM_der, HIGH);
}
void ESTRATEGIAS() {
  switch (estrategiaSeleccionada) {
    case 1:
      PID();
      break;

    case 2:
      //PID_IZQ();
      break;

    case 3:
      TEST_pid();
      break;

    case 4:
      test_sensores();
      break;
  }
}
void PID() {
  //priorizamos las lecturas del borde*
  bool bordeLinea1 = (digitalRead(linea1) == lecturaBorde);  // borde izquierdo
  bool bordeLinea2 = (digitalRead(linea2) == lecturaBorde);  // borde derecho

  if (bordeLinea1 || bordeLinea2) {
    esquivarLinea(bordeLinea1, bordeLinea2);
    return;
  }

  //comenzamos leyendo las distancias de cada sensor del robot
  int distanciaIzq = LecturaDistancia(TRIG_izq, ECHO_izq);
  int distanciaANGULO_1 = digitalRead(d80_1);  //angulo de 35° izquierdo
  int distanciaCen = digitalRead(js200xf);
  int distanciaDer = LecturaDistancia(TRIG_der, ECHO_der);
  int distanciaANGULO_2 = digitalRead(d80_2);  //angulo de 35° derecho

  //preguntamos si detectan dentro del limite propuesto. *un SI es = 1, un NO = 0*
  bandera_SensorIzq = (distanciaIzq <= limite_Objetivo);
  bandera_SensorAnguloIZQ = (distanciaANGULO_1 == 0);
  bandera_SensorCen = (distanciaCen == 0);
  bandera_SensorAnguloDER = (distanciaANGULO_2 == 0);
  bandera_SensorDer = (distanciaDer <= limite_Objetivo);

  error = (-valor_HCSR04 * bandera_SensorIzq) + (-valor_d80nk * bandera_SensorAnguloIZQ) + (0.0 * bandera_SensorCen) + (valor_d80nk * bandera_SensorAnguloDER) + (valor_HCSR04 * bandera_SensorDer);
  sensores_activados = bandera_SensorIzq + bandera_SensorAnguloIZQ + bandera_SensorCen + bandera_SensorAnguloDER + bandera_SensorDer;

  //LOGICA de memoria de deteccion//
  if (sensores_activados > 0) {  //promediamos el error entre los sensores, y guardamos los datos.
    error = error / sensores_activados;
    ultima_direccion = error;
  } else {                       // situacion en cuando no se detecta nada en ningun sensor.
    if (ultima_direccion > 0) {  // si el error guardado era postivo, se reemplaza por el de memoria mas alto (va a la izquierda)
      error = valor_Memoria;
    } else if (ultima_direccion < 0) {  //si el error guardado era negativo, se reemplazo con uno negativo mas alto (va a la derecha)
      error = -valor_Memoria;
    } else {
      error = 0;
    }
  }

  float P = kp * error;  //Parte Proporcional

  integral = integral + error;
  integral = constrain(integral, -50, 50);  //-MODIFICAR-
  float I = ki * integral;                  //Parte Integral

  derivada = error - error_anterior;
  float D = kd * derivada;  //Parte derivativa.

  salidaPID = P + I + D;  //Resultado del PID, que luega va a los motores.

  error_anterior = error;  //Se retroalimenta.

  //0.3 es una variable para AJUSTAR.
  bool alineado = ((sensores_activados > 0) && (abs(error)) < 0.3);  // preguntamos si el enemigo esta delante, aun si detectan los otros sensores por eso el abs() para que entre un error sin signo.
  int velocidadCrucero = alineado ? 240 : velocidad_BASE;            // preguntamos con un Op.ternario que si la variable "alineado" es TRUE o FALSE. TRUE = 240, FALSE = velocidad_BASE.
  velocidadActual = velocidadActual + (velocidadCrucero - velocidadActual) * 0.3;

  int velocidadMotorIzq = velocidadActual - salidaPID;
  int velocidadMotorDer = velocidadActual + salidaPID;

  motores(velocidadMotorDer, velocidadMotorIzq);
  delay(1);
}
/*void PID_IZQ(){
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
      error = valor_Memoria * direccionInicialIZQ;
    }
  }

  float P = kp * error;     //Parte Proporcional
  
  integral = integral + error;
  integral = constrain(integral, -50, 50); //-MODIFICAR-
  float I = ki * integral;    //Parte Integral

  derivada = error - error_anterior;
  float D = kd * derivada;   //Parte derivativa.

  salidaPID = P + I + D;  //Resultado del PID, que luega va a los motores.

  error_anterior = error;  //Se retroalimenta.
  bool alineado = ((sensores_activados > 0) && (abs(error)) < 0.3);  // preguntamos si el enemigo esta delante, aun si detectan los otros sensores por eso el abs() para que entre un error sin signo.
  int velocidadCrucero = alineado ? 240 : velocidad_BASE;  // preguntamos con un Op.ternario que si la variable "alineado" es TRUE o FALSE. TRUE = 240, FALSE = velocidad_BASE.
  velocidadActual = velocidadActual + (velocidadCrucero - velocidadActual) * 0.3;

  int velocidadMotorIzq = velocidadActual - salidaPID;
  int velocidadMotorDer = velocidadActual + salidaPID;

  motores(velocidadMotorDer, velocidadMotorIzq);
  delay(1);
}*/
/*void TEST_pid(){
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

  int velocidadMotorIzq = - salidaPID;
  int velocidadMotorDer = + salidaPID;

  motores(velocidadMotorDer, velocidadMotorIzq);
  delay(1);
}*/

void motores(int velocidad_A, int velocidad_B) {
  //limitar la velocidad de los Titan

  velocidad_A = constrain(velocidad_A, -255, 255);
  velocidad_B = constrain(velocidad_B, -255, 255);

  //Motor Derecho
  if (velocidad_A > 0) {
    digitalWrite(LPWM_der, LOW);
    analogWrite(RPWM_der, velocidad_A);
  } else if (velocidad_A < 0) {
    analogWrite(LPWM_der, -velocidad_A);
    digitalWrite(RPWM_der, LOW);
  } else {
    digitalWrite(LPWM_der, LOW);
    digitalWrite(RPWM_der, LOW);
  }

  //Motor Izquirdo
  if (velocidad_B > 0) {
    digitalWrite(LPWM_izq, LOW);
    analogWrite(RPWM_izq, velocidad_B);
  } else if (velocidad_B < 0) {
    analogWrite(LPWM_izq, -velocidad_B);
    digitalWrite(RPWM_izq, LOW);
  } else {
    digitalWrite(LPWM_izq, LOW);
    digitalWrite(RPWM_izq, LOW);
  }
}
void esquivarLinea(bool izq, bool der) {
  motores(-150, -150);
  delay(120);  // el robot retrocede hacia atras por 1 vez.

  if (izq && !der) {  // detecta el sensor izquierdo el borde
    motores(120, -120);
  } else if (der && !izq) {  // detecta el sensor derecho el borde
    motores(-120, 120);
  } else {
    motores(-120, -120);  // los 2 a la vez.
  }
  delay(200);

  integral = 0;  //se resetean los valores para el PID
  error_anterior = 0;
  velocidadActual = velocidad_BASE;
}
void test_sensores(){
  int bordeIZQ = analogRead(linea1); // Umbral entre =
  int bordeDER = analogRead(linea2);

  //-----para despues----
  //bool bordeIZQ = (digitalRead(linea1) == lecturaBorde); // borde izquierdo
  //bool bordeDER = (digitalRead(linea2) == lecturaBorde); // borde derecho

  int distanciaIzq = LecturaDistancia(TRIG_izq, ECHO_izq);
  int deteccionANGULO_1 = analogRead(d80_1); //angulo de 35° izquierdo
  int deteccionCen = analogRead(js200xf); 
  int deteccionANGULO_2 = analogRead(d80_2); //angulo de 35° derecho
  int distanciaDer = LecturaDistancia(TRIG_der, ECHO_der);

  if(distanciaIzq > 0 && distanciaIzq < limite_Objetivo){ // detecta el lado izquierdo: IZQ+D80_1
    digitalWrite(led1, HIGH); 
  } else if (deteccionANGULO_1 > 0 && deteccionANGULO_1 < 140){
    digitalWrite(led1, HIGH);
  } else { digitalWrite(led1, LOW);}
  if(deteccionCen){ // detecta el centro: JS200xf
    digitalWrite(led1, HIGH); digitalWrite(led2, HIGH);
  } else {digitalWrite(led1, LOW); digitalWrite(led2, LOW);}
  if(distanciaDer > 0 && distanciaDer < limite_Objetivo){ // detecta el lado derecho: DER+D80_2
    digitalWrite(led2, HIGH);
  } else if (deteccionANGULO_2 > 0 && deteccionANGULO_2 < 140){
    digitalWrite(led2, HIGH);
  } else { digitalWrite(led2, LOW);}
  
  Serial.print("IZQ: ");
  Serial.print(distanciaIzq);
  Serial.print(" cm |AnguloIzq: ");
  Serial.print(deteccionANGULO_1);
  Serial.print(" | CENTRO: ");
  Serial.print(deteccionCen);
  Serial.print(" | AnguloDer: ");
  Serial.print(deteccionANGULO_2);
  Serial.print(" | DER: ");
  Serial.print(distanciaDer);
  Serial.print(" cm | bordeIZQ: ");
  Serial.print(bordeIZQ);
  Serial.print(" | borderDer: ");
  Serial.println(bordeDER);
  delay(50);
}

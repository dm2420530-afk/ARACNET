/*---------------------------------------------------------------/
                         ARACNET <3
----------------------------------------------------------------*/
#include <IRremote.h>

#define boton_ON 0x1CE348B7
#define boton_OFF 0x1CE3C837
#define boton1 0x1CE39867  // Estrategia 1: ATAQUE
#define boton2 0x1CE330CF  // Estrategia 2: PID
#define boton3 0x1CE38877  // Estrategia 3: 
#define boton4 0x1CE3807F  // Estrategia 4: TEST
#define boton5 0x1CE5320F  // Salida izquierda
#define boton6 0x1C325481  // Salida derecha

int led1 = 4;
int led2 = 13;
int buzzer = 12;
int control = A0;
int js200xf = A1;
int d80_1 = A2;  // IZQUIERDA
int d80_2 = A3;  // DERECHA
int linea1 = 2;  // Línea izquierda
int linea2 = 3;  // Línea derecha
int LPWM_izq = 5;
int RPWM_izq = 6;
int LPWM_der = 10;
int RPWM_der = 9;

const unsigned long tRegresivo = 5000;
unsigned long t = 0;
enum ESTADO { ESPERA, CONFIGURACION, REGRESIVO, GO};
ESTADO estadoActual = ESPERA;
enum LADO { NADA, IZQUIERDA, DERECHA};
LADO ultimo_lado = NADA;
int ladoArranque = 1;
IRrecv irrecv(control);
decode_results codigo;
int estrategiaSeleccionada = 0;

const int velocidad_BASE = 180;
float kp = 26;
float ki = 0;
float kd = 20;

float posicion = 10;
float proporcional = 0;
float integral = 0;
float derivativo = 0;

float proporcional_pasado = 0;
float salida_control = 0;

int last_value = 10;

bool bandera_SensorIzq = 0;
bool bandera_SensorCen = 0;
bool bandera_SensorDer = 0;
int s1 = 0;
int s2 = 0;
int s3 = 0;
int sensores_activados = 0;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(js200xf, INPUT);
  pinMode(d80_1, INPUT);
  pinMode(d80_2, INPUT);
  pinMode(linea1, INPUT);
  pinMode(linea2, INPUT);
  pinMode(control, INPUT);
  pinMode(LPWM_izq, OUTPUT);
  pinMode(RPWM_izq, OUTPUT);
  pinMode(LPWM_der, OUTPUT);
  pinMode(RPWM_der, OUTPUT);
  STOP();
}
void loop() {
  lecturaControl();
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
void lecturaControl() {
  if (irrecv.decode(&codigo)) {
    unsigned long valor = codigo.value;
    if (valor == boton1 || valor == boton2 || valor == boton3 || valor == boton4 || valor == boton5 || valor == boton6) {
      if (estadoActual == ESPERA || estadoActual == CONFIGURACION) {

        if (valor == boton1) {
          estrategiaSeleccionada = 1;
          digitalWrite(led1, HIGH);
          digitalWrite(led2, LOW);
          digitalWrite(buzzer, HIGH);
          delay(200);
          digitalWrite(buzzer, LOW);
        }
        if (valor == boton2) {
          estrategiaSeleccionada = 2;
          digitalWrite(led1, LOW);
          digitalWrite(led2, HIGH);
          digitalWrite(buzzer, HIGH);
          delay(100);
          digitalWrite(buzzer, LOW);
          digitalWrite(buzzer, HIGH);
          delay(100);
          digitalWrite(buzzer, LOW);
        }
        if (valor == boton3) {
          estrategiaSeleccionada = 3;
          digitalWrite(led1, HIGH);
          digitalWrite(led2, HIGH);
          digitalWrite(buzzer, HIGH);
          delay(100);
          digitalWrite(buzzer, LOW);
        }
        if (valor == boton4) {
          estrategiaSeleccionada = 4;
          digitalWrite(led1, LOW);
          digitalWrite(led2, LOW);
          digitalWrite(buzzer, HIGH);
          delay(100);
          digitalWrite(buzzer, LOW);
        }
        if (valor == boton5) {
          ladoArranque = -1;
          Serial.println("SALIDA: IZQUIERDA");
        }
        if (valor == boton6) {
          ladoArranque = 1;
          Serial.println("SALIDA: DERECHA");
        }

        estadoActual = CONFIGURACION;
        Serial.print("Estrategia: ");
        Serial.println(estrategiaSeleccionada);
      }
    } else if (valor == boton_ON) {
      if (estadoActual == CONFIGURACION) {
        STOP();
        t = millis();
        estadoActual = REGRESIVO;
        Serial.println("ya comienza...");
      }
    } else if (valor == boton_OFF) {
      estadoActual = CONFIGURACION;
      ultimo_lado = NADA;
      integral = 0;
      proporcional = 0;
      proporcional_pasado = 0;
      derivativo = 0;
      salida_control = 0;
      last_value = 10;
      STOP();
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      Serial.println("ROBOT DETENIDO");
    }
    irrecv.resume();
  }
}
void tiempoSeguridad() {
  unsigned long tiempoTranscurrido = millis() - t;
  parpadeoLED();
  if (tiempoTranscurrido >= tRegresivo) {
    Serial.println("GO!");
    integral = 0;
    proporcional_pasado = 0;
    salida_control = 0;
    posicion = 10;
    last_value = 10;
    estadoActual = GO;
  }
}
void parpadeoLED() {
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
void leds() {
  bool encenderLed1 = bandera_SensorIzq || bandera_SensorCen;
  bool encenderLed2 = bandera_SensorDer || bandera_SensorCen;
  digitalWrite(led1, encenderLed1);
  digitalWrite(led2, encenderLed2);
}
void STOP() {
  digitalWrite(LPWM_izq, HIGH);
  digitalWrite(RPWM_izq, HIGH);
  digitalWrite(LPWM_der, HIGH);
  digitalWrite(RPWM_der, HIGH);
}
void ESTRATEGIAS() {
  switch (estrategiaSeleccionada) {
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
  
  sensores();
  leds();

  bandera_SensorIzq = s1;
  bandera_SensorCen = s2;
  bandera_SensorDer = s3;
  
  if (s1) ultimo_lado = IZQUIERDA;
  if (s3) ultimo_lado = DERECHA;

  if (s2) {
    motores(200, 200);
  } else if (s1) {
    motores(150, -150);
  } else if (s3) {
    motores(-150, 150);
  } else {
    switch (ultimo_lado) {
      case IZQUIERDA:
        motores(200, -200);
        break;

      case DERECHA:
        motores(-200, 200);
        break;

      case NADA:
        motores(-ladoArranque * 150, ladoArranque * 150);
        break;
    }
  }
}
void PID() {
  tracking3();
}
void tracking3() {

  sensores();
  leds();

  posicion = ponderacion();

  proporcional = posicion - 10;
  
  integral = integral + proporcional_pasado;
  int ITerm = integral * ki;

  if(ITerm >= 250) ITerm = 250;
  if(ITerm <= -250) ITerm = -250;

  derivativo = proporcional - proporcional_pasado;

  salida_control = (proporcional * kp) + (derivativo * kd) + ITerm;

  if(salida_control > velocidad_BASE) salida_control = velocidad_BASE;
  if(salida_control < -velocidad_BASE) salida_control = -velocidad_BASE;

  proporcional_pasado = proporcional;

  if(salida_control < 0) {
    motores(velocidad_BASE, velocidad_BASE + salida_control);
  } else if (salida_control > 0){
    motores(velocidad_BASE - salida_control, velocidad_BASE);
  } else {
    motores(velocidad_BASE, velocidad_BASE);
  }

  Serial.print("POS: ");
  Serial.print(posicion);

  Serial.print(" ERR: ");
  Serial.print(proporcional);

  Serial.print(" P: ");
  Serial.print(proporcional * kp);

  Serial.print(" D: ");
  Serial.print(derivativo * kd);

  Serial.print(" PID: ");
  Serial.print(salida_control);

  Serial.print(" IZQ: ");

  if (salida_control < 0)
    Serial.print(velocidad_BASE);
  else
    Serial.print(velocidad_BASE - salida_control);

  Serial.print(" DER: ");

  if (salida_control < 0)
    Serial.println(velocidad_BASE + salida_control);
  else
    Serial.println(velocidad_BASE);

}
void sensores(){
  //s3 = derecha
  //s2 = centro
  //s1 = izqueirda
  s3 = !digitalRead(d80_1);
  s2 = !digitalRead(js200xf);
  s1 = !digitalRead(d80_2);
}
int ponderacion(){
  int sensores_values[3];

  long avg = 0;
  long sum = 0;

  bool rival = false;

  sensores();

  sensores_values[0] = s3;
  sensores_values[1] = s2;
  sensores_values[2] = s1;

  for(int i = 0; i < 3; i++){
    int value = sensores_values[i];

    if(value > 0){
      rival = true;
    /*i = 0 -> izquierda -> 20
      i = 1 -> centro -> 10
      i = 2 -> izquierda -> 0*/
      avg += (long)value * ((2 -i) * 10);
      sum += value;
    }
  }

  if(!rival){
    return last_value;
  }
  last_value = avg / sum;
  return last_value;
}

void motores(int velocidad_A, int velocidad_B) {

  velocidad_A = constrain(velocidad_A, -255, 255);
  velocidad_B = constrain(velocidad_B, -255, 255);

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
void Sentido_Motor() {
  int velocidad_A = 150;
  int velocidad_B = 150;
  // MOTOR DERECHO
  digitalWrite(LPWM_der, LOW);
  analogWrite(RPWM_der, velocidad_A);
  delay(2000);
  analogWrite(LPWM_der, velocidad_A);
  digitalWrite(RPWM_der, LOW);
  delay(2000);
  digitalWrite(LPWM_der, LOW);
  digitalWrite(RPWM_der, LOW);
  delay(1000);
  // MOTOR IZQUIERDO
  digitalWrite(LPWM_izq, LOW);
  analogWrite(RPWM_izq, velocidad_B);
  delay(2000);
  analogWrite(LPWM_izq, velocidad_B);
  digitalWrite(RPWM_izq, LOW);
  delay(2000);
  digitalWrite(LPWM_izq, LOW);
  digitalWrite(RPWM_izq, LOW);
  delay(1000);
}

void test_sensores() {
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
}

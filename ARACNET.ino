/*---------------------------------------------------------------/
                         ARACNET <3
----------------------------------------------------------------*/
#include <IRremote.h>

#define boton_ON  0x1CE348B7
#define boton_OFF 0x1CE3C837
#define boton1 0x1CE39867    // Estrategia 1: LICUADORA + ATAQUE
#define boton2 0x1CE330CF    // Estrategia 2: COJA + PID
#define boton3 0x1CE38877    // Estrategia 3: SEMICOJA + PID
#define boton4 0x1CE3807F    // Estrategia 4: TEST
#define boton5 0x1CE5320F    // Salida izquierda
#define boton6 0x1C325481    // Salida derecha

int led1 = 4;
int led2 = 13;
int buzzer = 12;
int control = A0;
int js200xf = A1;
int d80_1 = A2;       // IZQUIERDA
int d80_2 = A3;       // DERECHA
int linea1 = 2;       // Línea izquierda
int linea2 = 3;       // Línea derecha
int LPWM_izq = 5;
int RPWM_izq = 6;
int LPWM_der = 10;
int RPWM_der = 9;

const unsigned long tRegresivo = 5000;
unsigned long t = 0;
enum ESTADO {ESPERA, CONFIGURACION, REGRESIVO, SALIDA, GO};
ESTADO estadoActual = ESPERA;
enum LADO { NADA, IZQUIERDA, DERECHA};
LADO ultimo_lado = NADA;
int ladoArranque = 1;
IRrecv irrecv(control);
decode_results codigo;
int estrategiaSeleccionada = 0;
enum SALIDA_TIPO {SALIDA_LICUADORA, SALIDA_COJA, SALIDA_SEMICOJA};
SALIDA_TIPO salidaActual;
int etapaSalida = 0;
unsigned long tiempoSalida = 0;
bool rivalDetectadoDuranteSalida = false;

const unsigned long TIEMPO_COJA = 900;
const unsigned long TIEMPO_SEMICOJA = 200;
const unsigned long TIEMPO_LICUADORA = 900;

const int velocidad_BASE = 180;
float valor_Memoria = 4.0;
float kp = 20;
float ki = 0;
float kd = 20;
float error = 0;
float error_anterior = 0;
float integral = 0;
float derivada = 0;
float salidaPID = 0;
float ultima_direccion = 0;
float posicion = 10;

bool bandera_SensorIzq = 0;
bool bandera_SensorCen = 0;
bool bandera_SensorDer = 0;
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

    case SALIDA:
      ejecutarSalida();
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
          digitalWrite(led1, HIGH); digitalWrite(led2, LOW);
          digitalWrite(buzzer, HIGH);
          delay(200);
          digitalWrite(buzzer, LOW);
        }
        if (valor == boton2) {
          estrategiaSeleccionada = 2;
          digitalWrite(led1, LOW); digitalWrite(led2, HIGH);
          digitalWrite(buzzer, HIGH);
          delay(100); digitalWrite(buzzer, LOW);
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
      error = 0;
      error_anterior = 0;
      salidaPID = 0;
      ultima_direccion = 0;
      etapaSalida = 0;
      STOP();
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(buzzer, HIGH);
      delay(50); digitalWrite(buzzer, LOW);
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
    error_anterior = 0;
    salidaPID = 0;
    if (estrategiaSeleccionada == 1) {
      salidaActual = SALIDA_LICUADORA;
      iniciarSalida();
    } else if (estrategiaSeleccionada == 2) {
      salidaActual = SALIDA_COJA;
      iniciarSalida();
    } else if (estrategiaSeleccionada == 3) {
      salidaActual = SALIDA_SEMICOJA;
      iniciarSalida();
    } else {
      // Estrategia 4 = TEST
      estadoActual = GO;
    }
  }
}
void iniciarSalida() {

  etapaSalida = 0;
  tiempoSalida = millis();
  rivalDetectadoDuranteSalida = false;
  estadoActual = SALIDA;
  Serial.println("COMIENZA MANIOBRA DE SALIDA");
}
void ejecutarSalida() {
  bool izq = digitalRead(d80_1);
  bool cen = digitalRead(js200xf);
  bool der = digitalRead(d80_2);

  if (izq || cen || der) {

    Serial.println("RIVAL ENCONTRADO DURANTE SALIDA");
    integral = 0;
    error_anterior = 0;
    estadoActual = GO;
    return;
  }
  
  if (salidaActual == SALIDA_LICUADORA) {

    if (ladoArranque == -1) {
      motores(-195, 195);
    } else {
      motores(195, -195);
    }

    if (millis() - tiempoSalida >= TIEMPO_LICUADORA) {
      estadoActual = GO;
      Serial.println("FIN LICUADORA -> ATAQUE");
    }
    return;
  }

  if (salidaActual == SALIDA_COJA) {
    if (ladoArranque == -1) {
      motores(20, 255);
    } else {
      motores(255, 20);
    }

    if ( millis() - tiempoSalida >= TIEMPO_COJA) {
      if (ladoArranque == -1) {
        motores(-200, 200);
      } else {
        motores(200, -200);
      }
      delay(1);
      tiempoSalida = millis();
      etapaSalida = 1;
    }


    // Segunda etapa de la coja
    if (etapaSalida == 1) {

      if (
        millis() - tiempoSalida >=
        300
      ) {

        estadoActual = GO;

        Serial.println("FIN COJA -> PID");
      }
    }

    return;
  }

  if (salidaActual == SALIDA_SEMICOJA) {

    if (ladoArranque == -1) {

      motores(140, 250);
    } else {

      motores(250, 140);
    }


    if (
      millis() - tiempoSalida >=
      TIEMPO_SEMICOJA
    ) {

      if (ladoArranque == -1) {

        motores(-180, 180);
      }

      else {

        motores(180, -180);
      }

      tiempoSalida = millis();

      etapaSalida = 1;
    }


    if (etapaSalida == 1) {

      if (
        millis() - tiempoSalida >=
        250
      ) {

        estadoActual = GO;

        Serial.println("FIN SEMICOJA -> PID");
      }
    }
    return;
  }
}
void parpadeoLED() {
  static unsigned long tAnterior = 0;
  static bool estado = false;
  int intervalo = 250;
  if ( millis() - tAnterior >= intervalo) {
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

  int deteccionIzq = digitalRead(d80_1);
  int deteccionCen = digitalRead(js200xf);
  int deteccionDer = digitalRead(d80_2);

  bandera_SensorIzq = deteccionIzq;
  bandera_SensorCen = deteccionCen;
  bandera_SensorDer = deteccionDer;
  leds();

  if (deteccionIzq) ultimo_lado = IZQUIERDA;
  if (deteccionDer) ultimo_lado = DERECHA;

  if (!deteccionCen) {
    motores(200, 200);
  } else if (deteccionIzq) {
    motores(150, -150);
  } else if (deteccionDer) {
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
        motores( -ladoArranque * 150, ladoArranque * 150);
        break;
    }
  }
}
void PID() {
  tracking3();
}
void tracking3() {
  bandera_SensorIzq = digitalRead(d80_1);
  bandera_SensorCen = digitalRead(js200xf);
  bandera_SensorDer = digitalRead(d80_2);
  leds();

  if (bandera_SensorIzq) ultima_direccion = 1;
  if (bandera_SensorDer) ultima_direccion = -1;

  int suma = 0;
  sensores_activados = 0;

  // IZQUIERDA = 20
  if (bandera_SensorIzq) {
    suma += 20;
    sensores_activados++;
  }
  // CENTRO = 10
  if (bandera_SensorCen) {
    suma += 10;
    sensores_activados++;
  }
  // DERECHA = 0
  if (bandera_SensorDer) {
    suma += 0;
    sensores_activados++;
  }

  if (sensores_activados > 0) {
    posicion = (float)suma / sensores_activados;
    error = posicion - 10.0;
  } else {
    if (ultima_direccion > 0) {
      error = valor_Memoria;
    } else if (ultima_direccion < 0) {
      error = -valor_Memoria;
    } else {
      error = 0;
    }
  }

  float P = kp * error;

  integral += error;
  integral = constrain( integral, -50, 50);
  float I = ki * integral;

  derivada = error -error_anterior;
  float D = kd * derivada;

  salidaPID = P + I + D;

  salidaPID = constrain(salidaPID, -velocidad_BASE, velocidad_BASE);
  error_anterior = error;

  int velocidadMotorIzq = velocidad_BASE -salidaPID;
  int velocidadMotorDer = velocidad_BASE + salidaPID;

  if (bandera_SensorIzq && bandera_SensorCen && bandera_SensorDer) {
    velocidadMotorIzq = 255;
    velocidadMotorDer = 255;
  }
  
  Serial.print("POS:");
  Serial.print(posicion);
  Serial.print(" ERROR:");
  Serial.print(error);
  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" PID:");
  Serial.print(salidaPID);
  Serial.print(" IZQ:");
  Serial.print(velocidadMotorIzq);
  Serial.print(" DER:");
  Serial.println(velocidadMotorDer);
  
  motores(velocidadMotorDer, velocidadMotorIzq);
}
void motores( int velocidad_A, int velocidad_B) {

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
    analogWrite( LPWM_izq, -velocidad_B);
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

const int sensorIzquierda = 2 ;
const int sensorCentro = 4;
const int sensorDerecha = 7 ;
const int motor1Pin1 = 10;  // Pin 1 del motor 1
const int motor1Pin2 = 9;  // Pin 2 del motor 1
const int motor2Pin1 = 6;  // Pin 1 del motor 2
const int motor2Pin2 = 5;  // Pin 2 del motor 2
const int enablemotor1 = 11;
const int enablemotor2 = 3;
int contador = 0;
bool bandera = false;

void setup() {
  pinMode(sensorIzquierda, INPUT);
  pinMode(sensorCentro, INPUT);
  pinMode(sensorDerecha, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablemotor1, OUTPUT);
  pinMode(enablemotor2, OUTPUT);
  Serial.begin(9600);
  
}

void seguir_linea(){
  int LecturaIzquierda = digitalRead(sensorIzquierda);
  int LecturaDerecha = digitalRead(sensorDerecha);
  int LecturaCentro = digitalRead(sensorCentro);

  if ((LecturaIzquierda == LOW) && (LecturaCentro == HIGH) && (LecturaDerecha == HIGH)){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enablemotor2, 80);
  }
  else if ((LecturaIzquierda == HIGH) && (LecturaCentro == HIGH) && (LecturaDerecha == LOW)){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enablemotor1, 80);
  }
  else if ((LecturaIzquierda == HIGH) && (LecturaCentro == LOW) && (LecturaDerecha == HIGH)){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(enablemotor1, 80);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enablemotor2, 80);
  }
}

bool detener(){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);  
}

void loop() {

  seguir_linea();
  bool bandera = true;
  int LecturaIzquierda = digitalRead(sensorIzquierda);
  int LecturaDerecha = digitalRead(sensorDerecha);
  int LecturaCentro = digitalRead(sensorCentro);

  if((LecturaIzquierda == LOW) && (LecturaCentro == LOW) && (LecturaDerecha == LOW)){
    //mover servos 
    detener();
    delay(400);
    contador = 0 ;
    if (bandera ==true && contador == 0){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(enablemotor1, 80);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enablemotor2, 0);
    delay(500);
    bandera = false;
    contador += 1;
    seguir_linea();
    int LecturaIzquierda = digitalRead(sensorIzquierda);
    int LecturaDerecha = digitalRead(sensorDerecha);
    int LecturaCentro = digitalRead(sensorCentro);
    if((LecturaIzquierda == LOW) && (LecturaCentro == LOW) && (LecturaDerecha == LOW)){
       detener();
      delay(400);
       if (bandera ==false && contador == 1){
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        analogWrite(enablemotor1, 0);
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
        analogWrite(enablemotor2, 80);
        delay(500);
        bandera = false;
        contador += 1;
          seguir_linea();
      }
    }
  }
  }
}

  
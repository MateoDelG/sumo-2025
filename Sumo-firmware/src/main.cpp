#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"

#define LED_BLUE 17
#define LED_RED 16
#define LED_ORANGE 0
#define LED_GREEN 2

#define BUTTON 32

#define MOTOR_RIGHT_A 12
#define MOTOR_RIGHT_B 14
#define MOTOR_RIGHT_PWM 27

#define MOTOR_LEFT_A 26
#define MOTOR_LEFT_B 25
#define MOTOR_LEFT_PWM 33

int velocidad = 255; // Velocidad del motor (0-255)

#define XSHUT_PIN_LEFT 19
#define XSHUT_PIN_RIGHT 18

#define VL6180X_ADDR_LEFT 0x29  // Dirección por defecto
#define VL6180X_ADDR_RIGHT 0x2A // Nueva dirección para segundo sensor

#define TRC5000_LEFT 4
#define TRC5000_RIGHT 15

Adafruit_VL6180X vl_left;
Adafruit_VL6180X vl_right;

void setupLEDs();

void setupMotors();
void forward();
void backward();
void turnLeft();
void turnRight();
void stopMotors();

void testLED();
void testMotor();
void testVL6180X();

void setVL6180XID();
int readVL6180XSensor(Adafruit_VL6180X &vl);
bool detectObstacle(Adafruit_VL6180X &vl);

void setupTRC5000();
bool readTRC5000(int sensor);

void controlBot();

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting setup...");

  setupLEDs();
  setupMotors();
  setupTRC5000();
  setVL6180XID();

  Serial.println("Setup complete");
}

void loop()
{
  // testLED();
  // testMotor();
  // testVL6180X();

  // readVL6180XSensor(vl_left);
  // Serial.print("   |   ");
  // readVL6180XSensor(vl_right);
  // Serial.println();
  // delay(300);
  controlBot();
  // readTRC5000(TRC5000_LEFT);
  // readTRC5000(TRC5000_RIGHT);
  // delay(500);
}

void testLED()
{
  static int buttonCount = 0;

  if (digitalRead(BUTTON) == LOW)
  {
    buttonCount++;
    Serial.print("Button pressed ");
    Serial.println(buttonCount);

    // Toggle LEDs based on button count
    if (buttonCount % 4 == 1)
    {
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, LOW);
      digitalWrite(LED_GREEN, LOW);
    }
    else if (buttonCount % 4 == 2)
    {
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_ORANGE, LOW);
      digitalWrite(LED_GREEN, LOW);
    }
    else if (buttonCount % 4 == 3)
    {
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, HIGH);
      digitalWrite(LED_GREEN, LOW);
    }
    else
    {
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, LOW);
      digitalWrite(LED_GREEN, HIGH);
      buttonCount = 0; // Reset after all LEDs have been lit
    }

    delay(500); // Debounce delay
  }
}
void setupLEDs()
{
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);
  delay(1000);
  pinMode(BUTTON, INPUT);

  // Initialize LEDs to off
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_GREEN, LOW);
}

void testMotor()
{
  // Example motor control logic
  digitalWrite(MOTOR_RIGHT_PWM, HIGH); // Enable motor
  digitalWrite(MOTOR_LEFT_PWM, HIGH);

  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
  delay(2000); // Run motors for 2 seconds
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
  delay(2000);                        // Reverse motors for 2 seconds
  digitalWrite(MOTOR_RIGHT_PWM, LOW); // Disable motor
  digitalWrite(MOTOR_LEFT_PWM, LOW);  // Disable left motor
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  delay(2000); // Wait before next operation
}
void setupMotors()
{
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_RIGHT_PWM, LOW);

  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_LEFT_PWM, LOW);
}
void forward()
{
  analogWrite(MOTOR_RIGHT_PWM, velocidad);
  analogWrite(MOTOR_LEFT_PWM, velocidad);
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}
void backward()
{
  analogWrite(MOTOR_RIGHT_PWM, velocidad);
  analogWrite(MOTOR_LEFT_PWM, velocidad);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}
void turnLeft()
{
  analogWrite(MOTOR_RIGHT_PWM, velocidad);
  analogWrite(MOTOR_LEFT_PWM, velocidad);
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}
void turnRight()
{
  analogWrite(MOTOR_RIGHT_PWM, velocidad);
  analogWrite(MOTOR_LEFT_PWM, velocidad);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}
void stopMotors()
{
  analogWrite(MOTOR_RIGHT_PWM, 0);
  analogWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void testVL6180X()
{
  // Leer distancia en milímetros
  uint8_t distancia = vl_left.readRange();
  uint8_t status = vl_left.readRangeStatus();

  if (status == VL6180X_ERROR_NONE)
  {
    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.println(" mm");
  }
  else
  {
    Serial.print("Error de lectura. Código: ");
    Serial.println(status);
  }

  delay(500);
}
void setVL6180XID()
{
  Wire.begin();
  // Apaga ambos
  pinMode(XSHUT_PIN_LEFT, OUTPUT);
  pinMode(XSHUT_PIN_RIGHT, OUTPUT);
  digitalWrite(XSHUT_PIN_LEFT, LOW);
  digitalWrite(XSHUT_PIN_RIGHT, LOW);
  delay(10);

  // --- 1) Enciende SOLO el RIGHT (LEFT sigue LOW) ---
  digitalWrite(XSHUT_PIN_RIGHT, HIGH);
  delay(50); // tiempo para boot del VL6180X

  // Inicializa RIGHT en 0x29 (no hay conflicto porque LEFT está apagado)
  if (!vl_right.begin())
  {
    Serial.println(F("Failed to boot right VL6180X"));
    while (1)
      delay(10);
  }

  // Cambia RIGHT a 0x2A
  vl_right.setAddress(VL6180X_ADDR_RIGHT);
  delay(5);

  // --- 2) Enciende ahora el LEFT ---
  digitalWrite(XSHUT_PIN_LEFT, HIGH);
  delay(50);

  // Inicializa LEFT en 0x29 (ya no hay nadie en 0x29 excepto él)
  if (!vl_left.begin())
  {
    Serial.println(F("Failed to boot left VL6180X"));
    while (1)
      delay(10);
  }
}
int readVL6180XSensor(Adafruit_VL6180X &vl)
{
  // Serial.print(" Addr: 0x");
  // Serial.print(vl.getAddress(), HEX);

  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE)
  {
    // Serial.print("  Range: "); Serial.print(range); Serial.print(" mm");
    return range;
  }
  else
  {
    // Serial.print("  Err: "); Serial.print(status);
    return -1; // Error de lectura
  }
}
bool detectObstacle(Adafruit_VL6180X &vl)
{
  if (readVL6180XSensor(vl) != -1)
  {
    // Serial.println("Obstacle detected!");
    return true;
  }
  else
  {
    // Serial.println("No obstacle detected.");
    return false;
  }
}

void setupTRC5000()
{
  pinMode(TRC5000_LEFT, INPUT);
  pinMode(TRC5000_RIGHT, INPUT);
}
bool readTRC5000(int sensor)
{
  int measure = -1;
  const int treshold = 1600;

  if (sensor == TRC5000_LEFT)
  {
    measure = analogRead(TRC5000_LEFT);
    measure = measure - 460;
    // Serial.print("TRC5000 Left: ");
    // Serial.println(measure);
    if (measure > treshold)
    {

      return true;
    }
    else
    {

      return false;
    }
  }
  else if (sensor == TRC5000_RIGHT)
  {
    measure = analogRead(TRC5000_RIGHT);
    // Serial.print("TRC5000 Right: ");
    // Serial.println(measure);
    if (measure > treshold)
    {

      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    Serial.println("Invalid TRC5000 sensor");
    return false;
  }
}

void controlBot(){

  bool obstacleLeft = detectObstacle(vl_left);
  bool obstacleRight = detectObstacle(vl_right);
  // bool trcLeft = readTRC5000(TRC5000_LEFT);
  // bool trcRight = readTRC5000(TRC5000_RIGHT);
  bool trcLeft = false;
  bool trcRight = false;

  // Lógica de control del robot
  if (trcLeft || trcRight) {
    // stopMotors();
    // backward();
    // delay(500);
    // turnLeft();
    // delay(1000);
    // stopMotors();
    delay(100);
  }
  else if (obstacleLeft && !obstacleRight) {
    Serial.println("Obstacle on the left, turning left");
    digitalWrite(LED_RED, HIGH);
    turnLeft();
  }
  else if (!obstacleLeft && obstacleRight) {
    Serial.println("Obstacle on the right, turning right");
    digitalWrite(LED_GREEN, HIGH);
    turnRight();
  }
  else if (obstacleLeft && obstacleRight) {
    forward();
    Serial.println("Obstacles on both sides, moving forward");
  }
  else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    stopMotors();
  }
}

// void controlBot() {
//   bool obstacleLeft  = detectObstacle(vl_left);
//   bool obstacleRight = detectObstacle(vl_right);
//   bool trcLeft       = readTRC5000(TRC5000_LEFT);
//   bool trcRight      = readTRC5000(TRC5000_RIGHT);

//   digitalWrite(LED_GREEN, LOW);



//   // Variables estáticas para control de estado y tiempo
//   static unsigned long stateStartTime = millis();

//   velocidad = 255; // velocidad por defecto

//   // --- Casos con detección inmediata de obstáculos o línea ---
//   if (trcLeft || trcRight) {
//     digitalWrite(LED_RED, HIGH);
//     stopMotors();
//     backward();
//     delay(300);
//     // stopMotors();
//     // delay(300);
//     turnLeft();
//     delay(600);
//     stopMotors();
//     stateStartTime = millis();
//     digitalWrite(LED_RED, LOW);
//     return;
//   }
//   else if (obstacleLeft && !obstacleRight) {
//     Serial.println("Obstacle on the left, turning left");
//     turnLeft();
//     stateStartTime = millis();
//     return;
//   }
//   else if (!obstacleLeft && obstacleRight) {
//     Serial.println("Obstacle on the right, turning right");
//     turnRight();
//     stateStartTime = millis();
//     return;
//   }
//   else if (obstacleLeft && obstacleRight) {
//     Serial.println("Obstacles on both sides, moving forward");
//     forward();
//     stateStartTime = millis();
//     return;
//   }
  
//   else if (!obstacleLeft && !obstacleRight && !trcLeft && !trcRight) {
//     digitalWrite(LED_GREEN, HIGH);
//           // Configuración de tiempos de cada estado
//       const unsigned long forwardDuration = 1000; // ms
//       const unsigned long pauseDuration   = 50;  // ms
//       const unsigned long turnDuration    = 500;  // ms
//       static int state = 0; // 0=avanzar, 1=pausar, 2=girar

//       unsigned long elapsed = millis() - stateStartTime;
//       switch (state) {
//         case 0: // Avanzar
//           trcLeft       = readTRC5000(TRC5000_LEFT);
//           trcRight      = readTRC5000(TRC5000_RIGHT);
//           if (trcLeft || trcRight) {
//             backward();
//             return;
//           }
//           velocidad = 195;
//           forward();
//           if (elapsed >= forwardDuration) {
//             state = 1; // Cambiar a pausa
//             stateStartTime = millis();
//           }
//           break;
    
//         case 1: // Pausa
//           state = 2; // Cambiar a giro
//           return;
//           stopMotors();
//           if (elapsed >= pauseDuration) {
//             state = 2; // Cambiar a giro
//             stateStartTime = millis();
//           }
//           break;
    
//         case 2: // Girar
//           trcLeft       = readTRC5000(TRC5000_LEFT);
//           trcRight      = readTRC5000(TRC5000_RIGHT);
//           if (trcLeft || trcRight) {
//             backward();
//             return;
//           }
//           velocidad = 255;
//           int turnDirection = (stateStartTime / 1000) % 2; // Alterna entre 0 y 1 cada segundo
//           if (turnDirection == 0) {
//             turnLeft();
//           } else {
//             turnRight();
//           }
//           if (elapsed >= turnDuration) {
//             state = 0; // Volver a avanzar
//             stateStartTime = millis();
//           }
//           break;
//       }
//       return;
//   }
//   else {
//     stopMotors();
//   }

// }


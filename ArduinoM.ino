#include <Servo.h>
#include <Wire.h>

// ======== Pines Motores ========
#define ENA 9
#define ENB 10
#define IN1 3
#define IN2 4
#define IN3 5
#define IN4 6
#define IN5 7
#define IN6 8
#define IN7 12
#define IN8 13

// ======== Pines Servos ========
#define SERVO1_PIN A0
#define SERVO2_PIN A1

// ======== MPU6050 ========
#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0

// ======== Buffers ========
static String buffer = "";
static bool lineaCompleta = false;

// ======== Servos ========
Servo servo1;
Servo servo2;

// ======== Variables de control ========
int x = 0, y = 0;
int botonCuadrado = 0;
int botonX = 0;
int botonCirculo = 0;
int botonTriangulo = 0;

int pos_servo1 = 90, last_pos_servo1 = 90;
int pos_servo2 = 90, last_pos_servo2 = 90;
const int PASO = 5;

// ======== IMU ========
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];

// ======== Temporizadores ========
unsigned long tiempoIMU = 0;
const unsigned long INTERVALO_IMU = 100;

void setup() {
    Serial.begin(9600);  // Comunicación por hardware (pines 0 = RX, 1 = TX)

    // --- Motores ---
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT);
    pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT);
    pinMode(IN8, OUTPUT);

    // --- Servos ---
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo1.write(pos_servo1);
    servo2.write(pos_servo2);

    // --- IMU ---
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Serial.println("=== Robot Control + IMU por Serial ===");
}

void loop() {
    // ---- Lectura de datos del control (por Serial) ----
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            lineaCompleta = true;
            break;
        } else {
            buffer += c;
        }
    }

    if (lineaCompleta) {
        int resultado = sscanf(buffer.c_str(), "%d,%d,%d,%d,%d,%d",
                               &x, &y, &botonCuadrado, &botonX, &botonCirculo, &botonTriangulo);
        buffer = "";
        lineaCompleta = false;

        if (resultado == 6) {
            controlarServos();
            controlarMotores();
        }
    }

    // ---- IMU ----
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoIMU >= INTERVALO_IMU) {
        tiempoIMU = tiempoActual;
        leerIMU();
    }
}

// ================= FUNCIONES =================

// ---- Control de Servos ----
void controlarServos() {
    if (botonTriangulo) pos_servo1 = max(0, pos_servo1 - PASO);
    if (botonX) pos_servo1 = min(180, pos_servo1 + PASO);
    if (botonCirculo) pos_servo2 = max(0, pos_servo2 - PASO);
    if (botonCuadrado) pos_servo2 = min(180, pos_servo2 + PASO);

    if (pos_servo1 != last_pos_servo1) {
        servo1.write(pos_servo1);
        last_pos_servo1 = pos_servo1;
    }

    if (pos_servo2 != last_pos_servo2) {
        servo2.write(pos_servo2);
        last_pos_servo2 = pos_servo2;
    }
}

// ---- Control de Motores ----
void controlarMotores() {
    int minspeed_rc = 50;
    int speed_rc = 255;
    int Rpwm = 0, Lpwm = 0;

    if (y >= 50) {
        Lpwm = map(abs(y), 11, 127, minspeed_rc, speed_rc);
        Rpwm = map(abs(y), 11, 127, minspeed_rc, speed_rc);
        forward(Rpwm, Lpwm);
    } else if (y <= -50) {
        Lpwm = map(abs(y), 11, 127, minspeed_rc, speed_rc);
        Rpwm = map(abs(y), 11, 127, minspeed_rc, speed_rc);
        backward(Rpwm, Lpwm);
    } else if (x >= 50) {
        Lpwm = map(abs(x), 11, 127, minspeed_rc, speed_rc);
        Rpwm = map(abs(x), 11, 127, minspeed_rc, speed_rc);
        right(Rpwm, Lpwm);
    } else if (x <= -50) {
        Lpwm = map(abs(x), 11, 127, minspeed_rc, speed_rc);
        Rpwm = map(abs(x), 11, 127, minspeed_rc, speed_rc);
        left(Rpwm, Lpwm);
    } else {
        stop();
    }
}

// ---- Lectura del MPU6050 ----
void leerIMU() {
    // Leer acelerómetro
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
    Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

    // Leer giroscopio
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();

    Gy[0] = GyX / G_R;
    Gy[1] = GyY / G_R;

    // Filtro complementario
    Angle[0] = 0.98 * (Angle[0] + Gy[0] * 0.010) + 0.02 * Acc[0];
    Angle[1] = 0.98 * (Angle[1] + Gy[1] * 0.010) + 0.02 * Acc[1];

    // Enviar al ESP32
    //Serial.print("IMU:");
    Serial.print(Angle[0], 2);
    Serial.print(",");
    Serial.println(Angle[1], 2);
}

// ================= FUNCIONES DE MOVIMIENTO =================
void forward(int Rpwm, int Lpwm) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
    digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
    analogWrite(ENA, Rpwm);
    analogWrite(ENB, Lpwm);
}

void backward(int Rpwm, int Lpwm) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
    analogWrite(ENA, Rpwm);
    analogWrite(ENB, Lpwm);
}

void right(int Rpwm, int Lpwm) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
    digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
    analogWrite(ENA, Rpwm);
    analogWrite(ENB, Lpwm);
}

void left(int Rpwm, int Lpwm) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
    digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
    analogWrite(ENA, Rpwm);
    analogWrite(ENB, Lpwm);
}

void stop() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
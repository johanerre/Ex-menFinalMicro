#include <PS4Controller.h>

// UART2 pins (ajusta si necesitas otros)
#define RXD2 16
#define TXD2 17

// ---------- Buffers globales (para no perder datos) ----------
String bufUSB = "";     // buffer para Serial (USB)
bool lineReadyUSB = false;

String bufUART2 = "";   // buffer para Serial2 (UART)
bool lineReadyUART2 = false;

// ---------- Variables leídas ----------
int dato1_usb = 0, dato2_usb = 0;
float dato1_uart2 = 0, dato2_uart2 = 0;   // ← FLOAT para IMU REAL

// ---------- Timers para debug ----------
unsigned long lastDebugUsb = 0;
unsigned long lastDebugUart2 = 0;

void setup() {
  Serial.begin(115200);                         // USB → Python HUD
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2 → Arduino IMU
  delay(50);
  Serial.println(F("=== Transmisor PS4 + IMU a Python ==="));
  PS4.begin();
}

// Helper: parse "n1,n2" (puede ser float)
bool parseTwoFloats(const String &s, float &a, float &b) {
  int c = s.indexOf(',');
  if (c < 0) return false;

  a = s.substring(0, c).toFloat();
  b = s.substring(c + 1).toFloat();
  return true;
}

void loop() {

  // ------------------ Lectura UART2 (IMU desde Arduino) ------------------
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (c == '\n') {
      lineReadyUART2 = true;
      break;
    } else if (c != '\r') {
      bufUART2 += c;
      if (bufUART2.length() > 200)
        bufUART2 = bufUART2.substring(bufUART2.length() - 200);
    }
  }

  // Cuando la IMU manda una línea completa
  if (lineReadyUART2) {
    if (parseTwoFloats(bufUART2, dato1_uart2, dato2_uart2)) {

      // =======================================================
      // *** FORMATO EXACTO QUE PYTHON LEE ***
      // =======================================================
      Serial.printf("X = %.2f , Y = %.2f\n", dato1_uart2, dato2_uart2);

    } else {
      Serial.print(F("⚠️ IMU parse FAIL: '"));
      Serial.print(bufUART2);
      Serial.println("'");
    }

    bufUART2 = "";
    lineReadyUART2 = false;
    lastDebugUart2 = millis();
  }

  // ------------------ PROCESO PS4 (NO CAMBIA NADA) ------------------
  if (PS4.isConnected()) {
    int x = PS4.LStickX();  // -127..127
    int y = PS4.LStickY();
    int square = PS4.Square() ? 1 : 0;
    int cross = PS4.Cross() ? 1 : 0;
    int circle = PS4.Circle() ? 1 : 0;
    int triangle = PS4.Triangle() ? 1 : 0;

    // Enviar por UART2 (tal como ya tenías)
    Serial2.printf("%d,%d,%d,%d,%d,%d\n",
                   x, y, square, cross, circle, triangle);

    // Debug en USB
    Serial.printf("➡️ Enviado PS4 -> %d,%d,%d,%d,%d,%d\n",
                  x, y, square, cross, circle, triangle);

    delay(50);
  } else {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      Serial.println(F("PS4 no conectado"));
      lastPrint = millis();
    }
  }

  yield();
}

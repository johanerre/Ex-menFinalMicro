# CÓDIGO ARDUINO
# Explicación del Sistema IMU (MPU6050)

## ¿Qué es el MPU6050?

El MPU6050 es un sensor que combina:
- **Acelerómetro**: Detecta la inclinación usando la gravedad
- **Giroscopio**: Mide la velocidad de rotación

Se comunica con Arduino mediante I2C (2 cables: SDA y SCL).

---

## Definiciones y constantes
```cpp
#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0
```

- **MPU 0x68**: Dirección I2C del sensor (como su número de identificación)
- **A_R 16384.0**: Factor para convertir datos del acelerómetro a "gs" (unidades de gravedad)
- **G_R 131.0**: Factor para convertir datos del giroscopio a grados/segundo

---

## Variables de almacenamiento
```cpp
int16_t AcX, AcY, AcZ;          // Valores crudos del acelerómetro
int16_t GyX, GyY, GyZ;          // Valores crudos del giroscopio
float Acc[2];                    // Ángulos calculados desde acelerómetro
float Gy[2];                     // Velocidades angulares en °/s
float Angle[2];                  // Ángulos finales (combinando ambos sensores)
```

- **AcX, AcY, AcZ**: Aceleración en los 3 ejes (sin procesar)
- **GyX, GyY**: Velocidad de rotación (sin procesar)
- **Acc[0] y Acc[1]**: Pitch y Roll desde el acelerómetro
- **Gy[0] y Gy[1]**: Velocidades de rotación procesadas
- **Angle[0] y Angle[1]**: Ángulos finales precisos (los que realmente usamos)

---

## Sistema de temporización
```cpp
unsigned long tiempoIMU = 0;
const unsigned long INTERVALO_IMU = 100;
```

- Lee el sensor cada 100ms (10 veces por segundo)
- Usa temporización no bloqueante para no detener otras tareas del robot

---

## Inicialización (setup)
```cpp
Wire.begin();                    // Inicia comunicación I2C
Wire.beginTransmission(MPU);     // Selecciona el sensor
Wire.write(0x6B);                // Registro de energía
Wire.write(0);                   // Despierta el sensor
Wire.endTransmission(true);      // Finaliza comunicación
```

**Función**: Despierta el MPU6050 que viene dormido por defecto.

---

## Lectura periódica (loop)
```cpp
unsigned long tiempoActual = millis();
if (tiempoActual - tiempoIMU >= INTERVALO_IMU) {
    tiempoIMU = tiempoActual;
    leerIMU();
}
```

**Función**: Ejecuta `leerIMU()` cada 100ms sin bloquear el programa.

---

## Función leerIMU()

### 1. Lectura del acelerómetro
```cpp
Wire.beginTransmission(MPU);
Wire.write(0x3B);                           // Registro inicial del acelerómetro
Wire.endTransmission(false);
Wire.requestFrom(MPU, 6, true);             // Pide 6 bytes (2 por eje)
AcX = Wire.read() << 8 | Wire.read();       // Combina 2 bytes en un valor de 16 bits
AcY = Wire.read() << 8 | Wire.read();
AcZ = Wire.read() << 8 | Wire.read();
```

**Función**: Obtiene los valores crudos de aceleración en los 3 ejes.

### 2. Cálculo de ángulos desde acelerómetro
```cpp
Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
```

**Función**: 
- Convierte valores crudos a "gs" (dividiendo por A_R)
- Usa trigonometría para calcular el ángulo de inclinación
- La gravedad siempre apunta hacia abajo, con eso se calcula la inclinación
- **Acc[0]**: Pitch (inclinación frontal)
- **Acc[1]**: Roll (inclinación lateral)

### 3. Lectura del giroscopio
```cpp
Wire.beginTransmission(MPU);
Wire.write(0x43);                           // Registro inicial del giroscopio
Wire.endTransmission(false);
Wire.requestFrom(MPU, 4, true);             // Pide 4 bytes (solo 2 ejes)
GyX = Wire.read() << 8 | Wire.read();
GyY = Wire.read() << 8 | Wire.read();

Gy[0] = GyX / G_R;                          // Convierte a °/s
Gy[1] = GyY / G_R;
```

**Función**: Obtiene las velocidades de rotación y las convierte a grados/segundo.

### 4. Filtro complementario
```cpp
Angle[0] = 0.98 * (Angle[0] + Gy[0] * 0.010) + 0.02 * Acc[0];
Angle[1] = 0.98 * (Angle[1] + Gy[1] * 0.010) + 0.02 * Acc[1];
```

**Función**: Combina lo mejor de ambos sensores:

**Problemas individuales:**
- **Acelerómetro**: No se desvía pero es ruidoso (vibra mucho)
- **Giroscopio**: Es suave pero se desvía con el tiempo

**Solución del filtro:**
- **98%** del giroscopio: Da movimiento suave y preciso
- **2%** del acelerómetro: Corrige lentamente la desviación
- **Gy[0] * 0.010**: Multiplica velocidad × tiempo para obtener cambio de ángulo

**Resultado**: Ángulos precisos, suaves y sin desviación.

### 5. Envío de datos
```cpp
Serial.print(Angle[0], 2);      // Envía Pitch con 2 decimales
Serial.print(",");              // Separador
Serial.println(Angle[1], 2);    // Envía Roll con 2 decimales
```

**Función**: Envía los ángulos por serial en formato: `pitch,roll` (ejemplo: `5.12,3.45`)


---

## Resumen de resultados

**Salida del sistema:**
- **Angle[0]**: Pitch (inclinación frontal) - Actualizado cada 100ms
- **Angle[1]**: Roll (inclinación lateral) - Actualizado cada 100ms

**Características:**
- Precisión del giroscopio (suave, sin ruido)
- Estabilidad del acelerómetro (sin desviación)
- Actualización 10 veces por segundo
- No bloquea otras tareas del robot

---


# Explicación del Código ESP32 - Control PS4 + IMU

## Descripción general

Este código funciona en un **ESP32** y tiene dos tareas principales:
1. **Recibir** datos del control PS4 y enviarlos al Arduino por UART2
2. **Recibir** datos del IMU desde el Arduino por UART2 y reenviarlos a Python por USB

Es un "puente" o intermediario entre el PS4, el Arduino y la computadora.

---

## Librerías y pines
```cpp
#include <PS4Controller.h>

#define RXD2 16
#define TXD2 17
```

- **PS4Controller.h**: Librería para conectar un control PS4 por Bluetooth
- **RXD2 (pin 16)**: Recepción de datos UART2 (recibe del Arduino)
- **TXD2 (pin 17)**: Transmisión de datos UART2 (envía al Arduino)

---

## Buffers de comunicación
```cpp
String bufUSB = "";
bool lineReadyUSB = false;

String bufUART2 = "";
bool lineReadyUART2 = false;
```

**Función**: Almacenan temporalmente los datos que llegan carácter por carácter.

- **bufUSB**: Para datos que vengan por USB (actualmente no se usa)
- **bufUART2**: Para datos del Arduino (IMU)
- **lineReadyUSB/UART2**: Banderas que indican cuando hay una línea completa

**¿Por qué buffers?**
- Los datos llegan de a poco, carácter por carácter
- Necesitamos acumularlos hasta tener una línea completa (terminada en '\n')

---

## Variables de datos
```cpp
int dato1_usb = 0, dato2_usb = 0;
float dato1_uart2 = 0, dato2_uart2 = 0;
```

- **dato1_usb, dato2_usb**: Para datos de USB (no usados actualmente)
- **dato1_uart2, dato2_uart2**: Almacenan los ángulos Pitch y Roll del IMU
  - Son **float** porque los ángulos tienen decimales (ej: 5.12°)

---

## Timers de debug
```cpp
unsigned long lastDebugUsb = 0;
unsigned long lastDebugUart2 = 0;
```

**Función**: Registran el último momento en que se recibieron datos (para debugging).

---

## Setup - Inicialización
```cpp
void setup() {
  Serial.begin(115200);                         // USB hacia Python
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2 con Arduino
  delay(50);
  Serial.println(F("=== Transmisor PS4 + IMU a Python ==="));
  PS4.begin();
}
```

### Desglose:

1. **Serial.begin(115200)**:
   - Puerto USB del ESP32 a la computadora
   - Velocidad: 115200 baudios (rápido)
   - Aquí se envían los datos del IMU a Python

2. **Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2)**:
   - Puerto UART2 del ESP32 al Arduino
   - Velocidad: 9600 baudios (igual que el Arduino)
   - **SERIAL_8N1**: 8 bits de datos, sin paridad, 1 bit de parada
   - Usa pines 16 (RX) y 17 (TX)

3. **PS4.begin()**:
   - Inicia el Bluetooth para conectarse al control PS4

---

## Función parseTwoFloats()
```cpp
bool parseTwoFloats(const String &s, float &a, float &b) {
  int c = s.indexOf(',');
  if (c < 0) return false;

  a = s.substring(0, c).toFloat();
  b = s.substring(c + 1).toFloat();
  return true;
}
```

### Función:
Convierte un texto como `"5.12,3.45"` en dos números float.

### Paso a paso:

1. **s.indexOf(',')**: Busca la posición de la coma
   - Si no hay coma, devuelve -1 (error)

2. **s.substring(0, c)**: Extrae todo antes de la coma → `"5.12"`
   - **.toFloat()**: Convierte a número → `5.12`

3. **s.substring(c + 1)**: Extrae todo después de la coma → `"3.45"`
   - **.toFloat()**: Convierte a número → `3.45`

4. **return true**: Indica que la conversión fue exitosa

**Ejemplo**:
```
Entrada: "5.12,3.45"
Salida: a = 5.12, b = 3.45
```

---

## Loop - Parte 1: Lectura del IMU (UART2)
```cpp
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
```

### Función:
Lee carácter por carácter los datos que envía el Arduino.

### Paso a paso:

1. **Serial2.available()**: Verifica si hay datos esperando
2. **Serial2.read()**: Lee un carácter
3. **if (c == '\n')**: Si es fin de línea
   - Marca que hay una línea completa
   - Sale del while
4. **else if (c != '\r')**: Si no es retorno de carro
   - Agrega el carácter al buffer
5. **Protección de overflow**:
   - Si el buffer pasa de 200 caracteres, recorta los más viejos
   - Evita que se llene la memoria

---

## Loop - Parte 2: Procesamiento del IMU
```cpp
if (lineReadyUART2) {
    if (parseTwoFloats(bufUART2, dato1_uart2, dato2_uart2)) {

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
```

### Función:
Cuando hay una línea completa del IMU, la procesa y reenvía a Python.

### Paso a paso:

1. **parseTwoFloats()**: Convierte el texto en dos números
2. **Si es exitoso**:
   - **Serial.printf()**: Envía a Python en formato exacto:
```
     X = 5.12 , Y = 3.45
```
   - **%.2f**: Muestra 2 decimales
3. **Si falla**:
   - Muestra error con el texto recibido
4. **Limpieza**:
   - Limpia el buffer
   - Resetea la bandera
   - Actualiza el timer de debug

**Formato de salida**:
```
X = 5.12 , Y = 3.45
```
Este formato es el que Python espera recibir.

---

## Loop - Parte 3: Control PS4
```cpp
if (PS4.isConnected()) {
    int x = PS4.LStickX();      // -127 a 127
    int y = PS4.LStickY();
    int square = PS4.Square() ? 1 : 0;
    int cross = PS4.Cross() ? 1 : 0;
    int circle = PS4.Circle() ? 1 : 0;
    int triangle = PS4.Triangle() ? 1 : 0;

    Serial2.printf("%d,%d,%d,%d,%d,%d\n",
                   x, y, square, cross, circle, triangle);

    Serial.printf("➡️ Enviado PS4 -> %d,%d,%d,%d,%d,%d\n",
                  x, y, square, cross, circle, triangle);

    delay(50);
}
```

### Función:
Lee el control PS4 y envía los datos al Arduino.

### Paso a paso:

1. **PS4.isConnected()**: Verifica si el control está conectado

2. **Lectura de controles**:
   - **PS4.LStickX()**: Joystick izquierdo horizontal (-127 a 127)
   - **PS4.LStickY()**: Joystick izquierdo vertical (-127 a 127)
   - **Botones**: Devuelven true/false, se convierten a 1/0

3. **Serial2.printf()**: Envía al Arduino por UART2
   - Formato: `x,y,square,cross,circle,triangle\n`
   - Ejemplo: `50,-30,0,1,0,0\n`

4. **Serial.printf()**: Debug en USB
   - Muestra qué se envió (para verificar)

5. **delay(50)**: Espera 50ms entre envíos
   - Envía datos 20 veces por segundo

### Control desconectado:
```cpp
else {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      Serial.println(F("PS4 no conectado"));
      lastPrint = millis();
    }
}
```

- Si el PS4 no está conectado, imprime un mensaje cada 2 segundos
- **static**: La variable mantiene su valor entre llamadas

## Resumen de comunicaciones

### UART2 (ESP32 ↔ Arduino):
- **Velocidad**: 9600 baudios
- **ESP32 → Arduino**: Datos del PS4 (`x,y,sq,cr,ci,tr`)
- **Arduino → ESP32**: Datos del IMU (`pitch,roll`)

### USB (ESP32 ↔ Python):
- **Velocidad**: 115200 baudios
- **ESP32 → Python**: Datos del IMU formateados (`X = pitch , Y = roll`)
- **Python**: Muestra los ángulos en tiempo real

---


# Explicación del Código Python - HUD con IMU y Cámara

## Descripción general

Este programa crea un **HUD (Head-Up Display)** que:
1. Lee datos del IMU desde el ESP32 por puerto serial
2. Muestra video en vivo de una cámara
3. Dibuja un horizonte artificial que se mueve según la inclinación del robot
4. Muestra telemetría en tiempo real (Roll, Pitch, FPS, fecha/hora)

---

## Librerías importadas
```python
import cv2
import numpy as np
from datetime import datetime
import math
import time
import serial
import re
```

- **cv2**: OpenCV para procesar video y dibujar gráficos
- **numpy**: Manejo de arrays para puntos y coordenadas
- **datetime**: Obtener fecha y hora actual
- **math**: Funciones trigonométricas (seno, coseno)
- **time**: Medir tiempo para calcular FPS
- **serial**: Comunicación con el ESP32 por USB
- **re**: Expresiones regulares para extraer números del texto

---

## Configuración inicial
```python
CAM_INDEX = 1
FLIP_HORIZONTAL = True

TEXT_COLOR = (0, 0, 0)
HORIZON_COLOR = (0, 0, 255)
AIRCRAFT_COLOR = (0, 0, 0)
PITCH_PIXELS_PER_DEGREE = 3.0

PORT = "COM4"
BAUD = 115200
```

### Variables:

- **CAM_INDEX = 1**: Índice de la cámara (0 = cámara integrada, 1 = cámara USB)
- **FLIP_HORIZONTAL = True**: Invierte la imagen horizontalmente (efecto espejo)

### Colores (formato BGR de OpenCV):
- **TEXT_COLOR = (0, 0, 0)**: Negro para texto
- **HORIZON_COLOR = (0, 0, 255)**: Rojo para la línea del horizonte
- **AIRCRAFT_COLOR = (0, 0, 0)**: Negro para el símbolo del avión

### Sensibilidad del horizonte:
- **PITCH_PIXELS_PER_DEGREE = 3.0**: Cada grado de pitch mueve el horizonte 3 píxeles

### Puerto serial:
- **PORT = "COM4"**: Puerto donde está conectado el ESP32
- **BAUD = 115200**: Velocidad de comunicación (igual que en el ESP32)

---

## Función connect_serial()
```python
def connect_serial():
    print(f"\n🔌 Conectando al ESP32 en {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        print("✅ ESP32 conectado.\n")
        return ser
    except Exception as e:
        print(f"❌ No se pudo abrir {PORT}: {e}")
        return None
```

### Función:
Establece conexión con el ESP32 por USB.

### Paso a paso:

1. **serial.Serial()**: Abre el puerto serial
   - **PORT**: Puerto COM (Windows) o /dev/ttyUSB0 (Linux)
   - **BAUD**: Velocidad en baudios
   - **timeout=1**: Espera máximo 1 segundo por datos

2. **time.sleep(2)**: Espera 2 segundos
   - Algunos dispositivos necesitan tiempo para iniciar

3. **return ser**: Devuelve el objeto de conexión
   - Si falla, devuelve None

---

## Función read_imu_data()
```python
def read_imu_data(ser):
    if ser is None:
        return None

    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None

            m = re.search(r"X\s*=\s*(-?\d+\.?\d*)\s*[, ]+\s*Y\s*=\s*(-?\d+\.?\d*)", line)
            if m:
                roll = float(m.group(1))
                pitch = float(m.group(2))
                return roll, pitch, 0.0

    except:
        pass

    return None
```

### Función:
Lee y procesa los datos del IMU que envía el ESP32.

### Paso a paso:

1. **ser.in_waiting > 0**: Verifica si hay datos disponibles

2. **ser.readline()**: Lee una línea completa hasta '\n'
   - Ejemplo: `"X = 5.12 , Y = 3.45\n"`

3. **decode('utf-8')**: Convierte bytes a texto
   - **errors='ignore'**: Ignora caracteres raros

4. **strip()**: Elimina espacios y saltos de línea

5. **Expresión regular**:
```python
   r"X\s*=\s*(-?\d+\.?\d*)\s*[, ]+\s*Y\s*=\s*(-?\d+\.?\d*)"
```

6. **m.group(1) y m.group(2)**: Extrae los números encontrados
   - group(1) = valor de X (roll)
   - group(2) = valor de Y (pitch)

7. **return roll, pitch, 0.0**: Devuelve los tres ángulos
   - yaw = 0.0 porque no se usa

**Ejemplo de entrada/salida**:
```
Entrada: "X = 5.12 , Y = 3.45"
Salida: (5.12, 3.45, 0.0)
```

---

## Función draw_horizon()
```python
def draw_horizon(img, center, roll_deg, pitch_deg, line_color, thickness=2):
    h, w = img.shape[:2]
    cx, cy = center

    dy = int(-pitch_deg * PITCH_PIXELS_PER_DEGREE)
    cy_shift = cy + dy

    L = int(max(w, h) * 1.5)
    theta = math.radians(roll_deg)

    def rot(x, y, cx, cy, ang):
        xr = cx + math.cos(ang) * (x - cx) - math.sin(ang) * (y - cy)
        yr = cy + math.sin(ang) * (x - cx) + math.cos(ang) * (y - cy)
        return int(xr), int(yr)

    x1, y1 = cx - L, cy_shift
    x2, y2 = cx + L, cy_shift

    p1 = rot(x1, y1, cx, cy_shift, theta)
    p2 = rot(x2, y2, cx, cy_shift, theta)

    cv2.line(img, p1, p2, line_color, thickness, cv2.LINE_AA)
```

### Función:
Dibuja la línea del horizonte artificial que responde al roll y pitch.

### Paso a paso:

1. **Cálculo del desplazamiento vertical (pitch)**:
```python
   dy = int(-pitch_deg * PITCH_PIXELS_PER_DEGREE)
   cy_shift = cy + dy
```
   - Si pitch = 10°, dy = -30 píxeles (sube)
   - Si pitch = -10°, dy = 30 píxeles (baja)
   - **cy_shift**: Nueva posición Y del horizonte

2. **Longitud de la línea**:
```python
   L = int(max(w, h) * 1.5)
```
   - Línea más larga que la pantalla (para que siempre se vea al rotar)

3. **Conversión de roll a radianes**:
```python
   theta = math.radians(roll_deg)
```

4. **Función de rotación interna rot()**:
   - Aplica transformación de rotación 2D
   - Usa matriz de rotación:
```
     x' = cx + cos(θ)·(x-cx) - sin(θ)·(y-cy)
     y' = cy + sin(θ)·(x-cx) + cos(θ)·(y-cy)
```

5. **Calcular puntos extremos**:
```python
   x1, y1 = cx - L, cy_shift  # Punto izquierdo
   x2, y2 = cx + L, cy_shift  # Punto derecho
```

6. **Rotar los puntos**:
```python
   p1 = rot(x1, y1, cx, cy_shift, theta)
   p2 = rot(x2, y2, cx, cy_shift, theta)
```

7. **Dibujar la línea**:
```python
   cv2.line(img, p1, p2, line_color, thickness, cv2.LINE_AA)
```
   - **LINE_AA**: Anti-aliasing para línea suave

**Resultado**:
- Horizonte que sube/baja con pitch
- Horizonte que rota con roll

---

## Función draw_aircraft_symbol()
```python
def draw_aircraft_symbol(img, center, size=20, color=(255,255,255), thickness=2):
    cx, cy = center
    pts = np.array([
        [cx, cy - size],
        [cx - size, cy + size // 2],
        [cx + size, cy + size // 2]
    ], np.int32)

    cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA)
    cv2.line(img, (cx - size // 2, cy), (cx + size // 2, cy), color, thickness, cv2.LINE_AA)
```

### Función:
Dibuja el símbolo del avión en el centro del HUD.

### Paso a paso:

1. **Definir puntos del triángulo**:
```python
   pts = np.array([
       [cx, cy - size],              # Punta superior
       [cx - size, cy + size // 2],  # Esquina izquierda
       [cx + size, cy + size // 2]   # Esquina derecha
   ])
```

2. **cv2.polylines()**: Dibuja el triángulo
   - **True**: Cierra la figura
   - **LINE_AA**: Suavizado

3. **cv2.line()**: Dibuja la línea horizontal (alas)
   - Desde la mitad izquierda hasta la mitad derecha


## Función main()
```python
def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("❌ No se pudo abrir la cámara.")
        return

    ser = connect_serial()

    roll = pitch = yaw = 0.0
    fps_timer = time.time()

    while True:
        # ... código del loop ...
```

### Inicialización:

1. **cv2.VideoCapture(CAM_INDEX)**: Abre la cámara
2. **connect_serial()**: Conecta con el ESP32
3. **Variables iniciales**: roll, pitch, yaw = 0
4. **fps_timer**: Para calcular FPS

---

## Loop principal - Parte 1: Captura y procesamiento
```python
ret, frame = cap.read()
if not ret:
    break

if FLIP_HORIZONTAL:
    frame = cv2.flip(frame, 1)

h, w = frame.shape[:2]
cx, cy = w // 2, h // 2
```

1. **cap.read()**: Captura un frame de la cámara
   - **ret**: True si fue exitoso
   - **frame**: Imagen capturada

2. **cv2.flip(frame, 1)**: Voltea horizontalmente (si está activado)

3. **Calcular centro**: cx, cy = mitad de ancho y alto

---

## Loop principal - Parte 2: Leer IMU y dibujar
```python
imu = read_imu_data(ser)
if imu:
    roll, pitch, yaw = imu

draw_horizon(frame, (cx, cy), roll, pitch, HORIZON_COLOR)
draw_aircraft_symbol(frame, (cx, cy + 80), size=18, color=AIRCRAFT_COLOR)
```

1. **read_imu_data()**: Obtiene datos del IMU
2. **draw_horizon()**: Dibuja el horizonte artificial
3. **draw_aircraft_symbol()**: Dibuja el avión (80 píxeles abajo del centro)

---

## Loop principal - Parte 3: Telemetría
```python
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
cv2.putText(frame, timestamp, (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1, cv2.LINE_AA)

cv2.putText(frame, f"Roll:  {roll:6.2f}",  (10, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)
cv2.putText(frame, f"Pitch: {pitch:6.2f}", (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

now = time.time()
fps = 1 / (now - fps_timer)
fps_timer = now

cv2.putText(frame, f"FPS: {fps:4.1f}", (w - 120, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)
```

### Información mostrada:

1. **Fecha y hora**: Arriba izquierda
2. **Roll y Pitch**: Valores con 2 decimales
3. **FPS**: Arriba derecha
   - Se calcula: 1 / tiempo_entre_frames

---

## Loop principal - Parte 4: Mostrar y salir
```python
cv2.imshow("HUD IMU + Camera", frame)

if cv2.waitKey(1) & 0xFF == ord("q"):
    break

cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()
```

1. **cv2.imshow()**: Muestra la ventana con el HUD
2. **cv2.waitKey(1)**: Espera 1ms por tecla
   - Si presionas 'q', sale del loop
3. **Limpieza**:
   - Cierra la cámara
   - Cierra el puerto serial
   - Destruye ventanas

---


---

## Resumen de funcionalidades

### Entrada:
- Datos del IMU vía serial: `"X = 5.12 , Y = 3.45"`
- Video en vivo de cámara USB

### Procesamiento:
- Extrae roll y pitch con expresiones regulares
- Calcula posición del horizonte según ángulos
- Aplica rotación 2D para el roll

### Salida visual:
- Horizonte artificial que responde a inclinación
- Símbolo de avión fijo en el centro
- Fecha y hora
- Valores de Roll y Pitch
- FPS en tiempo real

---

# CONFIGURACIÓN MAESTRO-ESCLAVO
---

## Conexiones físicas

### Cables UART (ESP32 ↔ Arduino):
```
ESP32 pin 17 (TX) ──────► Arduino pin 0 (RX)
ESP32 pin 16 (RX) ◄────── Arduino pin 1 (TX)
ESP32 GND        ──────── Arduino GND (CRÍTICO)
```

**IMPORTANTE**:
- TX de uno va a RX del otro
- GND común es OBLIGATORIO
- NO conectar VCC entre ellos (diferente voltaje)

---

## Tabla de velocidades

| Conexión | Puerto | Velocidad | Razón |
|----------|--------|-----------|-------|
| ESP32 ↔ Python | USB (Serial) | 115200 | Rápida para HUD fluido |
| ESP32 ↔ Arduino | UART2 ↔ Serial | 9600 | Estándar, confiable |
| Arduino ↔ MPU6050 | I2C (Wire) | 400kHz | Protocolo I2C estándar |

**Sincronización crítica**: Las velocidades DEBEN coincidir en ambos extremos de cada conexión.

---

## Formatos de datos en cada etapa

### Control PS4 → ESP32:
```
Formato: Señales Bluetooth
Contenido: Estados de joysticks y botones
```

### ESP32 → Arduino:
```
Formato: "x,y,sq,cr,ci,tr\n"
Ejemplo: "50,-30,0,1,0,0\n"
Velocidad: 20 Hz (cada 50ms)
```

### Arduino → ESP32:
```
Formato: "pitch,roll\n"
Ejemplo: "5.12,3.45\n"
Velocidad: 10 Hz (cada 100ms)
```

### ESP32 → Python:
```
Formato: "X = pitch , Y = roll\n"
Ejemplo: "X = 5.12 , Y = 3.45\n"
Velocidad: 10 Hz (según lleguen del Arduino)
```

---

## Ventajas de esta arquitectura

### 1. Separación de responsabilidades:
- **Arduino**: Hardware de bajo nivel (motores, IMU, I2C)
- **ESP32**: Comunicaciones (Bluetooth, conversión de formatos)
- **Python**: Interfaz visual (procesamiento de video, HUD)

### 2. Optimización de velocidades:
- Comunicación lenta pero confiable entre micros (9600)
- Comunicación rápida para video (115200)

### 3. Escalabilidad:
- Fácil agregar sensores al Arduino
- Fácil mejorar HUD sin tocar firmware

### 4. Modularidad:
- Cada dispositivo se programa independientemente
- Fácil debugging por secciones

---

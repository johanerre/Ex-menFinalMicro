# Configuración Esclavo-Triángulo: ESP32 ↔ Arduino

## Descripción del sistema

Este sistema implementa una comunicación **bidireccional** entre ESP32 y Arduino usando **UART (Serial)**:

- **ESP32 → Arduino**: Envía comandos del control PS4
- **Arduino → ESP32**: Envía datos del IMU
- **ESP32 → Python**: Reenvía datos del IMU para visualización

Es una arquitectura en **triángulo** donde el ESP32 actúa como puente entre el PS4, el Arduino y la computadora.

---

## Configuración del ESP32 (Maestro/Puente)

### Código de configuración:
```cpp
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);                         // USB a Python
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2 con Arduino
  delay(50);
  PS4.begin();
}
```

### Explicación detallada:

#### 1. Definición de pines UART2:
```cpp
#define RXD2 16  // Pin de recepción
#define TXD2 17  // Pin de transmisión
```

- **RXD2 (pin 16)**: Recibe datos DEL Arduino
- **TXD2 (pin 17)**: Envía datos AL Arduino
- El ESP32 tiene **3 puertos seriales**:
  - Serial (USB): Comunicación con la computadora
  - Serial1: Disponible pero no usado aquí
  - **Serial2**: Comunicación con el Arduino

#### 2. Inicialización del puerto USB:
```cpp
Serial.begin(115200);
```

- **Serial**: Puerto USB del ESP32
- **115200 baudios**: Velocidad rápida para enviar datos a Python
- **Dirección**: ESP32 → Computadora (solo transmite, no recibe)

#### 3. Inicialización del UART2:
```cpp
Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
```

Desglose de parámetros:

- **9600**: Velocidad en baudios (debe coincidir con Arduino)
- **SERIAL_8N1**: Configuración de bits
  - **8**: 8 bits de datos
  - **N**: Sin paridad (No parity)
  - **1**: 1 bit de parada
- **RXD2, TXD2**: Pines físicos a usar (16 y 17)

**Nota importante**: El ESP32 permite configurar pines custom para UART, el Arduino NO.

---

## Configuración del Arduino (Esclavo)

### Código de configuración:
```cpp
void setup() {
    Serial.begin(9600);  // Puerto hardware (RX=0, TX=1)
    
    // ... configuración de motores, servos, IMU ...
}
```

### Explicación detallada:

#### 1. Inicialización del puerto serial:
```cpp
Serial.begin(9600);
```

- **Serial**: Puerto hardware del Arduino (ÚNICO puerto serial nativo)
- **Pines fijos**:
  - **Pin 0 (RX)**: Recibe datos DEL ESP32
  - **Pin 1 (TX)**: Envía datos AL ESP32
- **9600 baudios**: Velocidad estándar (debe coincidir con ESP32)
- **No se pueden cambiar los pines**: El Arduino UNO solo tiene estos dos pines para serial hardware

#### 2. Restricción importante:
```cpp
// Serial.println("Debug");  // ❌ NO hacer durante operación
```

- El mismo puerto se usa para **programar** el Arduino
- Durante la operación, NO usar `Serial.print()` para debug porque:
  - Interfiere con los datos del IMU
  - Confunde al ESP32
- **Solución**: Comentar todos los prints de debug antes de uso final

---

## Flujo de comunicación bidireccional

### ESP32 → Arduino (Comandos del PS4)

#### En el ESP32:
```cpp
void loop() {
  if (PS4.isConnected()) {
    int x = PS4.LStickX();
    int y = PS4.LStickY();
    int square = PS4.Square() ? 1 : 0;
    int cross = PS4.Cross() ? 1 : 0;
    int circle = PS4.Circle() ? 1 : 0;
    int triangle = PS4.Triangle() ? 1 : 0;

    // Envía por UART2 al Arduino
    Serial2.printf("%d,%d,%d,%d,%d,%d\n",
                   x, y, square, cross, circle, triangle);
    
    delay(50);
  }
}
```

**Formato enviado**: `x,y,square,cross,circle,triangle\n`
**Ejemplo**: `50,-30,0,1,0,0\n`

#### En el Arduino:
```cpp
void loop() {
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
                               &x, &y, &botonCuadrado, &botonX, 
                               &botonCirculo, &botonTriangulo);
        buffer = "";
        lineaCompleta = false;

        if (resultado == 6) {
            controlarServos();
            controlarMotores();
        }
    }
}
```

**Proceso**:
1. Lee carácter por carácter del puerto serial
2. Acumula en buffer hasta encontrar `\n`
3. Usa `sscanf()` para extraer los 6 valores
4. Controla motores y servos según los valores

---

### Arduino → ESP32 (Datos del IMU)

#### En el Arduino:
```cpp
void leerIMU() {
    // ... lectura y procesamiento del MPU6050 ...
    
    // Envía por Serial al ESP32
    Serial.print(Angle[0], 2);  // Pitch
    Serial.print(",");
    Serial.println(Angle[1], 2); // Roll
}
```

**Formato enviado**: `pitch,roll\n`
**Ejemplo**: `5.12,3.45\n`
**Frecuencia**: Cada 100ms (10 veces por segundo)

#### En el ESP32:
```cpp
void loop() {
    // Lee desde Arduino por UART2
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        
        if (c == '\n') {
            lineReadyUART2 = true;
            break;
        } else if (c != '\r') {
            bufUART2 += c;
        }
    }

    if (lineReadyUART2) {
        if (parseTwoFloats(bufUART2, dato1_uart2, dato2_uart2)) {
            // Reenvía a Python por USB
            Serial.printf("X = %.2f , Y = %.2f\n", 
                         dato1_uart2, dato2_uart2);
        }
        
        bufUART2 = "";
        lineReadyUART2 = false;
    }
}
```

**Proceso**:
1. Lee del UART2 (Arduino) carácter por carácter
2. Acumula en buffer hasta encontrar `\n`
3. Convierte los valores con `parseTwoFloats()`
4. **Reformatea** y reenvía por USB a Python

---

### ESP32 → Python (Telemetría para HUD)

#### En el ESP32:
```cpp
Serial.printf("X = %.2f , Y = %.2f\n", dato1_uart2, dato2_uart2);
```

**Formato enviado**: `X = pitch , Y = roll\n`
**Ejemplo**: `X = 5.12 , Y = 3.45\n`

#### En Python:
```python
def read_imu_data(ser):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        
        m = re.search(r"X\s*=\s*(-?\d+\.?\d*)\s*[, ]+\s*Y\s*=\s*(-?\d+\.?\d*)", line)
        if m:
            roll = float(m.group(1))
            pitch = float(m.group(2))
            return roll, pitch, 0.0
    
    return None
```

**Proceso**:
1. Lee línea completa del puerto serial USB
2. Usa expresión regular para extraer los números
3. Convierte a float y retorna

---

## Conexiones físicas (Hardware)

### Cables necesarios:
```
ESP32 pin 17 (TX) ──────► Arduino pin 0 (RX)
ESP32 pin 16 (RX) ◄────── Arduino pin 1 (TX)
ESP32 GND        ──────── Arduino GND (IMPORTANTE)
```

### Reglas de conexión:

1. **TX → RX**: Transmisión de uno va a recepción del otro
2. **RX → TX**: Recepción de uno viene de transmisión del otro
3. **GND común**: SIEMPRE conectar las tierras
4. **Niveles de voltaje**:
   - ESP32: 3.3V lógico
   - Arduino: 5V lógico
   - **Funciona** porque el Arduino acepta 3.3V como HIGH
   - **Cuidado**: NO conectar VCC (alimentación) entre ellos

---


## Sincronización y timing

### Arduino (Envío IMU cada 100ms):
```cpp
unsigned long tiempoIMU = 0;
const unsigned long INTERVALO_IMU = 100;

void loop() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoIMU >= INTERVALO_IMU) {
        tiempoIMU = tiempoActual;
        leerIMU();  // Envía datos por Serial
    }
}
```

**Frecuencia**: 10 Hz (10 veces por segundo)

### ESP32 (Envío PS4 cada 50ms):
```cpp
void loop() {
    if (PS4.isConnected()) {
        // ... leer PS4 ...
        Serial2.printf(...);  // Envía al Arduino
        delay(50);
    }
}
```

**Frecuencia**: 20 Hz (20 veces por segundo)

### Python (Lee tan rápido como pueda):
```python
while True:
    ret, frame = cap.read()  # ~30 FPS de cámara
    imu = read_imu_data(ser)  # Lee si hay datos disponibles
    # ... procesar y mostrar ...
```

**Frecuencia**: ~30 Hz (limitado por la cámara)

---

## Gestión de buffers

### ¿Por qué usar buffers?

Los datos llegan **carácter por carácter**, no líneas completas:
```
Tiempo 0ms:   '5'
Tiempo 1ms:   '.'
Tiempo 2ms:   '1'
Tiempo 3ms:   '2'
Tiempo 4ms:   ','
Tiempo 5ms:   '3'
Tiempo 6ms:   '.'
Tiempo 7ms:   '4'
Tiempo 8ms:   '5'
Tiempo 9ms:   '\n'  ← Ahora tenemos línea completa
```

### En el Arduino:
```cpp
static String buffer = "";
static bool lineaCompleta = false;

while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
        lineaCompleta = true;
        break;
    } else {
        buffer += c;
    }
}
```

### En el ESP32:
```cpp
String bufUART2 = "";
bool lineReadyUART2 = false;

while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
        lineReadyUART2 = true;
        break;
    } else if (c != '\r') {
        bufUART2 += c;
    }
}
```

**Protección contra desbordamiento**:
```cpp
if (bufUART2.length() > 200)
    bufUART2 = bufUART2.substring(bufUART2.length() - 200);
```

---

## Formatos de datos en cada etapa

### 1. Control PS4 → ESP32:
```
Formato: Señales Bluetooth
Contenido: Estados de joysticks y botones
```

### 2. ESP32 → Arduino:
```
Formato: "x,y,sq,cr,ci,tr\n"
Ejemplo: "50,-30,0,1,0,0\n"
Protocolo: UART 9600 baudios
```

### 3. Arduino procesa y lee IMU:
```
Interno: Cálculos del filtro complementario
Resultado: Angle[0] y Angle[1] (float)
```

### 4. Arduino → ESP32:
```
Formato: "pitch,roll\n"
Ejemplo: "5.12,3.45\n"
Protocolo: UART 9600 baudios
```

### 5. ESP32 → Python:
```
Formato: "X = pitch , Y = roll\n"
Ejemplo: "X = 5.12 , Y = 3.45\n"
Protocolo: USB 115200 baudios
```

### 6. Python procesa:
```
Extracción: Expresiones regulares
Uso: Dibujo del horizonte artificial
```


---

## Resumen de la configuración

### Arduino (Esclavo):
- **Puerto**: Serial hardware (pins 0, 1)
- **Velocidad**: 9600 baudios
- **Recibe**: Comandos PS4 del ESP32
- **Envía**: Datos IMU al ESP32
- **Frecuencia IMU**: 10 Hz

### ESP32 (Puente):
- **Puerto 1**: Serial2 (pins 16, 17) ↔ Arduino
- **Puerto 2**: USB (Serial) ↔ Python
- **Velocidad 1**: 9600 baudios (Arduino)
- **Velocidad 2**: 115200 baudios (Python)
- **Función**: Intermediario bidireccional

### Python (Visualizador):
- **Puerto**: USB (COM4 en Windows)
- **Velocidad**: 115200 baudios
- **Recibe**: Datos IMU del ESP32
- **Función**: HUD y telemetría

---


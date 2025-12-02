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

## Flujo completo del sistema
```
Inicialización:
  └─> Despierta el MPU6050

Loop continuo:
  └─> ¿100ms? → SI
      └─> Lee acelerómetro → Calcula ángulos estáticos
      └─> Lee giroscopio → Calcula velocidades
      └─> Aplica filtro (98% giro + 2% acel)
      └─> Envía ángulos por serial
      └─> Regresa al loop
```

---

## Resumen de resultados

**Salida del sistema:**
- **Angle[0]**: Pitch (inclinación frontal) - Actualizado cada 100ms
- **Angle[1]**: Roll (inclinación lateral) - Actualizado cada 100ms

**Características:**
- ✅ Precisión del giroscopio (suave, sin ruido)
- ✅ Estabilidad del acelerómetro (sin desviación)
- ✅ Actualización 10 veces por segundo
- ✅ No bloquea otras tareas del robot

**Aplicaciones:**
- Detectar inclinación del robot
- Control de estabilidad
- Detectar caídas
- Saber si sube/baja rampas

---

## Conceptos clave

- **Pitch**: Ángulo de inclinación hacia adelante/atrás
- **Roll**: Ángulo de inclinación hacia los lados
- **Filtro complementario**: Combina giroscopio (preciso) + acelerómetro (estable)
- **I2C**: Protocolo de comunicación de 2 cables
- **°/s**: Grados por segundo (velocidad de rotación)
- **g**: Unidad de gravedad (1g = 9.8 m/s²)
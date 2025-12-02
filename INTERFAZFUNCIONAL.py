import cv2
import numpy as np
from datetime import datetime
import math
import time
import serial
import re

# ---------------- CONFIGURACIÓN ----------------
CAM_INDEX = 1
FLIP_HORIZONTAL = True

TEXT_COLOR = (0, 0, 0)
HORIZON_COLOR = (0, 0, 255)
AIRCRAFT_COLOR = (0, 0, 0)
PITCH_PIXELS_PER_DEGREE = 3.0

PORT = "COM4"
BAUD = 115200


# ----------- Conectar Serial -----------
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


# ----------- Leer IMU del Serial -----------
def read_imu_data(ser):
    """
    Lee líneas como:
    "X = -34.55 , Y = 66.1"
    Devuelve roll, pitch
    """

    if ser is None:
        return None

    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None

            # Busca X e Y
            m = re.search(r"X\s*=\s*(-?\d+\.?\d*)\s*[, ]+\s*Y\s*=\s*(-?\d+\.?\d*)", line)
            if m:
                roll = float(m.group(1))
                pitch = float(m.group(2))
                return roll, pitch, 0.0  # yaw = 0 porque no lo envías

    except:
        pass

    return None


# ----------- Dibujar Horizonte -----------
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


# ----------- Simbolo del avión -----------
def draw_aircraft_symbol(img, center, size=20, color=(255,255,255), thickness=2):
    cx, cy = center
    pts = np.array([
        [cx, cy - size],
        [cx - size, cy + size // 2],
        [cx + size, cy + size // 2]
    ], np.int32)

    cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA)
    cv2.line(img, (cx - size // 2, cy), (cx + size // 2, cy), color, thickness, cv2.LINE_AA)


# ---------------- MAIN ----------------
def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("❌ No se pudo abrir la cámara.")
        return

    ser = connect_serial()

    roll = pitch = yaw = 0.0
    fps_timer = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if FLIP_HORIZONTAL:
            frame = cv2.flip(frame, 1)

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        # Leer valores de IMU
        imu = read_imu_data(ser)
        if imu:
            roll, pitch, yaw = imu

        # Dibujar horizonte
        draw_horizon(frame, (cx, cy), roll, pitch, HORIZON_COLOR)

        # Dibujar HUD avión
        draw_aircraft_symbol(frame, (cx, cy + 80), size=18, color=AIRCRAFT_COLOR)

        # Hora y fecha
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1, cv2.LINE_AA)

        # Telemetría
        cv2.putText(frame, f"Roll:  {roll:6.2f}",  (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)
        cv2.putText(frame, f"Pitch: {pitch:6.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

        # FPS
        now = time.time()
        fps = 1 / (now - fps_timer)
        fps_timer = now

        cv2.putText(frame, f"FPS: {fps:4.1f}", (w - 120, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1)

        # Mostrar
        cv2.imshow("HUD IMU + Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    if ser:
        ser.close()
    cv2.destroyAllWindows()


# Ejecutar
if __name__ == "__main__":
    main()
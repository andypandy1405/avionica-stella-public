from machine import I2C, Pin, UART
import time
import sys
import uselect

# =========================
# PINES (AJUSTADOS)
# =========================
I2C_SDA = 17
I2C_SCL = 16

GPS_RX = 11  # ESP32 RX  <- GPS TX
GPS_TX = 10  # ESP32 TX  -> GPS RX
GPS_BAUD = 9600

# =========================
# CONFIG
# =========================
SEA_LEVEL_PA = 101325
G0 = 9.80665

IMU_LOOP_MS = 10        # ~100 Hz interno
PRINT_PERIOD_MS = 200   # 5 Hz telemetría (más estable)
ALT_ALPHA = 0.15
VBARO_ALPHA = 0.20
W_IMU = 0.90

# =========================
# BMP180
# =========================
BMP180_ADDR = 0x77
REG_CALIB   = 0xAA
REG_CONTROL = 0xF4
REG_DATA    = 0xF6
CMD_TEMP     = 0x2E
CMD_PRESSURE = 0x34

class BMP180:
    def __init__(self, i2c, addr=BMP180_ADDR, oss=0):
        self.i2c = i2c
        self.addr = addr
        self.oss = oss
        self._read_calibration()

    def _read_u16(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (b[0] << 8) | b[1]

    def _read_s16(self, reg):
        v = self._read_u16(reg)
        if v & 0x8000:
            v = -((v ^ 0xFFFF) + 1)
        return v

    def _read_calibration(self):
        self.AC1 = self._read_s16(REG_CALIB + 0)
        self.AC2 = self._read_s16(REG_CALIB + 2)
        self.AC3 = self._read_s16(REG_CALIB + 4)
        self.AC4 = self._read_u16(REG_CALIB + 6)
        self.AC5 = self._read_u16(REG_CALIB + 8)
        self.AC6 = self._read_u16(REG_CALIB + 10)
        self.B1  = self._read_s16(REG_CALIB + 12)
        self.B2  = self._read_s16(REG_CALIB + 14)
        self.MB  = self._read_s16(REG_CALIB + 16)
        self.MC  = self._read_s16(REG_CALIB + 18)
        self.MD  = self._read_s16(REG_CALIB + 20)

    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _read_ut(self):
        self._write8(REG_CONTROL, CMD_TEMP)
        time.sleep_ms(5)
        return self._read_u16(REG_DATA)

    def _read_up(self):
        self._write8(REG_CONTROL, CMD_PRESSURE + (self.oss << 6))
        delay = [5, 8, 14, 26][self.oss]
        time.sleep_ms(delay)
        b = self.i2c.readfrom_mem(self.addr, REG_DATA, 3)
        up = ((b[0] << 16) | (b[1] << 8) | b[2]) >> (8 - self.oss)
        return up

    def pressure_pa(self):
        ut = self._read_ut()
        up = self._read_up()

        x1 = ((ut - self.AC6) * self.AC5) >> 15
        x2 = (self.MC << 11) // (x1 + self.MD)
        b5 = x1 + x2

        b6 = b5 - 4000
        x1 = (self.B2 * ((b6 * b6) >> 12)) >> 11
        x2 = (self.AC2 * b6) >> 11
        x3 = x1 + x2
        b3 = (((self.AC1 * 4 + x3) << self.oss) + 2) >> 2

        x1 = (self.AC3 * b6) >> 13
        x2 = (self.B1 * ((b6 * b6) >> 12)) >> 16
        x3 = ((x1 + x2) + 2) >> 2
        b4 = (self.AC4 * (x3 + 32768)) >> 15
        b7 = (up - b3) * (50000 >> self.oss)

        if b7 < 0x80000000:
            p = (b7 * 2) // b4
        else:
            p = (b7 // b4) * 2

        x1 = (p >> 8) * (p >> 8)
        x1 = (x1 * 3038) >> 16
        x2 = (-7357 * p) >> 16
        p = p + ((x1 + x2 + 3791) >> 4)
        return int(p)

    def altitude_m(self, sea_level_pa=SEA_LEVEL_PA):
        p = self.pressure_pa()
        return 44330.0 * (1.0 - (p / sea_level_pa) ** 0.1903)

def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0

# =========================
# MPU6050
# =========================
MPU6050_ADDRS = (0x68, 0x69)
MPU_PWR_MGMT_1   = 0x6B
MPU_GYRO_CONFIG  = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_XOUT_H = 0x3B

def _twos16(msb, lsb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v = -((v ^ 0xFFFF) + 1)
    return v

class MPU6050:
    def __init__(self, i2c, addr=0x68, gyro_fs=3, accel_fs=3):
        # gyro_fs=3 => ±2000 dps; accel_fs=3 => ±16 g
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, MPU_PWR_MGMT_1, b"\x00")
        time.sleep_ms(50)
        self.i2c.writeto_mem(self.addr, MPU_GYRO_CONFIG, bytes([(gyro_fs & 0x03) << 3]))
        self.i2c.writeto_mem(self.addr, MPU_ACCEL_CONFIG, bytes([(accel_fs & 0x03) << 3]))
        time.sleep_ms(10)
        self._gyro_scale  = [131.0, 65.5, 32.8, 16.4][gyro_fs & 0x03]
        self._accel_scale = [16384.0, 8192.0, 4096.0, 2048.0][accel_fs & 0x03]

    def read(self):
        b = self.i2c.readfrom_mem(self.addr, MPU_ACCEL_XOUT_H, 14)
        ax = _twos16(b[0],  b[1])
        ay = _twos16(b[2],  b[3])
        az = _twos16(b[4],  b[5])
        gx = _twos16(b[8],  b[9])
        gy = _twos16(b[10], b[11])
        gz = _twos16(b[12], b[13])

        ax_g = ax / self._accel_scale
        ay_g = ay / self._accel_scale
        az_g = az / self._accel_scale

        gx_dps = gx / self._gyro_scale
        gy_dps = gy / self._gyro_scale
        gz_dps = gz / self._gyro_scale

        return ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps

# =========================
# GPS (NEO-6M) NMEA mínimo
# =========================
def _nmea_degmin_to_deg(dm, hemi):
    if not dm:
        return None
    try:
        v = float(dm)
    except:
        return None
    deg = int(v // 100)
    minutes = v - deg * 100
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec

def _knots_to_mps(knots_str):
    try:
        k = float(knots_str)
    except:
        return None
    return k * 0.514444

class GPS:
    def __init__(self, uart):
        self.uart = uart
        self.buf = b""
        self.fix = 0
        self.sats = 0
        self.lat = None
        self.lon = None
        self.alt_m = None
        self.spd_mps = None

    def update(self):
        data = self.uart.read()
        if data:
            self.buf += data
            if len(self.buf) > 4096:
                self.buf = self.buf[-2048:]

        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"$"):
                continue
            s = line.decode("ascii", "ignore")
            self._parse_line(s)

    def _parse_line(self, s):
        if "*" in s:
            s = s.split("*", 1)[0]
        parts = s.split(",")
        if not parts:
            return
        msg = parts[0]

        if msg.endswith("RMC") and len(parts) >= 8:
            status = parts[2]
            lat = _nmea_degmin_to_deg(parts[3], parts[4])
            lon = _nmea_degmin_to_deg(parts[5], parts[6])
            spd = _knots_to_mps(parts[7])
            if status == "A":
                if lat is not None and lon is not None:
                    self.lat, self.lon = lat, lon
                if spd is not None:
                    self.spd_mps = spd

        if msg.endswith("GGA") and len(parts) >= 10:
            try:
                self.fix = int(parts[6]) if parts[6] else 0
            except:
                self.fix = 0
            try:
                self.sats = int(parts[7]) if parts[7] else 0
            except:
                self.sats = 0
            try:
                self.alt_m = float(parts[9]) if parts[9] else None
            except:
                self.alt_m = None

# =========================
# INIT BUSES + SENSORES
# =========================
i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
uart = UART(1, baudrate=GPS_BAUD, rx=Pin(GPS_RX), tx=Pin(GPS_TX))
gps = GPS(uart)

print("\n=== SISTEMA LISTO (GY-87 + GPS) ===")
time.sleep_ms(200)

devices = i2c.scan()
print("I2C detectados:", [hex(d) for d in devices])

if BMP180_ADDR not in devices:
    print("ERROR: BMP180 (0x77) no detectado.")
    while True:
        time.sleep(1)

mpu_addr = None
for a in MPU6050_ADDRS:
    if a in devices:
        mpu_addr = a
        break
if mpu_addr is None:
    print("ERROR: MPU6050 (0x68/0x69) no detectado.")
    while True:
        time.sleep(1)

bmp = BMP180(i2c, addr=BMP180_ADDR, oss=0)
mpu = MPU6050(i2c, addr=mpu_addr, gyro_fs=3, accel_fs=3)

# =========================
# ESTADO GLOBAL (calibración + velocidades)
# =========================
alt0 = 0.0
az0_g = 0.0

alt_rel_f = None
alt_rel_prev = None
v_baro_f = 0.0
v_imu = 0.0
v_fused = 0.0

t_prev_us = time.ticks_us()

def reset_states():
    global alt_rel_f, alt_rel_prev, v_baro_f, v_imu, v_fused, t_prev_us
    alt_rel_f = None
    alt_rel_prev = None
    v_baro_f = 0.0
    v_imu = 0.0
    v_fused = 0.0
    t_prev_us = time.ticks_us()

def calibrate():
    global alt0, az0_g
    print("\n[CMD 1] CALIBRACIÓN (mantén quieto 2-3s)")
    # Altura base
    alts = []
    for _ in range(30):
        alts.append(bmp.altitude_m(SEA_LEVEL_PA))
        time.sleep_ms(50)
    alt0 = mean(alts)

    # Baseline Z (gravedad + bias)
    azs = []
    for _ in range(200):
        gps.update()
        ax_g, ay_g, az_g, gx, gy, gz = mpu.read()
        azs.append(az_g)
        time.sleep_ms(5)
    az0_g = mean(azs)

    reset_states()
    print("OK -> alt0(abs)=%.2f m | az0=%.4f g\n" % (alt0, az0_g))

def read_altitude_once():
    # Baro
    alt_abs = bmp.altitude_m(SEA_LEVEL_PA)
    alt_rel = alt_abs - alt0
    return alt_abs, alt_rel

def update_velocities(alt_rel, az_g):
    # Actualiza v_baro_f, v_imu, v_fused usando dt real
    global alt_rel_f, alt_rel_prev, v_baro_f, v_imu, v_fused, t_prev_us

    # dt
    t_now_us = time.ticks_us()
    dt_us = time.ticks_diff(t_now_us, t_prev_us)
    t_prev_us = t_now_us
    dt = dt_us / 1_000_000.0 if dt_us > 0 else (IMU_LOOP_MS / 1000.0)
    if dt > 0.05:
        dt = 0.05

    # IMU integración (Z dinámica)
    az_mps2 = (az_g - az0_g) * G0
    v_imu += az_mps2 * dt

    # Altitud filtrada
    if alt_rel_f is None:
        alt_rel_f = alt_rel
        alt_rel_prev = alt_rel_f
    else:
        alt_rel_f = ALT_ALPHA * alt_rel + (1.0 - ALT_ALPHA) * alt_rel_f

    # Velocidad baro
    v_baro = (alt_rel_f - alt_rel_prev) / dt
    alt_rel_prev = alt_rel_f
    v_baro_f = VBARO_ALPHA * v_baro + (1.0 - VBARO_ALPHA) * v_baro_f

    # Fusión
    v_fused = W_IMU * v_imu + (1.0 - W_IMU) * v_baro_f

    return dt, az_mps2

def print_menu():
    print("Comandos:")
    print("  1) Calibrar (altura base + accel Z baseline)")
    print("  2) 10 datos: Altura relativa")
    print("  3) 10 datos: Giroscopio (X,Y,Z)")
    print("  4) 10 datos: Presión (Pa)")
    print("  5) Telemetría completa (salir con 'q' + Enter)")
    print("  m) Mostrar menú")
    print("")

def read_cmd_line():
    # En MicroPython, sys.stdout puede no tener flush()
    try:
        sys.stdout.write("CMD> ")
    except:
        print("CMD> ", end="")  # respaldo

    line = sys.stdin.readline()
    if not line:
        return ""
    return line.strip()

# =========================
# TELEMETRÍA (CMD 5) - sale con 'q'
# =========================
def telemetry_loop():
    print("\n[CMD 5] TELEMETRÍA COMPLETA (q + Enter para salir)\n")
    print("t(ms) | ALT_rel(m) | V_baro(m/s) V_imu(m/s) V_fused(m/s) | "
          "ACC(m/s^2) X Y Z | GYRO(dps) X Y Z | GPS fix sats lat lon alt(m) spd(m/s)")

    poll = uselect.poll()
    poll.register(sys.stdin, uselect.POLLIN)

    t_print_prev = time.ticks_ms()

    while True:
        # salir si entra 'q'
        if poll.poll(0):
            s = sys.stdin.readline().strip()
            if s.lower() == "q":
                print("\nSaliendo de telemetría.\n")
                return

        gps.update()

        # IMU
        ax_g, ay_g, az_g, gx, gy, gz = mpu.read()
        ax = ax_g * G0
        ay = ay_g * G0

        # Baro
        alt_abs, alt_rel = read_altitude_once()

        # Velocidades
        dt, az_dyn = update_velocities(alt_rel, az_g)

        # Print limitado
        if time.ticks_diff(time.ticks_ms(), t_print_prev) >= PRINT_PERIOD_MS:
            t_print_prev = time.ticks_ms()

            lat = "--" if gps.lat is None else ("%.6f" % gps.lat)
            lon = "--" if gps.lon is None else ("%.6f" % gps.lon)
            galt = "--" if gps.alt_m is None else ("%.1f" % gps.alt_m)
            gspd = "--" if gps.spd_mps is None else ("%.2f" % gps.spd_mps)

            print(
                "%6d | %9.2f | %10.2f %9.2f %11.2f | "
                "%8.2f %8.2f %8.2f | %8.1f %8.1f %8.1f | "
                "%3d %3d %s %s %s %s"
                % (
                    time.ticks_ms(),
                    alt_rel_f if alt_rel_f is not None else alt_rel,
                    v_baro_f, v_imu, v_fused,
                    ax, ay, az_dyn,
                    gx, gy, gz,
                    gps.fix, gps.sats,
                    lat, lon, galt, gspd
                )
            )

        time.sleep_ms(IMU_LOOP_MS)

# =========================
# COMANDOS 2-4 (10 muestras)
# =========================
def cmd_altitude_10():
    print("\n[CMD 2] 10 DATOS ALTURA (relativa)\n")
    for i in range(10):
        gps.update()
        alt_abs, alt_rel = read_altitude_once()
        print("%2d) ALT_rel = %8.2f m | ALT_abs = %8.2f m" % (i+1, alt_rel, alt_abs))
        time.sleep_ms(200)
    print("")

def cmd_gyro_10():
    print("\n[CMD 3] 10 DATOS GIROSCOPIO (°/s)\n")
    for i in range(10):
        gps.update()
        ax_g, ay_g, az_g, gx, gy, gz = mpu.read()
        print("%2d) GYRO X=%8.2f  Y=%8.2f  Z=%8.2f  (dps)" % (i+1, gx, gy, gz))
        time.sleep_ms(200)
    print("")

def cmd_pressure_10():
    print("\n[CMD 4] 10 DATOS PRESIÓN (Pa)\n")
    for i in range(10):
        gps.update()
        p = bmp.pressure_pa()
        print("%2d) P = %d Pa" % (i+1, p))
        time.sleep_ms(200)
    print("")

# =========================
# MAIN
# =========================
print_menu()
print("Sugerencia: ejecuta primero CMD 1 (calibrar)\n")

while True:
    cmd = read_cmd_line().lower()

    if cmd == "1":
        calibrate()
    elif cmd == "2":
        cmd_altitude_10()
    elif cmd == "3":
        cmd_gyro_10()
    elif cmd == "4":
        cmd_pressure_10()
    elif cmd == "5":
        telemetry_loop()
        print_menu()
    elif cmd == "m":
        print_menu()
    elif cmd in ("exit", "quit"):
        print("Saliendo.")
        break
    else:
        print("Comando no válido. Escribe 'm' para menú.\n")

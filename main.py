# PRUEBA DE ALTURA + GIROSCOPIO (GY-87)
# - BMP180/BMP085 (I2C 0x77): presión -> altitud
# - MPU6050 (I2C 0x68/0x69): giroscopio (°/s) + acelerómetro (g)
# - Calibra altura base (ref=0) al inicio
# - Imprime en consola cada 0.5 s

from machine import I2C, Pin
import time
import math

# ----------------------------
# BMP180
# ----------------------------
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

    def altitude_m(self, sea_level_pa=101325):
        p = self.pressure_pa()
        return 44330.0 * (1.0 - (p / sea_level_pa) ** 0.1903)

def mean(values):
    return sum(values) / len(values) if values else 0.0


# ----------------------------
# MPU6050 (GYRO)
# ----------------------------
MPU6050_ADDRS = (0x68, 0x69)

# Registros clave
MPU_PWR_MGMT_1   = 0x6B
MPU_GYRO_CONFIG  = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_XOUT_H = 0x3B  # aquí empieza bloque accel/temp/gyro

def _twos_complement_16(msb, lsb):
    v = (msb << 8) | lsb
    if v & 0x8000:
        v = -((v ^ 0xFFFF) + 1)
    return v

class MPU6050:
    def __init__(self, i2c, addr=0x68, gyro_fs=0, accel_fs=0):
        """
        gyro_fs:
          0 => ±250°/s  (131 LSB/°/s)
          1 => ±500°/s  (65.5)
          2 => ±1000°/s (32.8)
          3 => ±2000°/s (16.4)
        accel_fs:
          0 => ±2g   (16384 LSB/g)
          1 => ±4g   (8192)
          2 => ±8g   (4096)
          3 => ±16g  (2048)
        """
        self.i2c = i2c
        self.addr = addr
        self.gyro_fs = gyro_fs
        self.accel_fs = accel_fs

        # Despierta el MPU6050 (sale de sleep)
        self.i2c.writeto_mem(self.addr, MPU_PWR_MGMT_1, b"\x00")
        time.sleep_ms(50)

        # Configura rangos
        self.i2c.writeto_mem(self.addr, MPU_GYRO_CONFIG, bytes([(gyro_fs & 0x03) << 3]))
        self.i2c.writeto_mem(self.addr, MPU_ACCEL_CONFIG, bytes([(accel_fs & 0x03) << 3]))
        time.sleep_ms(10)

        # Factores de escala
        self._gyro_scale = [131.0, 65.5, 32.8, 16.4][gyro_fs & 0x03]
        self._accel_scale = [16384.0, 8192.0, 4096.0, 2048.0][accel_fs & 0x03]

    def read_raw(self):
        # Lee 14 bytes: accel(6) temp(2) gyro(6)
        b = self.i2c.readfrom_mem(self.addr, MPU_ACCEL_XOUT_H, 14)

        ax = _twos_complement_16(b[0],  b[1])
        ay = _twos_complement_16(b[2],  b[3])
        az = _twos_complement_16(b[4],  b[5])
        # temp_raw = _twos_complement_16(b[6], b[7])  # si lo necesitas
        gx = _twos_complement_16(b[8],  b[9])
        gy = _twos_complement_16(b[10], b[11])
        gz = _twos_complement_16(b[12], b[13])

        return ax, ay, az, gx, gy, gz

    def accel_g(self):
        ax, ay, az, _, _, _ = self.read_raw()
        return ax / self._accel_scale, ay / self._accel_scale, az / self._accel_scale

    def gyro_dps(self):
        _, _, _, gx, gy, gz = self.read_raw()
        return gx / self._gyro_scale, gy / self._gyro_scale, gz / self._gyro_scale


# ============================================================
# 1) I2C en ESP32-S3 (SDA=17, SCL=16)
# ============================================================
I2C_SDA = 17
I2C_SCL = 16
i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=100000)

print("\n=== PRUEBA GY-87: BMP180 + MPU6050 ===")
time.sleep_ms(300)

# ============================================================
# 2) Escaneo I2C
# ============================================================
devices = i2c.scan()
print("I2C scan:", [hex(d) for d in devices])

# BMP180
if BMP180_ADDR not in devices:
    print("ERROR: No veo el BMP180 en 0x77.")
    while True:
        time.sleep(1)

# MPU6050
mpu_addr = None
for a in MPU6050_ADDRS:
    if a in devices:
        mpu_addr = a
        break

if mpu_addr is None:
    print("ERROR: No veo el MPU6050 en 0x68 ni 0x69.")
    print("Revisa conexiones del GY-87 (VCC 3.3V, GND, SDA/SCL).")
    while True:
        time.sleep(1)

print("BMP180 OK (0x77). MPU6050 OK (%s)." % hex(mpu_addr))

# ============================================================
# 3) Inicializar sensores
# ============================================================
bmp = BMP180(i2c, addr=BMP180_ADDR, oss=0)
mpu = MPU6050(i2c, addr=mpu_addr, gyro_fs=0, accel_fs=0)  # ±250°/s, ±2g

# ============================================================
# 4) Calibrar altura base
# ============================================================
print("Calibrando altura base (ref=0)... mantén el módulo quieto.")
samples = []
for _ in range(30):
    samples.append(bmp.altitude_m(101325))
    time.sleep_ms(50)
alt0 = mean(samples)

print("Altura base (abs): %.2f m" % alt0)
print("Imprimiendo: Altura + Gyro(°/s) cada 0.5 s...\n")
time.sleep_ms(300)

# ============================================================
# 5) Loop principal
# ============================================================
while True:
    alt_abs = bmp.altitude_m(101325)
    alt_rel = alt_abs - alt0

    gx, gy, gz = mpu.gyro_dps()
    ax, ay, az = mpu.accel_g()

    # Una sola línea, útil para “telemetría” y graficar luego
    print(
        "Alt_rel:%7.2f m | Alt_abs:%7.2f m | "
        "Gyro(dps) X:%7.2f Y:%7.2f Z:%7.2f | "
        "Acc(g) X:%6.3f Y:%6.3f Z:%6.3f"
        % (alt_rel, alt_abs, gx, gy, gz, ax, ay, az)
    )

    time.sleep_ms(500)

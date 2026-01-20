
# PRUEBA DE ALTURA
# Lee presión con BMP180/BMP085 (I2C 0x77)
# Convierte a altitud 
# Calibra una altura base (referencia 0) al inicio
# imprime en el monitor serial la altura cada 0.5


from machine import I2C, Pin  # I2C y pines del ESP32-S3
import time                  # delays y temporización
import math                  # potencias/raíces (usado en fórmula de altitud)

# ----------------------------
# Dirección I2C del BMP180/BMP085
# En la gran mayoría de módulos BMP180, la dirección es 0x77
# ----------------------------
BMP180_ADDR = 0x77

# ----------------------------
# Registros importantes del BMP180
# (según datasheet)
# ----------------------------
REG_CALIB   = 0xAA  # inicio del bloque de calibración interno
REG_CONTROL = 0xF4  # registro de control: aquí se ordena "mide temp" o "mide presión"
REG_DATA    = 0xF6  # registro donde el sensor deja el resultado

CMD_TEMP     = 0x2E  # comando para medir temperatura
CMD_PRESSURE = 0x34  # comando base para medir presión (con OSS se ajusta precisión)

# ============================================================
# Clase BMP180: driver mínimo para MicroPython
# - Lee coeficientes de calibración del sensor
# - Lee temperatura y presión compensadas
# - Convierte presión a altitud
# ============================================================
class BMP180:
    def __init__(self, i2c, addr=BMP180_ADDR, oss=0):
        # i2c: bus I2C ya inicializado
        # addr: dirección I2C del sensor (normalmente 0x77)
        # oss: oversampling setting (0..3), a mayor => más preciso pero más lento
        self.i2c = i2c
        self.addr = addr
        self.oss = oss
        # Leer coeficientes del sensor (sin esto, presión/altitud salen mal)
        self._read_calibration()

    def _read_u16(self, reg):
        # Lee 2 bytes desde un registro y los interpreta como entero sin signo (0..65535)
        b = self.i2c.readfrom_mem(self.addr, reg, 2)
        return (b[0] << 8) | b[1]

    def _read_s16(self, reg):
        # Lee 2 bytes y los interpreta como entero con signo (-32768..32767)
        v = self._read_u16(reg)
        if v & 0x8000:
            v = -((v ^ 0xFFFF) + 1)
        return v

    def _read_calibration(self):
        # El BMP180 trae coeficientes en memoria interna para compensar la medición.
        # Los guardamos como atributos de la instancia.
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
        # Escribe 1 byte a un registro del sensor
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _read_ut(self):
        # UT = Uncompensated Temperature
        # 1) ordena medir temperatura
        # 2) espera un poco
        # 3) lee el resultado crudo (2 bytes)
        self._write8(REG_CONTROL, CMD_TEMP)
        time.sleep_ms(5)
        return self._read_u16(REG_DATA)

    def _read_up(self):
        # UP = Uncompensated Pressure
        # 1) ordena medir presión (con OSS)
        # 2) espera el tiempo recomendado
        # 3) lee 3 bytes y arma el valor crudo
        self._write8(REG_CONTROL, CMD_PRESSURE + (self.oss << 6))
        delay = [5, 8, 14, 26][self.oss]  # ms, aproximado por OSS
        time.sleep_ms(delay)

        b = self.i2c.readfrom_mem(self.addr, REG_DATA, 3)
        up = ((b[0] << 16) | (b[1] << 8) | b[2]) >> (8 - self.oss)
        return up

    def pressure_pa(self):
        # Devuelve presión en Pascales (Pa) usando compensación del datasheet.
        # Nota: internamente necesita leer temperatura también, porque la compensación
        # de presión depende de ella.
        ut = self._read_ut()
        up = self._read_up()

        # Cálculos estándar del datasheet del BMP180
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
        # Convierte presión a altitud usando la ecuación barométrica.
        # sea_level_pa: presión al nivel del mar (Pa). 101325 es estándar.
        p = self.pressure_pa()

        # Altitud aproximada (válida para cambios moderados)
        # 44330 * (1 - (p/p0)^0.1903)
        return 44330.0 * (1.0 - (p / sea_level_pa) ** 0.1903)

# Función auxiliar: promedio de lista
def mean(values):
    return sum(values) / len(values) if values else 0.0

# ============================================================
# 1) CONFIGURAR I2C EN EL ESP32-S3
#   - Usa GPIO 17 como SDA
#   - Usa GPIO 16 como SCL
#   - freq=100kHz (estándar I2C)
# ============================================================
I2C_SDA = 17
I2C_SCL = 16
i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=100000)

print("\n=== PRUEBA BMP180 (GY-87) ===")

# ============================================================
# 2) ESCANEAR DISPOSITIVOS I2C
#   - Esto confirma si el ESP32-S3 "ve" al sensor en el bus I2C
# ============================================================
devices = i2c.scan()
print("I2C scan:", [hex(d) for d in devices])

# Si no aparece 0x77, no tiene sentido continuar.
if BMP180_ADDR not in devices:
    print("ERROR: No veo el BMP180 en 0x77.")
    print("Revisa: VCC(3.3V), GND, SDA/SCL, y que el módulo esté bien soldado.")
    while True:
        time.sleep(1)

# ============================================================
# 3) INICIALIZAR BMP180
#   - Lee coeficientes internos
# ============================================================
bmp = BMP180(i2c, addr=BMP180_ADDR, oss=0)
print("BMP180 OK.")

# ============================================================
# 4) CALIBRAR ALTURA BASE (REFERENCIA)
#   - Leemos 30 muestras al inicio y promediamos
#   - Ese promedio será alt0 (altura "absoluta" actual)
#   - Luego la altura relativa será: alt_rel = alt_abs - alt0
# ============================================================
print("Calibrando altura base (ref=0)... mantén el módulo quieto.")
samples = []
for _ in range(30):
    samples.append(bmp.altitude_m(101325))
    time.sleep_ms(50)

alt0 = mean(samples)
print("Altura base (abs): %.2f m" % alt0)
print("A partir de aquí: Altura_rel = Altura_abs - Altura_base")
print("Imprimiendo altura cada 0.5 s...\n")

# ============================================================
# 5) BUCLE PRINCIPAL
#   - Lee altitud absoluta (según presión y p0=101325)
#   - Calcula altitud relativa respecto a alt0
#   - Imprime ambas cada 0.5 segundos
# ============================================================
while True:
    alt_abs = bmp.altitude_m(101325)  # altura estimada sobre nivel del mar
    alt_rel = alt_abs - alt0          # altura relativa a tu "cero" local

    print("Altura_rel: %7.2f m | Altura_abs: %7.2f m" % (alt_rel, alt_abs))

    time.sleep_ms(500)

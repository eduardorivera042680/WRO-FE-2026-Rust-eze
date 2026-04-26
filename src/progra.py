from machine import Pin, PWM, UART, I2C
import time
import math  # Para cálculos en PID

# --- CONFIGURACIÓN DE HARDWARE ---
# Dirección (Servo)
servo = PWM(Pin(13), freq=50)  # Pin 13 al Signal del Servo

# Tracción (TB6612FNG + Motor Pololu)
motor_pwm = PWM(Pin(12), freq=1000)
in1 = Pin(14, Pin.OUT)
in2 = Pin(27, Pin.OUT)
stby = Pin(26, Pin.OUT)
stby.on()  # Activar driver

# Sensores Ultrasónicos (HC-SR04)
trig = Pin(33, Pin.OUT)
echo_izq = Pin(32, Pin.IN)
echo_der = Pin(35, Pin.IN)

# Botones Reglamentarios
btn_encendido = Pin(0, Pin.IN, Pin.PULL_UP)  # Botón físico 1
btn_inicio = Pin(4, Pin.IN, Pin.PULL_UP)     # Botón físico 2 (inicio programa)

# Comunicación con ESP32-CAM
uart = UART(1, baudrate=115200, tx=17, rx=16)

# IMU (MPU6050 para detección de giros) - Asumiendo conexión I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
imu_addr = 0x68  # Dirección típica de MPU6050

# --- CONSTANTES ---
MAX_DIST = 400  # Distancia máxima en cm para sensores
TIMEOUT_US = 30000  # Timeout para eco en microsegundos
KP = 2.0  # Ganancia proporcional para PID
KI = 0.1  # Ganancia integral
KD = 0.5  # Ganancia derivativa
TARGET_DIST = 20  # Distancia objetivo entre sensores en cm
VEL_BASE = 600  # Velocidad base
VEL_OVERTAKE = 700  # Velocidad para rebasar
ANG_BASE = 90  # Ángulo base (recto)

# --- CLASE PID ---
class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.ticks_ms()

    def update(self, measurement):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.prev_time) / 1000.0  # en segundos
        if dt <= 0:
            dt = 0.01
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = now
        return output

# Instancia PID para dirección
pid_steer = PID(KP, KI, KD, setpoint=0)  # Setpoint 0 para centrado

# --- FUNCIONES DE CONTROL ---
def get_dist(echo_pin, timeout=TIMEOUT_US):
    trig.off()
    time.sleep_us(2)
    trig.on()
    time.sleep_us(10)
    trig.off()
    start = time.ticks_us()
    while echo_pin.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > timeout:
            return MAX_DIST  # Timeout, devolver distancia máxima
    t1 = time.ticks_us()
    while echo_pin.value() == 1:
        if time.ticks_diff(time.ticks_us(), t1) > timeout:
            return MAX_DIST
    t2 = time.ticks_us()
    dist = (time.ticks_diff(t2, t1) * 0.034) / 2
    return min(dist, MAX_DIST)  # Limitar distancia

def conducir(velocidad, angulo):
    # angulo 90 = recto, <90 izquierda, >90 derecha
    duty = int(40 + (angulo / 180) * 75)  # Ajustar según tu servo
    servo.duty(duty)
    
    if velocidad >= 0:
        in1.on()
        in2.off()
    else:
        in1.off()
        in2.on()
    motor_pwm.duty(abs(velocidad))

def leer_vision():
    comando = ""
    if uart.any():
        try:
            comando = uart.read().decode('utf-8').strip()
        except:
            comando = ""  # Error en decodificación
    return comando

def leer_distancias():
    d_izq = get_dist(echo_izq)
    d_der = get_dist(echo_der)
    return d_izq, d_der

def detectar_giro_completo():
    # Placeholder para detección de vuelta completa usando IMU
    # Leer datos del IMU (simplificado, necesitarías biblioteca para MPU6050)
    # Por ahora, usar un contador simple basado en tiempo (no preciso)
    # Implementar lectura real de giroscopio aquí
    # Ejemplo: si giro acumulado > 360 grados, return True
    return False  # Placeholder

# --- LÓGICA DE CARRERA ---
def carrera_autonoma():
    vueltas = 0
    print("Iniciando Reto...")
    
    while vueltas < 3:
        try:
            # 1. Leer visión de la ESP32-CAM
            comando = leer_vision()
            
            # 2. Leer distancias de seguridad
            d_izq, d_der = leer_distancias()
            
            # 3. Toma de decisiones (Estrategia Obstáculos)
            if "VERDE" in comando:
                conducir(VEL_OVERTAKE, 60)  # Rebasar por la izquierda
            elif "ROJO" in comando:
                conducir(VEL_OVERTAKE, 120)  # Rebasar por la derecha
            else:
                # Mantenerse centrado entre muros usando PID
                error = d_izq - d_der
                ajuste = ANG_BASE + pid_steer.update(error)
                angulo = max(60, min(120, ajuste))  # Limitar ángulo
                conducir(VEL_BASE, angulo)
            
            # 4. Conteo de vueltas
            if detectar_giro_completo():
                vueltas += 1
                print(f"Vuelta {vueltas} completada")
            
            time.sleep(0.1)  # Pequeño delay para estabilidad
        
        except Exception as e:
            print(f"Error en bucle principal: {e}")
            conducir(0, ANG_BASE)  # Detener en caso de error
            time.sleep(1)
    
    # Estacionamiento final
    conducir(0, ANG_BASE)
    print("Reto completado")

# --- FLUJO PRINCIPAL ---
try:
    while btn_inicio.value() == 1:
        print("Esperando botón de inicio...")
        time.sleep(0.5)
    
    carrera_autonoma()
except KeyboardInterrupt:
    print("Programa interrumpido")
    conducir(0, ANG_BASE)
except Exception as e:
    print(f"Error general: {e}")
    conducir(0, ANG_BASE)
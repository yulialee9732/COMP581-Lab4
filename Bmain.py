#!/usr/bin/env pybricks-micropython

## ESTE ES EL DEFINITIVO

# LAB 3 - BUG 2 CON M-LINE "GRUESA" (BANDA)
# Seguimiento de pared reciclado de main3.py (wall following con sonar a la izquierda)
#
# Coordenadas en mm. Origen del workspace en (0, 0).
# Salida: (500, 0) mirando hacia +y (perpendicular a la linea de salida).
# Objetivo: (2500, 2500), region de llegada de 300 mm de radio.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import math

# Initialize EV3
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize sensors
ultrasonic = UltrasonicSensor(Port.S3)
right_touch = TouchSensor(Port.S1)
left_touch = TouchSensor(Port.S4)
gyro = GyroSensor(Port.S2)

# ---------------------------------------------------------------
# CONSTANTES (mm, deg/s, ms)
# ---------------------------------------------------------------
WHEEL_DIAMETER = 49     # mm -> CALIBRAR
SPEED = 350               # velocidad de seguimiento de pared (la de main3, funciona)
GOAL_SPEED = 300          # velocidad en tramo libre hacia el objetivo (~145 mm/s)
MIN_DIST = 100            # rango de seguimiento de pared (igual que main3)
MAX_DIST = 120
FAR_DIST = 250            # pared perdida -> esquina exterior
KP = 1.5                  # control P de pared (igual que main3)
TARGET = (MIN_DIST + MAX_DIST) / 2

TIME_LIMIT = 175000       # 175 s (el limite es 180 s)

START_X = 500.0
START_Y = 0.0
GOAL_X = 2500.0
GOAL_Y = 2500.0
GOAL_RADIUS = 75         # la region es de 300 mm; 250 deja margen por error de odometria

# M-line como BANDA: el robot esta "sobre la m-line" si su distancia
# a la recta salida->objetivo es menor que M_BAND (banda de 2*M_BAND de ancho).
# Ademas se detecta el CRUCE (cambio de lado), asi que aunque la atraviese
# de golpe (p. ej. durante una maniobra de choque) tambien se entera.
M_BAND = 50               # semiancho -> banda de 10 cm
MIN_PROGRESS = 100        # al salir debe estar al menos 10 cm mas cerca del goal que al chocar
MIN_WALL_LEAVE = 200      # mm minimos rodeando antes de poder abandonar la pared
MAX_WALL = 7000           # si rodea mas de 7 m sin salir -> va directo al objetivo
# Vector unitario de la m-line
ML_LEN = math.sqrt((GOAL_X - START_X) ** 2 + (GOAL_Y - START_Y) ** 2)
UX = (GOAL_X - START_X) / ML_LEN
UY = (GOAL_Y - START_Y) / ML_LEN

# Correccion del objetivo segun cuantas veces abandona la pared sobre la m-line.
# La m-line (UX, UY) ya esta calculada con el objetivo real y NO cambia;
# solo se mueve el punto al que apunta el robot.
GOAL_REAL = 2500.0
GOAL_OFFSETS = [-250, 0, 250]   # 0 cruces -> 2250 | 1 cruce -> 2500 | 2 o mas -> 2750
mline_count = 0

# Pose (variables globales)
x = START_X
y = START_Y
prev_left = 0
prev_right = 0

# ---------------------------------------------------------------
# FUNCIONES RECICLADAS DE main3.py
# ---------------------------------------------------------------
def wait_for_center_button():
    # Muestra el giroscopio: si no se queda en 0 con el robot quieto, NO empezar
    while Button.CENTER not in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.print("Gyro:", gyro.angle())
        ev3.screen.print("Pulsa centro")
        wait(100)

def bumped():
    return left_touch.pressed() or right_touch.pressed()

def update_pose():
    global x, y, prev_left, prev_right
    left = left_motor.angle()
    right = right_motor.angle()
    dL = (left - prev_left) * math.pi * WHEEL_DIAMETER / 360
    dR = (right - prev_right) * math.pi * WHEEL_DIAMETER / 360
    prev_left = left
    prev_right = right
    d = (dL + dR) / 2
    theta = (90 - gyro.angle()) * math.pi / 180   # gyro 0 = mirando a +y
    x = x + d * math.cos(theta)
    y = y + d * math.sin(theta)
    return d

def stop():
    left_motor.brake()
    right_motor.brake()

def turn_right(deg):
    target = gyro.angle() + deg
    while abs(target - gyro.angle()) > 2:
        error = target - gyro.angle()
        speed = max(60, min(SPEED, abs(error) * 4))
        if error > 0:
            left_motor.run(speed)
            right_motor.run(-speed)
        else:
            left_motor.run(-speed)
            right_motor.run(speed)
        wait(5)
    stop()

def drive_distance(dist_mm, speed):
    start = (left_motor.angle() + right_motor.angle()) / 2
    target_deg = dist_mm * 360 / (math.pi * WHEEL_DIAMETER)
    while abs((left_motor.angle() + right_motor.angle()) / 2 - start) < target_deg:
        left_motor.run(speed)
        right_motor.run(speed)
        update_pose()
        wait(10)
    stop()

# ---------------------------------------------------------------
# FUNCIONES NUEVAS (objetivo y m-line)
# ---------------------------------------------------------------
def dist_to_goal():
    return math.sqrt((GOAL_X - x) ** 2 + (GOAL_Y - y) ** 2)

def at_goal():
    return dist_to_goal() < GOAL_RADIUS

def set_goal():
    global GOAL_X, GOAL_Y
    i = min(mline_count, len(GOAL_OFFSETS) - 1)
    GOAL_X = GOAL_REAL + GOAL_OFFSETS[i]
    GOAL_Y = GOAL_REAL + GOAL_OFFSETS[i]

def time_up():
    return timer.time() > TIME_LIMIT

def mline_side():
    # Distancia CON SIGNO a la m-line (+ a la izquierda de la recta, - a la derecha)
    return UX * (y - START_Y) - UY * (x - START_X)

def norm180(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def goal_heading():
    # Rumbo hacia el objetivo expresado en grados de giroscopio
    ang = math.atan2(GOAL_Y - y, GOAL_X - x) * 180 / math.pi
    return 90 - ang

def face_goal():
    update_pose()
    turn_right(norm180(goal_heading() - gyro.angle()))
    update_pose()

def go_to_goal():
    # Tramo libre: avanza hacia el objetivo corrigiendo rumbo con el giroscopio.
    # Como sale siempre desde la m-line y apunta al goal, va sobre la m-line.
    face_goal()
    while True:
        update_pose()
        if at_goal():
            return 'goal'
        if time_up():
            return 'time'
        if bumped():
            stop()
            update_pose()
            return 'bump'
        error = norm180(goal_heading() - gyro.angle())
        corr = max(-100, min(100, error * 5))
        left_motor.run(GOAL_SPEED + corr)
        right_motor.run(GOAL_SPEED - corr)
        wait(10)

def follow_wall():
    # Rodea el obstaculo (logica de main3) hasta encontrar la m-line
    # mas cerca del objetivo que el punto de choque.
    hit_x = x
    hit_y = y
    hit_goal = dist_to_goal()

    # Maniobra inicial de main3: atras 3 cm y giro a la derecha (pared a la izquierda)
    drive_distance(30, -SPEED)
    turn_right(90)
    update_pose()

    band = M_BAND
    wall_traveled = 0.0
    far_from_hit = False
    prev_side = mline_side()

    while True:
        d_step = update_pose()
        wall_traveled += abs(d_step)

        if at_goal():
            return 'goal'
        if time_up():
            return 'time'

        # --- DETECCION DE LA M-LINE (banda + cruce) ---
        side = mline_side()
        on_band = abs(side) < band
        crossed = (side > 0) != (prev_side > 0)
        prev_side = side
        closer = dist_to_goal() < hit_goal - MIN_PROGRESS

        if (on_band or crossed) and closer and wall_traveled > MIN_WALL_LEAVE:
            stop()
            ev3.speaker.beep(1500, 100)   # pitido: abandono la pared
            return 'leave'

        # --- SEGURIDADES ---
        # Si vuelve al punto de choque sin haber salido, se le escapo la m-line:
        # ensanchamos la banda.
        dh = math.sqrt((x - hit_x) ** 2 + (y - hit_y) ** 2)
        if dh > 300:
            far_from_hit = True
        elif far_from_hit and dh < 100:
            far_from_hit = False
            band = band * 2
            ev3.speaker.beep(400, 200)
        if wall_traveled > MAX_WALL:
            return 'giveup'

        # --- SEGUIMIENTO DE PARED (identico a main3) ---
        if bumped():
            stop()
            drive_distance(40, -SPEED)
            update_pose()
            turn_right(70)
            update_pose()
            drive_distance(40, SPEED)
            update_pose()
        else:
            u = ultrasonic.distance()
            if u > FAR_DIST:
                # pared perdida -> esquina exterior: curva cerrada a la izquierda
                left_motor.run(SPEED * 0.3)
                right_motor.run(SPEED)
            else:
                # control P
                error = u - TARGET
                turn = KP * error
                turn = max(-SPEED * 0.5, min(SPEED * 0.5, turn))
                left_motor.run(SPEED - turn)
                right_motor.run(SPEED + turn)
        wait(10)

# ---------------------------------------------------------------
# PROGRAMA PRINCIPAL
# ---------------------------------------------------------------
wait_for_center_button()
wait(2000)   # tiempo para retirar la mano; se puede bajar a 500 si el gyro va estable
gyro.reset_angle(0)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
prev_left = 0
prev_right = 0
x = START_X
y = START_Y
timer = StopWatch()
mline_count = 0
set_goal()                                # 0 cruces -> objetivo (2250, 2250)

result = 'go'
while True:
    result = go_to_goal()
    if result == 'bump':
        ev3.speaker.beep(800, 100)       # pitido corto: choque
        result = follow_wall()
        if result == 'leave':
            mline_count += 1              # ha detectado la m-line y sale de la pared
            set_goal()
        if result == 'leave' or result == 'giveup':
            continue                      # vuelve a ir hacia el objetivo
    break                                 # 'goal' o 'time'

stop()
if result == 'goal':
    ev3.speaker.beep(1000, 1000)          # llegada: parar y pitar
else:
    ev3.speaker.beep(300, 1000)           # se acabo el tiempo
ev3.screen.clear()
ev3.screen.print(result)
ev3.screen.print("t =", timer.time() // 1000, "s")
wait(3000)
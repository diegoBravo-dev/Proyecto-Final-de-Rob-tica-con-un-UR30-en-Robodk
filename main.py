from robodk.robolink import *
from robodk.robomath import Mat
from UR30_Class import UR30
from trayectorias import trazarTrayectoria
from numpy import deg2rad, array, arange
from time import sleep

def moverRobot(ur30, xd, yd, zd, ea_vec_d):
    
    siguiente_pos = ur30.CinematicaInversa(xd, yd, zd, ea_vec_d)

    t, q, _, _ = trazarTrayectoria(ur30.θ, siguiente_pos)

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    for i in range(len(t)):
        θ1 = q1[i]
        θ2 = q2[i]
        θ3 = q3[i]
        θ4 = q4[i]
        θ5 = q5[i]
        θ6 = q6[i]
        movimiento = [θ1, θ2, θ3, θ4, θ5, θ6]
        pintor.MoveJ(movimiento)

    ur30.actualizarDatos(movimiento)

def ciclo1(ur30):
    # PINTADO 1
    moverRobot(ur30, 0.983765, -0.775725, -0.177751, [2.04, -130.783, 18.771])

    # PINTADO 2
    moverRobot(ur30, 0.8815, -0.752587, -0.017867, [7.219, -157.969, 19.888])

    # PINTADO 3
    moverRobot(ur30, 0.583188, -0.77159, 0.235376, [2.069, -158.588, -7.824])

    # PINTADO 4
    moverRobot(ur30, 0.103227, -0.79372, 0.420775, [2.069, -158.588, -7.824])

    # PINTADO 5
    moverRobot(ur30, -0.2183, -0.808545, 0.544975, [2.069, -158.588, -7.824])

    # REACOMODO 1
    moverRobot(ur30, -0.10162, -0.790585, 0.375586, [-3.454, -142.225, 67.186])

    # PINTADO 6
    moverRobot(ur30, -0.674523, -0.851675, 0.567096, [9.117, -156.612, 30.231])

    # PINTADO 4
    moverRobot(ur30, -0.2183, -0.808545, 0.544975, [2.069, -158.588, -7.824])

    # PINTADO 6
    moverRobot(ur30, -0.674523, -0.851675, 0.567096, [9.117, -156.612, 30.231])

    # PINTADO 7
    moverRobot(ur30, -0.716224, -0.670846, 0.366238, [21.631, -156.590, 73.313])

    # PINTADO 6
    moverRobot(ur30, -0.674523, -0.851675, 0.567096, [9.117, -156.612, 30.231])

    # PINTADO 8
    moverRobot(ur30, -0.230463, -0.837501, 0.496142, [9.117, -156.612, 30.231])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

def ciclo2(ur30):
    # PINTADO 9
    moverRobot(ur30, 0.314938, -0.696655, 0.938676, [-6.369, -138.797, 57.914])

    # PINTADO 10
    moverRobot(ur30, 0.412844, -0.615818, 0.510272, [0.601, -126.421, 81.001])

    # PINTADO 11
    moverRobot(ur30, 0.992808, -0.84292, 0.363076, [-13.575, -129.645, 87.516])

    # PINTADO 12
    moverRobot(ur30, 0.524244, -0.739613, 0.229283, [-44.284, -132.524, 98.994])

    # PINTADO 13
    moverRobot(ur30, 0.779221, -0.842326, 0.088577, [-3.9, -108.924, 88.253])

    # PINTADO 14
    moverRobot(ur30, 0.259673, -0.711864, 0.090304, [-40.133, -141.201, 75.058])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

    # PINTADO 15
    moverRobot(ur30, -0.64738, -0.591189, 0.796154, [-12.212, -130.69, 95.134])

    # PINTADO 16
    moverRobot(ur30, -0.572491, -0.604381, 0.391407, [45.742, -105.646, 62.388])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

    # PINTADO 17
    moverRobot(ur30, 0.172576, -0.70438, 0.052343, [33.698, -146.812, 58.73])

    # PINTADO 18
    moverRobot(ur30, -0.103924, -0.711737, 0.223352, [33.698, -146.812, 58.73])

    # PINTADO 19
    moverRobot(ur30, -0.465534, -0.590842, 0.431859, [-5.074, -170.413, 27.077])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

def ciclo3(ur30):
    # PINTADO 21
    moverRobot(ur30, 0.315338, -0.594302, 0.443479, [81.674, -81.918, 69.714])

    # PINTADO 22
    moverRobot(ur30, 0.535846, -0.587976, 0.510684, [34.652, -130.103, 102.779])

    # PINTADO 23
    moverRobot(ur30, 0.653121, -0.584859, 0.939328, [27.408, -150.016, 57.01])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

    # PINTADO 24
    moverRobot(ur30, -0.330581, -0.677182, 0.756908, [-15.534, -142.868, 77.739])

    # PINTADO 25
    moverRobot(ur30, -0.404924, -0.645261, 0.543357, [-15.534, -142.868, 77.739])

    # PINTADO 26
    moverRobot(ur30, -0.761694, -0.752705, 0.417788, [11.636, 140.874, -76.96])

    # PINTADO 27
    moverRobot(ur30, -0.822491, -0.81472, 0.236519, [29.868, -135.511, 104.205])

    # PINTADO 28
    moverRobot(ur30, -0.747724, -0.798694, 0.213, [29.868, -135.511, 104.205])

    # PINTADO 29
    moverRobot(ur30, -0.7755, -0.875217, 0.072556, [30.894, -127.084, 116.693])

    # PINTADO 30
    moverRobot(ur30, -0.499842, -0.742036, 0.053185, [30.894, -127.084, 116.693])

    # PINTADO 31
    moverRobot(ur30, -0.51408, -0.74861, 0.139502, [29.868, -135.511, 104.205])

    # PINTADO 32
    moverRobot(ur30, -0.240305, -0.633946, 0.050503, [27.963, -146.021, 84.507])

    # PINTADO 33
    moverRobot(ur30, -0.101552, -0.644516, 0.076211, [29.281, -139.301, 97.755])

    # PINTADO 30
    moverRobot(ur30, -0.499842, -0.742036, 0.053185, [30.894, -127.084, 116.693])

    # PINTADO 34
    moverRobot(ur30, -0.588837, -0.777236, 0.222495, [30.894, -127.084, 116.693])

    # PINTADO 35
    moverRobot(ur30, -0.738369, -0.80929, 0.269533, [30.894, -127.084, 116.693])

    # REACOMODO 2
    moverRobot(ur30, -0.13863, -0.600752, 0.76055, [-16.117, -123.878, 106.955])

RDK = Robolink()

pintor = RDK.Item('UR30')
spray = RDK.Item('Generic Paint Sprayer')
chasis = RDK.Item('Chasis')

pintor.setTool(spray)

home = [0, -90, -90, 0, 90, 0]

rows = [
    [ 0.0, -1.0,  0.0, -500.0],
    [ 1.0,  0.0,  0.0, -1100.0],
    [ 0.0,  0.0,  1.0,  0.0],
    [ 0.0,  0.0,  0.0,  1.0]
]

mat = Mat(rows)

chasis.setPose(mat)

pintor.MoveJ(home)

ur30 = UR30(pintor.Joints().tolist())

print("--- OBTENIENDO POSICIÓN Y ORIENTACIÓN ACTUAL ----\n")
sleep(2)

print("Posición [x, y, z] en mm: ", ur30.Pos)
sleep(2)
print("Posición [rx, ry, rz]: ", ur30.Ori)
sleep(2)

print("\n--- MOVIENDO ROBOT A LA POSICIÓN INICIAL DE LA TAREA ----\n")

nueva_pos = ur30.posicionarRobot(deg2rad(array(pintor.Joints())))

pintor.MoveJ(nueva_pos)

sleep(2)

print("\n¡Movimiento completado!\n")

sleep(2)

print("--- EJECUTANDO CICLO 1 DE PINTADO ---\n")

sleep(2)

ciclo1(ur30)

rows = [
    [ 0.0, -1.0,  0.0, -500.0],
    [ 1.0,  0.0,  0.0, -1100.0],
    [ 0.0,  0.0,  1.0,  700.0],
    [ 0.0,  0.0,  0.0,  1.0]
]

mat = Mat(rows)

chasis.setPose(mat)

ciclo2(ur30)

rows = [
    [0.0, 1.0, 0.0, 500.0],
    [-1.0, 0.0, 0.0, -1100.0],
    [0.0, 0.0, 1.0, 700.0],
    [0.0, 0.0, 0.0, 1.0]
]

mat = Mat(rows)

chasis.setPose(mat)

ciclo3(ur30)

rows = [
    [ 0.0, 1.0, 0.0, 500.0],
    [ 1.0, 0.0, 0.0, -1100.0],
    [ 0.0, 0.0, 1.0, 0.0],
    [ 0.0, 0.0, 0.0, 1.0]
]

mat = Mat(rows)

chasis.setPose(mat)
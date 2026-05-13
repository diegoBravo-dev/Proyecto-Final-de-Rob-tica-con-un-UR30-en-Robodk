from robodk.robolink import Robolink
from UR30_Class import UR30
from trayectorias import trazarTrayectoria
from numpy import deg2rad, array
from time import sleep

RDK = Robolink()

pintor = RDK.Item('UR30')
spray = RDK.Item('Generic Paint Sprayer')

pintor.setTool(spray)

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

siguiente_pos = ur30.CinematicaInversa(-0.310192, -1.402855, 0.498155, [-22.583, -134.362, 85.773])

print(ur30.θ)

t, q, w, a = trazarTrayectoria(ur30.θ, siguiente_pos)

q1 = q[0]
q2 = q[1]
q3 = q[2]
q4 = q[3]
q5 = q[4]
q6 = q[5]

spray.setParam("Activate", 1)

for i in range(len(t)):
    θ1 = q1[i]
    θ2 = q2[i]
    θ3 = q3[i]
    θ4 = q4[i]
    θ5 = q5[i]
    θ6 = q6[i]
    movimiento = [θ1, θ2, θ3, θ4, θ5, θ6]
    pintor.MoveJ(movimiento)
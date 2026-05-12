from robodk.robolink import Robolink
from UR30_Class import UR30
from numpy import deg2rad, array
from time import sleep

RDK = Robolink()

limpiador = RDK.Item('UR30')

ur30 = UR30(limpiador.Joints().tolist())


print("--- OBTENIENDO POSICIÓN Y ORIENTACIÓN ACTUAL ----\n")
sleep(2)

print("Posición [x, y, z] en mm: ", ur30.Pos)
sleep(2)
print("Posición [rx, ry, rz]: ", ur30.Ori)
sleep(2)

print("\n--- MOVIENDO ROBOT A LA POSICIÓN INICIAL DE LA TAREA ----\n")

nueva_pos = ur30.posicionarRobot(deg2rad(array(limpiador.Joints())))

limpiador.MoveJ(nueva_pos)

sleep(2)

print("\n¡Movimiento completado!")

θ = limpiador.Joints().tolist()

print(θ)

ur30.CinematicaInversa(θ, -0.310192, -1.402855, 0.498155, [-22.583, -134.362, 85.773])
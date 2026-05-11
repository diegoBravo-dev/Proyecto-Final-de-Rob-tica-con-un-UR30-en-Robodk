from robodk.robolink import Robolink
from UR30_Class import UR30

RDK = Robolink()

limpiador = RDK.Item('UR30')

ur30 = UR30(limpiador.Joints().tolist())

print(ur30.Pos)
print(ur30.Ori)
from robodk.robolink import Robolink
from numpy import deg2rad, array, matrix, transpose
from numpy import arange, zeros, sin, cos, rad2deg

def f(t, θs):
    θ1 = θs[0]
    θ2 = θs[1]
    θ3 = θs[2]
    θ4 = θs[3]
    θ5 = θs[4]

    # Definir el control cinemático
    x = 0.1543*sin(θ1)*cos(θ5) + 0.201*sin(θ1) - 0.1543*sin(θ5)*cos(θ1)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.637*cos(θ1)*cos(θ2) - 0.5037*cos(θ1)*cos(θ2 + θ3)
    y = -0.1543*sin(θ1)*sin(θ5)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ1)*sin(θ2 + θ3 + θ4) - 0.637*sin(θ1)*cos(θ2) - 0.5037*sin(θ1)*cos(θ2 + θ3) - 0.1543*cos(θ1)*cos(θ5) - 0.201*cos(θ1)
    z = -0.637*sin(θ2) - 0.1543*sin(θ5)*sin(θ2 + θ3 + θ4) - 0.5037*sin(θ2 + θ3) - 0.1593*cos(θ2 + θ3 + θ4) + 0.2363

    X = array([[x], [y], [z]])

    # Jacobiano
    j11 = 0.1543*sin(θ1)*sin(θ5)*cos(θ2 + θ3 + θ4) - 0.1593*sin(θ1)*sin(θ2 + θ3 + θ4) + 0.637*sin(θ1)*cos(θ2) + 0.5037*sin(θ1)*cos(θ2 + θ3) + 0.1543*cos(θ1)*cos(θ5) + 0.201*cos(θ1)
    j12 = 0.637*sin(θ2)*cos(θ1) + 0.1543*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) + 0.5037*sin(θ2 + θ3)*cos(θ1) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
    j13 = 0.1543*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) + 0.5037*sin(θ2 + θ3)*cos(θ1) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
    j14 = 0.1543*sin(θ5)*sin(θ2 + θ3 + θ4)*cos(θ1) + 0.1593*cos(θ1)*cos(θ2 + θ3 + θ4)
    j15 = -0.1543*sin(θ1)*sin(θ5) - 0.1543*cos(θ1)*cos(θ5)*cos(θ2 + θ3 + θ4)
    j16 = 0

    j21 = 0.1543*sin(θ1)*cos(θ5) + 0.201*sin(θ1) - 0.1543*sin(θ5)*cos(θ1)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4)*cos(θ1) - 0.637*cos(θ1)*cos(θ2) - 0.5037*cos(θ1)*cos(θ2 + θ3)
    j22 = 0.637*sin(θ1)*sin(θ2) + 0.1543*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) + 0.5037*sin(θ1)*sin(θ2 + θ3) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
    j23 = 0.1543*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) + 0.5037*sin(θ1)*sin(θ2 + θ3) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
    j24 = 0.1543*sin(θ1)*sin(θ5)*sin(θ2 + θ3 + θ4) + 0.1593*sin(θ1)*cos(θ2 + θ3 + θ4)
    j25 = -0.1543*sin(θ1)*cos(θ5)*cos(θ2 + θ3 + θ4) + 0.1543*sin(θ5)*cos(θ1)
    j26 = 0

    j31 = 0
    j32 = -0.1543*sin(θ5)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4) - 0.637*cos(θ2) - 0.5037*cos(θ2 + θ3)
    j33 = -0.1543*sin(θ5)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4) - 0.5037*cos(θ2 + θ3)
    j34 = -0.1543*sin(θ5)*cos(θ2 + θ3 + θ4) + 0.1593*sin(θ2 + θ3 + θ4)
    j35 = -0.1543*sin(θ2 + θ3 + θ4)*cos(θ5)
    j36 = 0


    J = matrix([[j11, j12, j13, j14, j15, j16],
                [j21, j22, j23, j24, j25, j26],
                [j31, j32, j33, j34, j35, j36]])
    
    # Inversa / pseudoinversa (dependiendo del robot (n) y el problema/tarea (m))
    Jinv = J.getI()

    # Valores deseados de posición
    if t<2:
        xd = 0.6537
        yd = -0.201
        zd = 1.0326
    elif t>=2 and t<5:
        xd = 0.8187
        yd = -0.201
        zd = 1.0326
    elif t>=5 and t<20:
        zd = 1.1576 + 0.1*sin(t)
        yd = -0.201 + 0.1*cos(t)
        xd = 0.8187
    elif t>=20 and t<=25:
        xd = -0.4
        yd = -0.201
        zd = 1.0326
    elif t>=25 and t<=40:
        zd = 1.0326 + 0.1*sin(t)
        yd = -0.201 + 0.1*cos(t)
        xd = 0.2737
    else:
        xd = 0.6537
        yd = -0.201
        zd = 1.0326
    
    Xd = array([[xd], [yd], [zd]])

    # Valores deseados de velocidad
    if t<2:
        dxd = 0
        dyd = 0
        dzd = 0
    elif t>=2 and t<5:
        dxd = 0
        dyd = 0
        dzd = 0
    elif t>=5 and t<20:
        dzd = 0.1*cos(t)
        dyd = -0.1*sin(t)
        dxd = 0
    elif t>=20 and t<=25:
        dxd = 0
        dyd = 0
        dzd = 0
    elif t>=25 and t<=30:
        dxd = 0
        dyd = 0
        dzd = 0
    else:
        dxd = 0
        dyd = 0
        dzd = 0
    
    dXd = array([[dxd], [dyd], [dzd]])

    #Ganancias de control
    kx = 1
    ky = 2
    kz = 3

    K = matrix([[kx, 0, 0],
                [0, ky, 0],
                [0, 0, kz]])
    
    dq = Jinv @ (dXd - K@(X - Xd))
    
    return dq

def euler(f, ts, θs, h):

    q = zeros((len(ts), 6))
    
    q[0, :] = θs

    for i in range(len(ts) - 1):

        dq = f(ts[i], q[i, :])

        q[i+1, :] = q[i, :] + h * transpose(dq)

    t = array(ts)

    return t, q



RDK = Robolink()

ur30 = RDK.Item('UR30')

θs = deg2rad(array(ur30.Joints()))

ti = 0 
h = 0.03
tf = 40
ts = arange(ti, tf+h, h)

t,q = euler(f, ts, θs, h) 

# Desempaqueto solución
θ1 = q[:,0] 
θ2 = q[:,1]
θ3 = q[:,2] 
θ4 = q[:,3]
θ5 = q[:,4] 
θ6 = q[:,5]


for i in range(len(t)):
    q1 = rad2deg(θ1[i])
    q2 = rad2deg(θ2[i])
    q3 = rad2deg(θ3[i])
    q4 = rad2deg(θ4[i])
    q5 = rad2deg(θ5[i])
    q6 = rad2deg(θ6[i])
    home = [q1, q2, q3, q4, q5, q6]
    ur30.MoveJ(home)